#pragma once

#include "state_base.hpp"

namespace model {
  namespace states {

    // Model of multi-state sub-state selector
    // we need to supply a specialization for an specific Agent
    template <typename Agent>
    struct sub_state_selector {
      // returns sub_state
      // friend of MultiState
      // we can have multiple specializations of this operator
      template <typename MultiState>
      size_t operator()(MultiState& multi_state, Agent* self, size_t idx, tick_t T, const Simulation& sim, const state_info_t* copy_state);
    };


    template <typename Agent, typename ... SubStates>
    class multi_state : public state<Agent> {
    public:
      using agent_type = Agent;
      using base_type = state<agent_type>;
      using Selector = sub_state_selector<Agent>;
      using substate_tuple = std::tuple<SubStates...>;

      static constexpr const char* name() noexcept { return "multi_state"; }
      std::string descr() const override { return descr_ + "::" + sub_states_[current_sub_state_]->descr(); }

      multi_state(size_t state_idx, size_t idx, const json& J) : 
        idx_(state_idx),
        selector_(J)
      {
        descr_ = J["description"];
        if (J.contains("copyable")) copyable_ = J["copyable"];
        if (sizeof...(SubStates) != J["sub_states"].size()) {
          throw std::runtime_error("number of sub-states doesn't match");
        }
        init_sub_state<0>(idx, J["sub_states"]);
      }

      float assess_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim) override {
          return 0.f;
      }

      state_info_t enter(agent_type* self, size_t idx, tick_t T, const Simulation& sim, const state_info_t* copy_state) override {
        if (copy_state) {
          assert(copy_state->state() == idx_);
          if (sub_states_[copy_state->sub_state()]->is_copyable()) {
            current_sub_state_ = copy_state->sub_state();
          }
        }
        else {
          current_sub_state_ = selector_(*this, self, idx, T, sim, idx_);
        }
        auto si = sub_states_[current_sub_state_]->enter(self, idx, T, sim, copy_state);
        return state_info_t(si.copyable(), idx_, si.state(), si.exit_tick());
      }

      float assess_substate(agent_type* self, size_t idx, tick_t T, const Simulation& sim, const size_t substate_idx)  override {
          return sub_states_[substate_idx]->assess_entry(self, idx, T, sim);
      }

      void resume(agent_type* self, size_t idx, tick_t T, const Simulation& sim) override {
        sub_states_[current_sub_state_]->resume(self, idx, T, sim);
      }

      bool is_copyable() const noexcept override { return copyable_; }
      size_t sub_states() const override { return num_substates(); }
      static constexpr size_t num_substates() noexcept { return sizeof...(SubStates); }

    private:
      template <size_t I>
      void init_sub_state(size_t idx, const json& J) {
        sub_states_[I] = std::make_unique<std::tuple_element_t<I, std::tuple<SubStates...>>>(I, idx, J[I]);
        if constexpr (I < sizeof...(SubStates) - 1) {
          init_sub_state<I + 1>(idx, J);
        }
      }

    private:
      friend Selector;
      std::array<std::unique_ptr<state<Agent>>, sizeof...(SubStates)> sub_states_;
      size_t current_sub_state_ = 0;
      bool copyable_ = false;
      Selector selector_;
      size_t idx_;            // self-index
      std::string descr_;
    };

  }

}
