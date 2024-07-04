#ifndef MODEL_STATES_PERSISTENT_HPP_INCLUDED
#define MODEL_STATES_PERSISTENT_HPP_INCLUDED

#include "state_base.hpp"


namespace model {
  namespace states {


    template <typename IP>
    class persistent : public state<typename IP::agent_type>
    {
      make_state_from_this(persistent);
    
    public:
      persistent(size_t state_idx, size_t idx, const json& J) :
        idx_(state_idx),
        actions(IP::create(idx, J["actions"])), 
        duration_(static_cast<tick_t>(double(J["duration"]) / Simulation::dt())) // [tick]
      {
        descr_ = J["description"];
        if (J.contains("copyable")) copyable_ = J["copyable"];
        effective_dur_ = duration_;
	    sai_ = flight::create_state_aero<float>(J["aeroState"]);
        tr_ = std::max(tick_t(1), static_cast<tick_t>(double(J["tr"]) / Simulation::dt())); // [tick]
      }

      float assess_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim) override {
          chain_assess_entry<0>(self, idx, T, sim);
          float state_potential = *std::max_element(actions_potential_.begin(), actions_potential_.end());
          return state_potential;
      }

      float assess_substate(agent_type* self, size_t idx, tick_t T, const Simulation& sim, const size_t substate_idx)  override {
          return 0.f;
      }

      state_info_t enter(agent_type* self, size_t idx, tick_t T, const Simulation& sim, const state_info_t* copy_state) override
      {
        t_exit_ = copy_state ? copy_state->exit_tick() : (T + duration_);
        chain_on_entry<0>(self, idx, T, sim);
        return state_info_t(is_copyable(), idx_, 0, t_exit_);
      }

      void resume(agent_type* self, size_t idx, tick_t T, const Simulation& sim) override
      {
	      self->reaction_time = tr_;
   	    self->sa = sai_;

        chain_actions<0>(self, idx, T, sim);
        if (T >= t_exit_) {
          effective_dur_ = duration_;
          self->on_state_exit(idx, T, sim);
        }
      };

    public:
      tick_t t_exit_;
    protected:
      tick_t tr_;        // [tick]
      tick_t duration_;  // [tick]
      tick_t effective_dur_;
	  flight::state_aero<float> sai_; // state specific aero info
      size_t idx_;       // self-index
      std::vector<float> actions_potential_;
    };

  }
}

#endif
