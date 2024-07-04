#ifndef MODEL_STATE_TRANSIENT_HPP_INCLUDED
#define MODEL_STATE_TRANSIENT_HPP_INCLUDED

#include "state_base.hpp"


namespace model {
  namespace states {


    template <typename IP>
    class transient : public state<typename IP::agent_type>
    {
      make_state_from_this(transient);

    public:
      explicit transient(size_t state_idx, size_t idx, const json& J) :
        idx_(state_idx), actions(IP::create(idx, J["actions"]))
	    {
        descr_ = J["description"];
        if (J.contains("copyable")) copyable_ = J["copyable"];
	    	sai_ = flight::create_state_aero<float>(J["aeroState"]);
        tr_ = std::max(tick_t(1), static_cast<tick_t>(double(J["tr"]) / Simulation::dt())); // [tick]
        //normalize_actions<0>();
      }
      float assess_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim) override {
          return 0.f;
      }

      float assess_substate(agent_type* self, size_t idx, tick_t T, const Simulation& sim, const size_t substate_idx)  override {
          return 0.f;
      }

      state_info_t enter(agent_type* self, size_t idx, tick_t T, const Simulation& sim, const state_info_t* copy_state) override
      {
        if (tr_ < 1) throw std::runtime_error("Reaction time smaller than 1");
        chain_on_entry<0>(self, idx, T, sim);
        return state_info_t(is_copyable(), idx_, 0, -1);
      }

      void resume(agent_type* self, size_t idx, tick_t T, const Simulation& sim) override
      {
		    self->reaction_time = tr_;
        self->sa = sai_;

        chain_actions<0>(self, idx, T, sim);
        self->on_state_exit(idx, T, sim);
      };

    protected: 
      tick_t tr_;  // [tick]
	    flight::state_aero<float> sai_; // state specific aero info
      size_t idx_; // self-index
      std::vector<float> actions_potential_;
	};

  } 
}

#endif
