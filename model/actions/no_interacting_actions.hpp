#ifndef OTHER_ACTIONS_HPP_INCLUDED
#define OTHER_ACTIONS_HPP_INCLUDED

#include "action_base.hpp"


namespace model {
  namespace actions {

	template <typename Agent>
	class waypoint
	{ // reach way-point

		make_action_from_this(waypoint);

	public:
		waypoint() {}
		waypoint(size_t, const json& J)
		{
			pos_.x = J["pos"][0];
            pos_.y = J["pos"][1];
            pos_.z = J["pos"][2];
			tolerance_ = J["tolerance"];
			tolerance_[0] = tolerance_[0] * tolerance_[0];
			tolerance_[1] = tolerance_[1] * tolerance_[1];
			tolerance_[2] = std::cos(glm::radians(tolerance_[2]));
			w_ = J["w"];
		}

		void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
		{
		}

		void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
		{
			auto ofs = math::ofs(self->pos, pos_);
			const auto Fdir = math::save_normalize(ofs, self->dir);
			self->steering += w_ * Fdir;
			const auto dd = glm::length2(ofs);
			const auto b = std::abs(glm::dot(self->dir, Fdir));
			if ((dd < tolerance_[0]) && ((dd < tolerance_[1]) || (b < tolerance_[2]))) {
				self->on_state_exit(idx, T, sim);
			}
		}

	public:
		vec3 pos_;
		float w_;           // [1]
		std::array<float, 3> tolerance_;
	};



    template <typename Agent>
    class wiggle
    {
      make_action_from_this(wiggle);

    public:
      wiggle() {}
      wiggle(size_t, const json& J)
      {
		    w_ = J["w"];               // [deg/s]
      }

      float assess_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
      {	// returning a probability for whether choose this in a substate context
          return 0.f;
      }

      void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
      {
      }

      void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
      {
        auto w = std::uniform_real_distribution<float>(-w_, w_)(reng); // [rad]
		    self->steering += self->H.side() * w;
      }

    private:
      float w_ = 0;      // [1] 

    };

    // constant radius turn 
    template <typename Agent>
    class r_turn
    {
      make_action_from_this(r_turn);

    public:
      r_turn() {}
      r_turn(size_t, const json& J)
      {
        turn_ = J["turn"];               // [deg]
        turn_ = glm::radians(turn_);     // [rad]
      }

      void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
      {
      }

      void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
      {
        // Fz = m * v*v/r
        auto Fz = self->ai.bodyMass * self->speed * self->speed / turn_;
		    self->steering += Fz * glmutils::perpDot(self->dir);
      }

    private:
      float turn_ = 0; // [rad/s]
      float w_ = 0;      // [1] NOT USED BUT IMPORTANT FOR NORMALIZING ALL WEIGHTS
    };
  }
}


#endif