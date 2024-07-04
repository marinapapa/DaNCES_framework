#ifndef AVOID_PRED_ACTIONS_HPP_INCLUDED
#define AVOID_PRED_ACTIONS_HPP_INCLUDED

#include <glmutils/ray.hpp>
#include <glmutils/random.hpp>
#include <model/while_topo.hpp>
#include "action_base.hpp"


namespace model {
  namespace actions {

	// PREDATOR - PREY INTERACTIONS 
	template <typename Agent>
	class avoid_p_position
	{ // avoid predators position

		make_action_from_this(avoid_p_position);

	public:
		avoid_p_position() {}
		avoid_p_position(size_t, const json& J)
		{
			float minsep = J["minsep"];       // [m]
			minsep2 = minsep * minsep;        // [m^2]
			w_ = J["w"];                       // [1]
		}

		void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
		{
		}

		void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
		{
			const auto nv = sim.sorted_view<Tag, pred_tag>(idx);

			// WITHOUT FOV APPLIED - ?
			if (nv.size() && (nv[0].dist2 < minsep2))
			{
				const auto& predator = sim.pop<pred_tag>()[nv[0].idx];    // nearest predator

				// check whether predator is coming from the left or right  
				//  and change the sign of the weight so that turn is in the opposite direction
				const int is_pred_left = self->H.hemisphere(predator.pos).z;
				auto w = static_cast<float>(is_pred_left) * w_;

				self->steering += glmutils::perpDot(self->dir) * w;
			}
		}

	public:
		float minsep2 = 0;  // [m^2]

	private:
		float w_ = 0;       // [1]
	};

	// PREDATOR - PREY INTERACTIONS 
	template <typename Agent>
	class move_away_from_predator
	{ // avoid predators position

			make_action_from_this(move_away_from_predator);

	public:
			move_away_from_predator() {}
			move_away_from_predator(size_t, const json& J)
			{
					float minsep = J["minsep"];       // [m]
					minsep2 = minsep * minsep;        // [m^2]
					w_ = J["w"];                       // [1]
			}

			void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
			}

			void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
					const auto nv = sim.sorted_view<Tag, pred_tag>(idx);

					if (nv.size() && (nv[0].dist2 < minsep2))
					{
							const auto& predator = sim.pop<pred_tag>()[nv[0].idx];    // nearest predator
							const auto away_predator = self->H.local(predator.pos);
							self->steering += away_predator * w_;
					}
			}

	public:
			float minsep2 = 0;  // [m^2]

	private:
			float w_ = 0;       // [1]
	};

	
  }
}


#endif