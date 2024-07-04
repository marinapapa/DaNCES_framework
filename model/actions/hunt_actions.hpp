#ifndef HUNT_ACTIONS_HPP_INCLUDED
#define HUNT_ACTIONS_HPP_INCLUDED

#include <agents/prey.hpp>
#include "action_base.hpp"


namespace model {
	namespace actions {

			template <typename Agent>
			class position_to_attack
			{ // position (teleport) around target // PREY SPECIFIC (prey tag in)

					make_action_from_this(position_to_attack);

			public:
					position_to_attack() {}
					position_to_attack(size_t, const json& J)
					{
						rel_pos_ = J["rel_pos"];
					}

					float assess_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
					{	// returning a probability for whether choose this in a substate context
						return 0.f;
					}

					template <typename Sim>
					void on_entry(agent_type* self, size_t idx, tick_t T, const Sim& sim)
					{
						get_target_id(self, idx, sim);
						if (target_idx_ != -1) {
							const auto& target = sim.template pop<prey_tag>()[target_idx_];
							self->pos = target.H.global_pos(rel_pos_);
							self->dir = target.dir;
						}
					}

					void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
					{
					}

			private:

					void get_target_id(agent_type* self, size_t idx, const Simulation& sim)
					{
							target_idx_ = self->target;

							// if there is no target selected through select_group, chooses the closest one
							if (target_idx_ == -1)
							{
									const auto sv = sim.sorted_view<Tag, prey_tag>(idx);
									if (sv.size())
									{
											target_idx_ = sv[0].idx; // nearest prey
									}
							}
					}

					size_t target_idx_ = -1;           // [1]
					glm::vec3 rel_pos_;
			};

		template <typename Agent>
		class chase_closest_prey
		{ // go towards the closest individual from the group // PREY SPECIFIC (prey tag in)

			make_action_from_this(chase_closest_prey);

		public:
			chase_closest_prey() {}
			chase_closest_prey(size_t, const json& J)
			{
				w_ = J["w"];                       // [1]
				prey_speed_scale_ = J["prey_speed_scale"];                       // [1]
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
				const auto sv = sim.sorted_view<Tag, prey_tag>(idx);

				if (sv.size())
				{ 
					const auto& target = sim.pop<prey_tag>()[sv[0].idx]; // nearest prey
					auto ofss = math::ofs(self->pos, target.pos);;

					const auto Fdir = math::save_normalize(ofss, vec3(0.f)) * w_;
					self->steering += Fdir;
					self->speed = prey_speed_scale_ * target.speed;
				}
			}

		private:
			float w_ = 0;           // [1]
			float prey_speed_scale_ = 0; // speed in relation to the preys speed [1]
		};


		template <typename Agent>
		class lock_on_closest_prey
		{ // lock on to the closest individual at entry and go towards it for the rest of the action // PREY SPECIFIC (prey tag in)

			make_action_from_this(lock_on_closest_prey);

		public:
			lock_on_closest_prey() {}
			lock_on_closest_prey(size_t, const json& J)
			{
				w_ = J["w"];                       // [1]
				prey_speed_scale_ = J["prey_speed_scale"];    
				catch2_ = J["catch_dist"];
			}

			float assess_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{	// returning a probability for whether choose this in a substate context
				return 0.f;
			}

			void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
				target_idx_ = -1;		// in case no prey around
				min_dist2_ = std::numeric_limits<float>::max();
				const auto sv = sim.sorted_view<Tag, prey_tag>(idx);
				if (sv.size()) {
					target_idx_ = sv[0].idx; // nearest prey
				}
			}

			template <typename Sim>
			void operator()(agent_type* self, size_t idx, tick_t T, const Sim& sim)
			{
				if (target_idx_ != -1) {
					const auto& target = sim.template pop<prey_tag>()[target_idx_]; // nearest prey
					auto sd = self->H.local_pos(target.pos);
					min_dist2_ = std::min(min_dist2_, glm::length2(sd));
					if (min_dist2_ > catch2_) [[likely]] {
						if (sd.forward_coor > 0.f) {
							// prey is still in front
							auto F = w_ * glm::normalize(sd);
							const auto Fg = self->H.global_vec(F);
							self->steering += Fg;
						}
						self->speed = prey_speed_scale_ * target.speed;
					}
					else {
						// victory turn
						self->steering += 15.f * self->H.side();
					}
				}
			}

		private:
			glm::vec3 w_ = {};           // [1]
			float prey_speed_scale_ = 0; // speed in relation to the preys speed [1]
			size_t target_idx_ = -1; //
			float min_dist2_;
			float catch2_;
		};


		template <typename Agent>
		class avoid_closest_prey
		{ // go towards the closest individual from the group // PREY SPECIFIC (prey tag in)

			make_action_from_this(avoid_closest_prey);

		public:
			avoid_closest_prey() {}
			avoid_closest_prey(size_t, const json& J)
			{
				w_ = J["w"];                       // [1]
			}

			void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
			}

			template <typename Sim>
			void operator()(agent_type* self, size_t idx, tick_t T, const Sim& sim)
			{
				const auto sv = sim.template sorted_view<Tag, prey_tag>(idx);

				if (sv.size())
				{
					const auto& group_ind = sim.template pop<prey_tag>()[sv[0].idx]; // nearest prey
					auto ofss = math::ofs(group_ind.pos, self->pos);;

					const auto Fdir = math::save_normalize(ofss, vec3(0.f)) * w_;
					self->steering += Fdir;
				}
			}

		public:
			float w_ = 0;           // [1]
		};

	}
}

#endif