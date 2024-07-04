#ifndef ESCAPE_ACTIONS_HPP_INCLUDED
#define ESCAPE_ACTIONS_HPP_INCLUDED

#include <glmutils/ray.hpp>
#include <glmutils/random.hpp>
#include <model/while_topo.hpp>
#include <agents/predator.hpp>
#include "action_base.hpp"


namespace model {
		namespace actions {

				// PREDATOR - PREY INTERACTIONS 
				template <typename Agent>
				class dive
				{ // avoid predators position

						make_action_from_this(dive);

				public:
						dive() {}
						dive(size_t, const json& J)
						{
								float minsep = J["minsep"];       // [m]
								minsep2_ = minsep * minsep;        // [m^2]
								w_ = J["w"];                       // [1]
								max_dive_ = J["max_dive"];                       // [1]
								selected_prob_ = optional_json<float>(J, "select_prob").value_or(1.f);
						}

						float assess_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
						{	// returning a probability for whether choose this in a substate context
							return selected_prob_;
						}
						void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
						{
								init_y_ = self->pos.y;
						}

						void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
						{
								const auto nv = sim.sorted_view<Tag, pred_tag>(idx);
								const auto dist2 = nv[0].dist2;
								// WITHOUT FOV APPLIED - ?
								if (nv.size())
								{
										auto rot_axis = self->H.up();
										const auto pitch = w_ * (math::smootherstep(dist2, 0.f, minsep2_) - 1.f);
										if (abs(self->pos.y - init_y_) > max_dive_)
										{
												rot_axis = math::rotate(self->dir, 1.5f * math::pi<float>, self->H.side());
										}
										self->steering += pitch * rot_axis;
								}
						}

				private:
						float w_ = 0;       // [1]
						float minsep2_ = 0;  // [m^2]
						float max_dive_ = 0;
						float init_y_ = 0;
				};

				//turn in time as reaction to a predator
				template <typename Agent>
				class t_turn_pred
				{
						make_action_from_this(t_turn_pred);

				public:
						t_turn_pred() {}
						t_turn_pred(size_t, const json& J)
						{
								turn_ = glm::radians(float(J["turn"]));
								time_ = J["time"];

								if (time_ == 0.f || turn_ == 0.f) throw std::runtime_error("wrong parameters in t_turn");
						}

						void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
						{
								// we want to turn turn_ radians in time_ seconds.
								auto w = turn_ / time_;       // required angular velocity
								r_ = self->speed / w;       // radius

								// find direction away from predator
								const auto nv = sim.sorted_view<Tag, pred_tag>(idx);

								if (nv.size())
								{
										const auto& predator = sim.pop<pred_tag>()[nv[0].idx];    // nearest predator
										auto dir_away = glm::normalize(math::ofs(predator.pos, self->pos));
										w_ = (glmutils::perpDot(self->dir, dir_away) > 0) ? 1.f : -1.f; // perp dot positive, b on right of a (for perpdot(a,b))
								}
								else
								{
										w_ = 0.f;
								}

						}

						void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
						{
								// Fz = m * v*v/r 
								turn_dir_ = w_ * self->H.side();
								auto Fz = self->ai.bodyMass * self->speed * self->speed / r_;
								self->steering += Fz * turn_dir_;
						}

				private:
						float r_ = 0;
						vec3 turn_dir_;
						float turn_ = 0;   // [rad]
						float time_ = 0;
						float w_ = 0;      // [1] 
				};


				//turn random degrees within a window in given time as reaction to a predator
				template <typename Agent>
				class random_t_turn_pred
				{
						make_action_from_this(random_t_turn_pred);

				public:
						random_t_turn_pred() {}
						random_t_turn_pred(size_t, const json& J)
						{
								wp_ = J["wp"];
								ad_ = J["altitude_dev"];

								const float turn_min = glm::radians(float(J["turn_min"]));
								const float turn_max = glm::radians(float(J["turn_max"]));
								const float time_min = J["time_min"];
								const float time_max = J["time_max"];

								if (time_min <= 0.f || time_max <= 0.f || turn_max == 0.f) throw std::runtime_error("wrong parameters in random_t_turn");

								turn_distr_ = std::uniform_real_distribution<float>(turn_min, turn_max);
								time_distr_ = std::uniform_real_distribution<float>(time_min, time_max);
								turn_dur_ = static_cast<tick_t>(time_min / Simulation::dt());

								selected_prob_ = optional_json<float>(J, "select_prob").value_or(1.f);
						}

						float assess_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
						{	// returning a probability for whether choose this in a substate context

							return selected_prob_;
						}

						void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
						{
								// we want to turn turn_ radians in time_ seconds.
								const auto thisturn = turn_distr_(model::reng);
								auto loc_time = time_distr_(model::reng);
								turn_dur_ = static_cast<tick_t>(static_cast<double>(loc_time) / Simulation::dt());

								auto w = thisturn / loc_time;       // required angular velocity
								r_ = self->speed / w;       // radius

								// find direction away from predator
								const auto nv = sim.sorted_view<Tag, pred_tag>(idx);

								if (nv.size())
								{
										const auto& predator = sim.pop<pred_tag>()[nv[0].idx];    // nearest predator
										// check whether predator is coming from the left or right  
										//  and change the sign of the weight so that turn is in the opposite direction
										const float is_pred_left = self->H.hemisphere(predator.pos).z;
										w_ = -1.f * static_cast<float>(is_pred_left);
								}
								else
								{
										w_ = 0.f;
								}
						}

						void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
						{
								// Fz = m * v*v/r 
								turn_dir_ = w_ * self->H.side();
								auto Fz = self->ai.bodyMass * self->speed * self->speed / r_;
								//	const auto pitch = wp_ * (1.f - 2.f * glm::smoothstep(-ad_, +ad_, ofs.y));

								self->steering += Fz * turn_dir_;

						}

				private:
						float r_ = 0;
						vec3 turn_dir_;
						tick_t turn_dur_;
						std::uniform_real_distribution<float> turn_distr_;
						std::uniform_real_distribution<float> time_distr_;
						float w_ = 0;      // [1] 
						float wp_, ad_;

				};

				//turn random degrees within a window in given time as reaction to a predator
				template <typename Agent>
				class random_t_turn_gamma_pred
				{
						make_action_from_this(random_t_turn_gamma_pred);

				public:
						random_t_turn_gamma_pred() {}
						random_t_turn_gamma_pred(size_t, const json& J)
						{
								const float turn_mean = glm::radians(float(J["turn_mean"]));
								const float turn_sd = glm::radians(float(J["turn_sd"]));
								const float time_mean = J["time_mean"];
								const float time_sd = J["time_sd"];

								if (turn_mean <= 0.f || turn_sd <= 0.f || time_mean <= 0.f || turn_mean == 0.f) throw std::runtime_error("wrong parameters in random_t_turn");

								const float turn_alpha = (turn_mean / turn_sd) * (turn_mean / turn_sd);
								const float turn_beta = (turn_sd * turn_sd) / turn_mean;

								const float time_alpha = (time_mean / time_sd) * (time_mean / time_sd);
								const float time_beta = (time_sd * time_sd) / time_mean;

								turn_distr_ = std::gamma_distribution<float>(turn_alpha, turn_beta);
								time_distr_ = std::gamma_distribution<float>(time_alpha, time_beta);

								turn_dur_ = static_cast<tick_t>(turn_mean / Simulation::dt());

								selected_prob_ = optional_json<float>(J, "select_prob").value_or(1.f);

						}

						float assess_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
						{	// returning a probability for whether choose this in a substate context

							return selected_prob_;
						}

						void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
						{
								// we want to turn turn_ radians in time_ seconds.
								auto loc_time = 0.f; // random to initialize
								auto thisturn = 0.f; // random to initialize
								do {
										loc_time = time_distr_(model::reng);
										thisturn = turn_distr_(model::reng);
								} while (loc_time * thisturn <= 0.f); // both not 0

								turn_dur_ = static_cast<tick_t>(static_cast<double>(loc_time) / Simulation::dt());

								auto w = thisturn / loc_time;       // required angular velocity
								r_ = self->speed / w;       // radius

								// find direction away from predator
								const auto nv = sim.sorted_view<Tag, pred_tag>(idx);

								if (nv.size())
								{
										const auto& predator = sim.pop<pred_tag>()[nv[0].idx];    // nearest predator
										// check whether predator is coming from the left or right  
										//  and change the sign of the weight so that turn is in the opposite direction
										const int is_pred_left = glm::sign(self->H.local_pos(predator.pos)).side_coor; 
										w_ = -1.f * static_cast<float>(is_pred_left);
								}
								else
								{
										w_ = 0.f;
								}
						}

						void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
						{
								// Fz = m * v*v/r 
								turn_dir_ = w_ * self->H.side();
								auto Fz = self->ai.bodyMass * self->speed * self->speed / r_;
								self->steering += Fz * turn_dir_;

						}

				private:
						float r_ = 0;
						vec3 turn_dir_;
						tick_t turn_dur_;
						std::gamma_distribution<float> turn_distr_;
						std::gamma_distribution<float> time_distr_;
						float w_ = 0;      // [1] 
				};

				template <typename Agent>
				class copy_escape
				{ // cohere by turning with all neighbors

						make_action_from_this(copy_escape);

				public:
						copy_escape() {}
						copy_escape(size_t, const json& J)
						{
								topo = J["topo"];    // [1]
								//cfov = glm::cos(glm::radians(180.0f)); // [1]
								float fov = J["fov"]; // [deg]
								cfov = glm::cos(glm::radians(180.0f - 0.5f * (360.0f - fov))); // [1]

								float maxdist = J["maxdist"];     // [m]
								maxdist2 = maxdist * maxdist;     // [m^2]
						}

						void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
						{
						}

						void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
						{
							const auto sv = sim.sorted_view<Tag>(idx);
							const auto& flock = sim.pop<Tag>();

							const auto n = std::min(sv.size(), topo);
							for (auto it = sv.cbegin(); it != sv.cbegin() + n; ++it) {
								if (in_fov(self, it->dist2, flock[it->idx].pos, this) && it->state_info.copyable()) {
									self->copied_state = it->state_info;
									break;
								}
							}
						}

				public:
						size_t topo = 0;        // [1]
						float cfov = 0;         // [1]
						float maxdist2 = 0;     // [m^2]
				};

				//turn in time as reaction to a predator
				template <typename Agent>
				class zig_zag
				{
						make_action_from_this(zig_zag);

				public:
						zig_zag() {}
						zig_zag(size_t, const json& J)
						{
								turn_ = glm::radians(float(J["turn"]));
								time_ = J["time"];
								min_sep2_ = J["minsep"];
								min_sep2_ *= min_sep2_;

								if (time_ == 0.f || turn_ == 0.f) throw std::runtime_error("wrong parameters in zig zag");

								zig_timer_ = static_cast<tick_t>(time_ / (Simulation::dt() * 2.f)); // [ticks for each subturn]
								w_ = 1.f; // control change in direction

								selected_prob_ = optional_json<float>(J, "select_prob").value_or(1.f);
						}

						float assess_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
						{	// returning a probability for whether choose this in a substate context

							return selected_prob_;
						}

						void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
						{
								// to signal exit of first zig
								entry_tick_ = T;

								// we want to turn turn_ radians in time_ seconds.
								auto w = 2.f * turn_ / time_;       // required angular velocity
								r_ = self->speed / w;       // radius

								// find direction away from predator
								const auto nv = sim.sorted_view<Tag, pred_tag>(idx);

								// in case there is no predator - switch to runtime error?
								if (nv.size())
								{
										const auto& predator = sim.pop<pred_tag>()[nv[0].idx];    // nearest predator
										auto dir_away = glm::normalize(math::ofs(predator.pos, self->pos));
										w_ = (glmutils::perpDot(self->dir, dir_away) > 0) ? 1.f : -1.f; // dot positive, b on right of a (for dot(a,b))
								}
								else
								{
										w_ = 0.f; // no predator in the simulation
								}

						}
						 
						void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
						{
								// Fz = m * v*v/r
								turn_dir_ = w_ * self->H.side();

								auto Fz = self->ai.bodyMass * self->speed * self->speed / r_;
								self->steering += Fz * turn_dir_;

								if ((T - entry_tick_) > zig_timer_)
								{
										w_ = -w_;
										entry_tick_ = T;		// restart count for next zig
								}
						}

				private:
						float r_ = 0;
						vec3 turn_dir_;
						tick_t zig_timer_ = 0; // count when it finished the first turn to switch turn_dir [ticks]
						tick_t entry_tick_ = 0;
						float turn_ = 0;   // [rad]
						float time_ = 0;
						float w_ = 0;      // [1]
						float min_sep2_ = 0; // distance to start zig zag, so that closest individual starts
				};


		template <typename Agent>
		class scatter
		{ // avoid perpendicular to predators offset

			make_action_from_this(scatter);

		public:
			scatter() {}
			scatter(size_t, const json& J)
			{
				float tdist = J["threshold_dist"];
				tdist2_ = tdist * tdist;
				w_ = J["w"];                 
			}

			float assess_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{	// returning a probability for whether choose this in a substate context
				
				// if predator too close, very high value
				const auto nv = sim.sorted_view<Tag, pred_tag>(idx);
				if (nv.size()) {
					const auto& predator = sim.pop<pred_tag>()[nv[0].idx];
					const auto sd = self->H.local_pos(predator.pos);		// a.k.a. signed distance
					if (glm::length2(sd) < tdist2_) {
						return 10.f; // very high value
					}
				}
				return 0.f;
			}

			void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
			}

			void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
				const auto nv = sim.sorted_view<Tag, pred_tag>(idx);
				if (nv.size()) {
					if (nv[0].dist2 < tdist2_) {
						const auto& predator = sim.pop<pred_tag>()[nv[0].idx];
						const auto sd = self->H.local_pos(predator.pos);		// a.k.a. signed distance
						const auto Fl = w_ * glm::sign(sd);
						self->steering -= self->H.global_vec(Fl);
					}
				}
			}

		private:
			glm::vec3 w_ = {};			// [1]
			float tdist2_ = 0;		  // [m^2]
		};


		}
}


#endif
