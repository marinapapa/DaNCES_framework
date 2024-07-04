#ifndef FLIGHT_HPP_INCLUDED
#define FLIGHT_HPP_INCLUDED

#include "action_base.hpp"


namespace model {
	namespace actions {

		template <typename Agent>
		class roost_attraction
		{ // level action, makes the individuals to turn 
			// towrads the roost after exiting the roost radius 
			// (does not affect the altitude)
			make_action_from_this(roost_attraction);

		public:
			roost_attraction() {}
			roost_attraction(size_t, const json& J)
			{
				roost_radius2_ = float(J["roost_radius"]);       // [m]
				roost_radius2_ *= roost_radius2_;  // [m^2]

				roost_plane_pos_.x = J["roost_pos_xz"][0];
				roost_plane_pos_.y = J["roost_pos_xz"][1];
				w_ = J["w"];
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
				const auto ofss = math::ofs(glm::vec2(self->pos.x, self->pos.z), roost_plane_pos_);
				const auto w = w_ * math::smootherstep(glm::length2(ofss), roost_radius2_, 500000.f);
				const glm::vec2 Fdir = math::save_normalize(ofss, glm::vec2(0.f)) * w;

				self->steering += vec3(Fdir.x, 0.f, Fdir.y);
			}

		private:
			float roost_radius2_ = 0;	  // [rad]
			glm::vec2 roost_plane_pos_ = { 0,0 };
			float w_ = 0;               // [1]
		};


		template <typename Agent>
		class altitude_attraction
		{ // altitude action, makes the individuals to pitch upwards or downwards 
			// (does not affect the x and z steering)
			make_action_from_this(altitude_attraction);

		public:
			altitude_attraction() {}
			altitude_attraction(size_t, const json& J)
			{
				pref_alt_ = float(J["pref_altitude"]);       // [m]
				smooth_range_ = float(J["smooth_range"]);       // [m]
				max_y_ = std::sin(glm::radians(float(J["max_pitch"])));		// max dir.y
				w_ = J["w"];
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
				const auto alt_dev = self->pos.y - pref_alt_;
				const auto y = -max_y_ * math::smootherstep_bipolar(alt_dev, -smooth_range_, +smooth_range_);		// desired
				const auto wup = w_ * (y - self->dir.y) * self->H.up();
				self->steering += wup;
			}

		private:
			float pref_alt_ = 0;				// [m] preferred altitude
			float smooth_range_ = 0;		// [m]
			float max_y_ = 0;				    // max 'y' - direction
			float w_ = 0;               // [1]
		};

		template <typename Agent>
		class level_attraction
		{ // makes the individuals to pitch upwards or downwards to return to level flight

				make_action_from_this(level_attraction);

		public:
				level_attraction() {}
				level_attraction(size_t, const json& J)
				{
						w_ = J["w"];
						max_y_ = std::sin(glm::radians(float(J["max_pitch"])));		// max dir.y
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
						const auto pitch_now = self->dir.y;
						const auto min_smooth = std::sin(glm::radians(3.f));		// max dir.y

						const auto w = w_ * math::smootherstep(abs(pitch_now), min_smooth, max_y_);
						self->steering += (pitch_now < 0) ? w * self->H.up() : -1.f * w * self->H.up();
									
				}

		private:
				float w_ = 0;               // [1]
				float max_y_ = 0;				    // max 'y' - direction
		};
	}
}


#endif
