#ifndef ROOSTING_HPP_INCLUDED
#define ROOSTING_HPP_INCLUDED

#include "action_base.hpp"


namespace model {
	namespace actions {

		template <typename Agent>
		class relative_roosting_transient
		{
			make_action_from_this(relative_roosting_transient);

		public:
			relative_roosting_transient() {}
			relative_roosting_transient(size_t, const json& J)
			{
				angl_to_home_ = glm::radians(float(J["home_direction"]));       // [deg]
				w_ = J["w"];                       // [1]
			}

			float assess_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{	// returning a probability for whether choose this in a substate context
				return 0.f;
			}

			void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
				//fl_id = sim.flock_of<Tag>(idx);
				//const auto& thisflock = sim.flocks<Tag>()[fl_id];

				// homing position relative to its flock current position
				const auto& this_flock = sim.groups<Tag>()[sim.group_of<Tag>(idx)];
				const auto& flock_pos = this_flock.gc();
				const auto& flock_head = math::save_normalize(this_flock.vel, vec3(0.f));
				home_pos_ = flock_pos + math::rotate(flock_head, angl_to_home_, self->H.up());
			}

			void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
				const auto ofss = math::ofs(self->pos, home_pos_);
				const vec3 Fdir = math::save_normalize(ofss, vec3(0.f)) * w_;
				self->steering += Fdir;
			}

		private:
			vec3 home_pos_ = { 0,0,0 };  // []
			float angl_to_home_ = 0;	  // [rad]
			float w_ = 0;               // [1]
		};

		template <typename Agent>
		class relative_roosting_persistant
		{
			make_action_from_this(relative_roosting_persistant);

		public:
			relative_roosting_persistant() {}
			relative_roosting_persistant(size_t, const json& J)
			{
				dist_to_home_ = float(J["home_dist"]);            // [m]
				angl_to_home_ = glm::radians(float(J["home_direction"]));       // [deg]
				w_ = J["w"];                       // [1]
			}

			void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
				//fl_id = sim.flock_of<Tag>(idx);
				//const auto& thisflock = sim.flocks<Tag>()[fl_id];

				// homing position relative to its flock current position
				const auto& this_flock = sim.groups<Tag>()[sim.group_of<Tag>(idx)];
				const auto& flock_pos = this_flock.gc();
				const auto& flock_head = math::save_normalize(this_flock.vel, vec3(0.f));
				home_pos_ = flock_pos + dist_to_home_ * math::rotate(flock_head, angl_to_home_, self->H.up());
			}

			void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
				const auto ofss = math::ofs(self->pos, home_pos_);
				const vec3 Fdir = math::save_normalize(ofss, vec3(0.f)) * w_;
				self->steering += Fdir;
			}

		private:
			vec3 home_pos_ = { 0,0,0 };  // []
			float dist_to_home_ = 0;    // [m]
			float angl_to_home_ = 0;	  // [rad]
			float w_ = 0;               // [1]
		};

		template <typename Agent>
		class roosting
		{
			make_action_from_this(roosting);

		public:
			roosting() {}
			roosting(size_t, const json& J)
			{
				home_pos_.x = J["home_pos"][0];    // [m]
				home_pos_.y = J["home_pos"][1];    // [m]
				home_pos_.z = J["home_pos"][2];    // [m]

				w_ = J["w"];                       // [1]
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
				const auto ofss = math::ofs(self->pos, home_pos_);
				const vec3 Fdir = math::save_normalize(ofss, vec3(0.f)) * w_;
				self->steering += Fdir;
			}

		private:
			vec3 home_pos_ = { 0,0,0 };  // []]
			float w_ = 0;               // [1]
		};

  }
}


#endif