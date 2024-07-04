#ifndef PREDATOR_ACTIONS_HPP_INCLUDED
#define PREDATOR_ACTIONS_HPP_INCLUDED


#include "action_base.hpp"


namespace model {

  namespace actions {


		template <typename Agent>
		class set
		{
			make_action_from_this(set);

		public:
			set() {}
			set(size_t, const json& J)
			{
				pos_.x = J["pos"][0];
				pos_.y = J["pos"][1];
				pos_.z = J["pos"][2];
				dir_.x = J["dir"][0];
				dir_.y = J["dir"][1];
				dir_.z = J["dir"][2];
				speed_ = J["speed"];
				dir_ = math::save_normalize(dir_, vec3(1, 0, 0));
			}

			void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
			}

			void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
				self->pos = pos_;
				self->dir = dir_;
				self->speed = speed_;
			}

		private:
			vec3 pos_;
			vec3 dir_;
			float speed_;
		};


		template <typename Agent>
		class set_retreat
		{
			make_action_from_this(set_retreat);

		public:
			set_retreat() {}
			set_retreat(size_t, const json& J)
			{
				dist_away_ = J["distAway"];
				speed_ = J["speed"];
			}

			void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{

			}

			void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
				self->pos = self->pos - dist_away_ * self->dir;
				self->dir = -self->dir;
				self->speed = speed_;
			}

		private:
			float dist_away_;
			float speed_;
		};


		template <typename Agent>
		class hold
		{ // circle around position

			make_action_from_this(hold);

		public:
			hold() {}
			hold(size_t, const json& J)
			{
				pos_.x = J["pos"][0];
				pos_.y = J["pos"][1];
				pos_.z = J["pos"][2];
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
				auto ofs = math::ofs(self->pos, pos_);
				const auto Fdir = math::save_normalize(ofs, self->dir) * w_;
				self->steering += Fdir;
			}

		public:
			vec3 pos_;
			float w_;
		};

		template <typename Agent>
		class hold_current
		{ // circle around position

			make_action_from_this(hold_current);

		public:
			hold_current() {}
			hold_current(size_t, const json& J)
			{
				w_ = J["w"];
				ad_ = J["altitude_dev"];
			}
		
			float assess_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{	// returning a probability for whether choose this in a substate context
				return 0.f;
			}
			void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
				pos_ = self->pos;
			}

			void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
   			const auto ad = self->H.local_pos(pos_).up_coor;
				const auto pitch = w_.up_coor * (1.f - 2.f * glm::smoothstep(-ad_, +ad_, ad));
				self->steering += w_.side_coor * self->H.side() 
					              - pitch * self->H.up();
			}

		public:
			glm::vec3 pos_ = { 0,0,0 };
			glm::vec3 w_;
			float ad_;
		};


		template <typename Agent>
		class select_group
		{
			make_action_from_this(select_group);

			enum class Selection {
				Nearest,
				Biggest,
				Smallest,
				Random,
				MaxSelection
			};
		public:
			select_group() {}
			select_group(size_t, const json& J)
			{
				std::array<std::string, static_cast<size_t>(Selection::MaxSelection)> SelectionStr = {
					"nearest", "biggest", "smallest", "random"
				};
				const auto selstr = std::string(J["selection"]);
				auto it = std::find(SelectionStr.cbegin(), SelectionStr.cend(), selstr);
				if (it == SelectionStr.cend()) throw std::runtime_error("select_group: unknown selection");
				selection_ = static_cast<Selection>(std::distance(SelectionStr.cbegin(), it));
			}

			float assess_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{	// returning a probability for whether choose this in a substate context
				return 0.f;
			}

			void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
				select_target(self, sim);
			}

			void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
				select_target(self, sim);
			}

		private:
			void select_target(agent_type* self, const Simulation& sim)
			{
				const auto& groups = sim.groups<prey_tag>();
				auto it = groups.cend();

				switch (selection_) {
				case Selection::Nearest:
					it = std::min_element(groups.cbegin(), groups.cend(), [pos = self->pos](const auto& a, const auto& b) {
						return glm::distance2(a.gc(), pos) <
							glm::distance2(b.gc(), pos);
					});
					break;
				case Selection::Biggest:
					it = std::max_element(groups.cbegin(), groups.cend(), [](const auto& a, const auto& b) {
						return a.size < b.size;
						});
					break;
				case Selection::Smallest:
					it = std::min_element(groups.cbegin(), groups.cend(), [](const auto& a, const auto& b) {
						return a.size < b.size;
						});
					break;
				case Selection::Random:
					if (!groups.empty()) {
						auto dist = std::uniform_int_distribution<size_t>(0ull, groups.size() - 1);
						it = groups.cbegin() + dist(reng);
					}
					break;
				default:
					break;
				}
				self->target = -1;
				if (it != groups.cend()) {
					const auto group_id = static_cast<size_t>(std::distance(groups.cbegin(), it));
					self->target = sim.group_mates<prey_tag>(group_id)[0];
				}
			}

			Selection selection_ = Selection::Nearest;
		};

		template <typename Agent>
		class shadowing
		{ // shadow closest prey

			make_action_from_this(shadowing);

		public:
			shadowing() {}
			shadowing(size_t, const json& J)
			{
				bearing_ = glm::radians(float(J["bearing"]));
				dist_ = J["distance"];
				placement_ = 0 != int(J["placement"]);
				w_ = J["w"];
				prey_speed_scale_ = J["prey_speed_scale"];                       // [1]
			}

			float assess_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{	// returning a probability for whether choose this in a substate context
				return 0.f;
			}

			void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
				if (placement_) {
					const auto& target = sim.pop<prey_tag>()[self->target];
					self->pos = target.pos + dist_ * math::rotate(target.dir, bearing_, self->H.up());
					self->dir = target.dir;
				}
			}

			void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
				if (-1 != self->target) {
					const auto& target = sim.pop<prey_tag>()[self->target];
					const auto pos = target.pos + dist_ * math::rotate(target.dir, bearing_, self->H.up());
					const auto ofs = math::ofs(self->pos, pos);
					const auto Fdir = math::save_normalize(ofs, self->dir);
					self->steering += w_ * Fdir;
					self->speed = prey_speed_scale_ * target.speed;
				}
			}

		private:
			float bearing_;
			float dist_;
			float w_;           // [1]
			bool placement_;
			float prey_speed_scale_; // [1]
		};
	}
}


#endif
