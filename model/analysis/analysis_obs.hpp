#ifndef ANALYSIS_OBS_HPP_INCLUDED
#define ANALYSIS_OBS_HPP_INCLUDED

#include <analysis/analysis.hpp>
#include <model/observer.hpp>
#include <agents/agents.hpp>
#include <algorithm> 


namespace analysis
{
	template <typename Tag>
	class TimeSeriesObserver : public model::AnalysisObserver
	{
	public:
		TimeSeriesObserver(const std::filesystem::path& out_path, const json& J) :
			AnalysisObserver(out_path, J)
		{}

	protected:
		void notify_collect(const model::Simulation& sim) override
		{
			const auto tt = static_cast<float>(sim.tick()) * model::Simulation::dt();

			sim.visit_all<Tag>([&](auto& p, size_t idx) {
				const auto fl_id = sim.group_of<Tag>(idx);
				const auto& thisgroup = sim.groups<Tag>()[fl_id];
				const auto dist2cent = glm::distance(p.pos, thisgroup.gc()); // distance to center of group
				const auto dir2fcent = glm::normalize(math::ofs(p.pos, thisgroup.gc()));
				auto si = p.get_current_state();

				const auto& all_nb = sim.sorted_view<Tag>(idx); // all neighbors
				float nnd2 = 0; 
				if (all_nb.size()) {
					nnd2 = all_nb.cbegin()->dist2;
			  }


				// copy into flat data store
				const auto last = data_out_.size();
				data_out_.resize(data_out_.size() + AnalysisObserver::columns());
				auto* pf = data_out_.data() + last;
				*pf = tt;
				*(++pf) = static_cast<float>(idx),
				*(++pf) = p.pos.x; *(++pf) = p.pos.y; *(++pf) = p.pos.z;
				*(++pf) = p.dir.x; *(++pf) = p.dir.z; *(++pf) = p.dir.y;
				*(++pf) = p.speed;
				*(++pf) = p.accel.x; *(++pf) = p.accel.y; *(++pf) = p.accel.z;
				*(++pf) = static_cast<float>(si.state()); *(++pf) = static_cast<float>(si.sub_state());
				*(++pf) = dist2cent;
				*(++pf) = dir2fcent.x; *(++pf) = dir2fcent.y; *(++pf) = dir2fcent.z;
				*(++pf) = nnd2;
			});
		}

		void notify_save(const model::Simulation& sim) override {
			exporter_(data_out_.data(), data_out_.size());
			data_out_.clear();
		}
	};


	template <typename Tag>
	class GroupObserver : public model::AnalysisObserver
	{
	public:
		GroupObserver(const std::filesystem::path & out_path, const json& J)
			: AnalysisObserver(out_path, J)
		{}

	protected:
		void notify_collect(const model::Simulation& sim) override
		{
			const auto tt = static_cast<float>(sim.tick()) * model::Simulation::dt();
			const auto& fi = sim.groups<Tag>();

			const auto last = data_out_.size();
			data_out_.resize(data_out_.size() + AnalysisObserver::columns() * fi.size());
			auto* pf = data_out_.data() + last;
			
			size_t idx = 0;
			for (auto& i : fi)
			{
				// polarization
				auto pol = 0.f;
				auto fdir = math::save_normalize(i.vel, vec3(0.f));
				auto fm = sim.group_mates<Tag>(idx);
				const auto& pop = sim.pop<Tag>();
				for (const auto& idx : fm) {
					pol += glm::dot(pop[idx].dir, fdir);
			  };
				pol /= fm.size();

				*pf = tt;
				*(++pf) = static_cast<float>(idx);
				*(++pf) = static_cast<float>(i.size);
				*(++pf) = i.vel.x; *(++pf) = i.vel.y; *(++pf) = i.vel.z;
				*(++pf) = pol;
				*(++pf) = i.ext.x * i.ext.y * i.ext.z;		// volume
				*(++pf) = i.ext.x; *(++pf) = i.ext.y; *(++pf) = i.ext.z;
				*(++pf) = i.H[0].x; *(++pf) = i.H[0].y; *(++pf) = i.H[0].z;
				*(++pf) = i.H[1].x; *(++pf) = i.H[1].y; *(++pf) = i.H[1].z;
				*(++pf) = i.H[2].x; *(++pf) = i.H[2].y; *(++pf) = i.H[2].z;
				++idx;
			}
		}

		void notify_save(const model::Simulation& sim) override
		{ 
			exporter_(data_out_.data(), data_out_.size());
			data_out_.clear();
		}
	};

}

#endif
