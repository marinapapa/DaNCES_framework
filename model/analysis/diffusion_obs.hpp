#ifndef DIFFUSION_OBS_HPP_INCLUDED
#define DIFFUSION_OBS_HPP_INCLUDED


#include <memory>
#include <future>
#include <deque>
#include <set>
#include <algorithm>
#include <tbb/tbb.h>
#include <hrtree/sorting/insertion_sort.hpp>
#include <model/observer.hpp>
#include <agents/agents.hpp>


namespace analysis {

  namespace diffusion {

    struct snapshot_data
    {
      model::vec3 pos;
      model::vec3 dir;
      std::vector<model::neighbor_info> ninfo;
    };

    using snapshot_t = std::vector<snapshot_data>;
    using window_t = std::deque<snapshot_t>;


    // fills 'qmtd' with Qm(t)
    inline void Qm(const window_t& win, size_t max_topo, float* qmtd)
    {
      const auto T = win.size();      // time steps
      const auto N = win[0].size();   // individuals
      const auto I = max_topo;        // neighbors to consider
      auto qmt = std::vector<size_t>(T, 0);
      auto M0 = std::vector<unsigned>(I);
      auto Mt = std::vector<unsigned>(I);
      for (auto n = 0; n < N; ++n) {
        std::transform(win[0][n].ninfo.cbegin(), win[0][n].ninfo.cbegin() + max_topo, M0.begin(), [](const auto& ni) { return ni.idx; });
        hrtree::insertion_sort(M0.begin(), M0.end());
        for (auto t = 1; t < T; ++t) {
          std::transform(win[t][n].ninfo.cbegin(), win[t][n].ninfo.cbegin() + max_topo, Mt.begin(), [](const auto& ni) { return ni.idx; });
          hrtree::insertion_sort(Mt.begin(), Mt.end());
          auto it = std::set_intersection(M0.cbegin(), M0.cend(), Mt.cbegin(), Mt.cend(), Mt.begin());
          qmt[t] += std::distance(Mt.begin(), it);
        }
      }
      std::transform(qmt.cbegin(), qmt.cend(), qmtd, [S = N * I](const auto& x) { return float(double(x) / S); });
      qmtd[0] = 1.0;
    }

    // fills 'dev' with r^2(t)
    inline void R(const window_t& win, float* dev)
    {
      const auto T = win.size();      // time steps
      const auto N = win[0].size();   // individuals
      auto cm = std::vector<model::vec3>(T, { 0,0,0 });
      for (auto t = 0; t < T; ++t) {
        // center of mass
        const auto pc = win[t][0].pos;    // pick one
        for (auto n = 0; n < N; ++n) {
          cm[t] += math::ofs(pc, win[t][n].pos);
        }
        cm[t] = cm[t] / float(N);
      }
      for (auto t = 0; t < T; ++t) {
        for (auto n = 0; n < N; ++n) {
          const auto r0 = math::ofs(win[0][n].pos, cm[0]);
          const auto rt = math::ofs(win[t][n].pos, cm[t]);
          dev[t] += glm::distance2(rt, r0);
        }
        dev[t] /= (N * T);
      }
    }
  }


  template <typename Tag>
  class DiffusionObserver : public model::AnalysisObserver 
  {
  public:
    DiffusionObserver(const std::filesystem::path& out_path, const json& J) :
      AnalysisObserver(out_path, J),
      max_topo_(J["max_topo"])
    {
      wsize_ = (exporter_.columns() - 1) / 2;
      std::regex qm_regex("Qm"), r_regex("R");
      if (wsize_ != std::distance(std::sregex_iterator(exporter_.header().cbegin(), exporter_.header().cend(), qm_regex), std::sregex_iterator{})) {
        throw std::runtime_error("DiffusionObserver: Qm: wrong number of repetitions");
      }
      if (wsize_ != std::distance(std::sregex_iterator(exporter_.header().cbegin(), exporter_.header().cend(), r_regex), std::sregex_iterator{})) {
        throw std::runtime_error("DiffusionObserver: R: wrong number of repetitions");
      }
    }

    ~DiffusionObserver()
    {
      if (future_.valid()) future_.get();
    }

  private:
    void notify_init(const model::Simulation& sim) override
    {
      max_topo_ = std::min(sim.pop<Tag>().size() - 1, max_topo_);
    }

    void notify_pre_collect(const model::Simulation& sim) override
    {
      sim.force_neighbor_info_update(true);
    }

    void notify_collect(const model::Simulation& sim) override
    {
      assert(sim.forced_neighbor_info_update());
      sim.force_neighbor_info_update(false);
      if (future_.valid()) future_.get();
      pull_data(sim);
      if (window_.size() == wsize_) {
        future_ = std::async(std::launch::async, &DiffusionObserver::analyse, this, sim.tick());
      }
    }

    void notify_save(const model::Simulation& sim) override
    {
      if (future_.valid()) future_.get();
      exporter_(data_.data(), data_.size());
    }

    void pull_data(const model::Simulation& sim)
    {
      const auto& pop = sim.pop<Tag>();
      if (window_.size() == wsize_) {
        // treat deque as ring-buffer
        auto oldest = std::move(window_.front());
        window_.pop_front();
        window_.emplace_back(std::move(oldest));  // ready for re-use
      }
      else {
        auto state = diffusion::snapshot_t(sim.pop<Tag>().size(), { {}, {}, std::vector<model::neighbor_info>(max_topo_) });
        window_.emplace_back(std::move(state));
      }
      auto& state = window_.back();
      for (size_t i = 0; i < pop.size(); ++i) {
        auto& pivot = state[i];
        pivot.pos = pop[i].pos;
        pivot.dir = pop[i].dir;
        auto sv = sim.sorted_view<Tag>(i);
        const auto n = std::min(sv.size(), max_topo_);
        pivot.ninfo.assign(sv.cbegin(), sv.cbegin() + n);
        std::fill(pivot.ninfo.begin() + n, pivot.ninfo.end(), model::neighbor_info{});
      }
    }

    void analyse(model::tick_t tick)
    {
      const auto row_size = (1 + 2 * wsize_);
      data_.resize(data_.size() + row_size, 0.f);
      float* p = data_.data() + data_.size() - row_size;
      p[0] = tick;
      auto fqm = std::async(std::launch::async, diffusion::Qm, window_, max_topo_, p + 1);
      diffusion::R(window_, p + 1 + wsize_);
      fqm.get();
    }

    diffusion::window_t window_;
    size_t max_topo_ = 0;
    double dt_;
    size_t wsize_;
    std::future<void> future_;
    std::vector<float> data_;    // raw data
  };

}


#endif
