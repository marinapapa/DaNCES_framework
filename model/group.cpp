#include <queue>
#include <algorithm>
#include <libs/graph.hpp>
#include <glmutils/oobb.hpp>
#include <model/math.hpp>
#include <agents/agents.hpp>
#include <model/group.hpp>
#include <model/simulation.hpp>

namespace model {

  void group_tracker::cluster(float dd)
  {
    group_id_.assign(proxy_.size(), no_group);
    const auto n = proxy_.size();
    auto cc = graph::connected_components(0, static_cast<int>(n), [&](int i, int j) {
      return dd > glm::distance2(proxy_[i].pos, proxy_[j].pos);
    });
    descr_.clear();
    for (unsigned ci = 0; ci < static_cast<unsigned>(cc.size()); ++ci) {
      vpos_.clear();
      vvel_.clear();
      vec3 vel = vec3(0);
      for (auto i : cc[ci]) {
        group_id_[proxy_[i].idx] = ci;
        vpos_.emplace_back(math::ofs(proxy_[cc[ci][0]].pos, proxy_[i].pos));
        vvel_.emplace_back(proxy_[i].vel);
        vel += proxy_[i].vel;
      }
      vec3 ext;
      auto H = glmutils::oobb(static_cast<int>(cc[ci].size()), vpos_.begin(), ext);
      vel /= cc[ci].size();
      H[2] += glm::vec4(proxy_[cc[ci][0]].pos, 0.f);
      descr_.push_back({ vpos_.size(), vel, H, ext });
    }
  }


  void group_tracker::track()
  {
    const auto dt = Simulation::dt();
    for (auto& fd : descr_) {
      vec3 gc = fd.gc();
      fd.H[2] = gc + dt * fd.vel;
    }
  }

}
