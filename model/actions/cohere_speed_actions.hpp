#ifndef COHERE_SPEED_ACTIONS_HPP_INCLUDED
#define COHERE_SPEED_ACTIONS_HPP_INCLUDED

#include "action_base.hpp"


namespace model {
  namespace actions {

    template <typename Agent>
    class cohere_accel_forwards
    {  // cohere by accelerating based on the forward position of closest neighbors

      make_action_from_this(cohere_accel_forwards);

    public:
      cohere_accel_forwards() {}
      cohere_accel_forwards(size_t, const json& J)
      {
        topo = J["topo"];    // [1]
        w_ = J["w"];    // [1]
        smooth_sigma_ = J["smooth_sigma"];    // [1]

        float fov = J["fov"]; // [deg]
        cfov = glm::cos(glm::radians(180.0f - 0.5f * (360.0f - fov))); // [1]

        float maxdist = J["maxdist"];     // [m]
        maxdist2 = maxdist * maxdist;     // [m^2]
      }

      void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
      {
      }

    private:

      const float smooth_gaussian_(const float& dist2) {
        return(1.f - std::exp(-dist2 / (2.f * std::pow(smooth_sigma_, 2.f))));
      }

    public:
      void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
      {
        const auto nv = sim.sorted_view<Tag>(idx);
        const auto& flock = sim.pop<Tag>();

        auto av_y_dev = 0.f; // average distance to neighbors
        auto realized_topo = while_topo(nv, topo, [&](const auto& ni) {
          if (in_fov(self, ni.dist2, flock[ni.idx].pos, this))
          {
            av_y_dev += math::ofs(self->pos.y, flock[ni.idx].pos.y);
            return true;
          }
          return false;
        });

        const auto w_scaled = (realized_topo) ?
          w_ * smooth_gaussian_(av_y_dev / realized_topo)
          : 0.f; //-w_ * decel_w_;
        self->steering += w_scaled * self->dir;
      }

    public:
      int topo = 0;         // [1]
      float maxdist2 = 0;   // [m^2]
      float cfov = 0;       // [1]
    private:
      float w_ = 0;         // [1] 
      float smooth_sigma_;
    };
  }
}

#endif