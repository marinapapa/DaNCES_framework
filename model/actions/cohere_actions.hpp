#ifndef COHERE_TURN_ACTIONS_HPP_INCLUDED
#define COHERE_TURN_ACTIONS_HPP_INCLUDED

#include "action_base.hpp"

namespace model {
  namespace actions {

    template <typename Agent>
    class cohere_centroid
    { // cohere by turning towards all neighbors

      make_action_from_this(cohere_centroid);

    public:
      cohere_centroid() {}
      cohere_centroid(size_t, const json& J)
      {
        topo = J["topo"];    // [1]

        float fov = J["fov"]; // [deg]
        cfov = glm::cos(glm::radians(180.0f - 0.5f * (360.0f - fov))); // [1]

        float maxdist = J["maxdist"];     // [m]
        maxdist2 = maxdist * maxdist;     // [m^2]

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
        const auto sv = sim.sorted_view<Tag>(idx);
        const auto& flock = sim.pop<Tag>();

        auto ofss = vec3(0.f);
        auto realized_topo = while_topo(sv, topo, [&](const auto& ni) {
          if (in_fov(self, ni.dist2, flock[ni.idx].pos, this)) {
		        ofss += math::ofs(self->pos, flock[ni.idx].pos);
            return true;
          }
          return false;
        });
		    const auto Fdir = math::save_normalize(ofss, vec3(0.f)) * w_;
		    self->steering += Fdir;
      }

    public:
      int topo = 0;           // [1]
      float cfov = 0;         // [1]
      float maxdist2 = 0;     // [m^2]
    
    private:
      float w_ = 0;           // [1]
    };
    
    template <typename Agent>
    class cohere_centroid_distance
    { // cohere by turning with all neighbors depending on distance to the centroid

      make_action_from_this(cohere_centroid_distance);

    public:
      cohere_centroid_distance() {}
      cohere_centroid_distance(size_t, const json& J)
      {
        topo = J["topo"];    // [1]

        float fov = J["fov"]; // [deg]
        cfov = glm::cos(glm::radians(180.0f - 0.5f * (360.0f - fov))); // [1]

        float maxdist = J["maxdist"];     // [m]
        maxdist2 = maxdist * maxdist;     // [m^2]

        max_w_dist_ = J["max_w_dist"];     // [m]
        min_w_dist_ = J["min_w_dist"];     // [m]

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
        const auto sv = sim.sorted_view<Tag>(idx);
        const auto& flock = sim.pop<Tag>();

        auto ofss = vec3(0.f);
        auto n = 0.f; // number of neighbors
        auto realized_topo = while_topo(sv, topo, [&](const auto& ni) {

          if (in_fov(self, ni.dist2, flock[ni.idx].pos, this))
          {
            ofss += math::ofs(self->pos, flock[ni.idx].pos);
            ++n;
            return true;
          }
          return false;
        });

        const auto w_scaled = (realized_topo) ? 
            w_ * math::smootherstep(glm::length(ofss / n), min_w_dist_, max_w_dist_) 
            : 0.f; 
        auto Fdir =  math::save_normalize(ofss, vec3(0.f)) * w_scaled;

        //Fdir.y = 0.f;
        self->steering += Fdir;
      }

    public:
      int topo = 0;           // [1]
      float cfov = 0;         // [1]
      float maxdist2 = 0;     // [m^2]

    private:
      float w_ = 0;           // [1]
      float min_w_dist_ = 0;   // minimum and max distance to cohere
      float max_w_dist_ = 0;
     // float max_w_dist_;      // [m]
     // float min_w_dist_;      // [m]

    };
  }
}

#endif