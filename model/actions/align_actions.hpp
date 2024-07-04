#ifndef ALIGN_ACTIONS_HPP_INCLUDED
#define ALIGN_ACTIONS_HPP_INCLUDED

#include <model/while_topo.hpp>
#include "action_base.hpp"


namespace model {

  namespace actions {

    template <typename Agent>
    class align_n
    { // align by turning with all neighbors

      make_action_from_this(align_n);

    public:
      align_n() {}
      align_n(size_t, const json& J)
      {
        topo = J["topo"];     // [1]
        float fov = J["fov"];  // [deg]
        cfov = glm::cos(glm::radians(180.0f - 0.5f * (360.0f - fov))); // [1]

        // max distance
        float maxdist = J["maxdist"];    // [m]
        maxdist2 = maxdist * maxdist;    // [m^2]
        w_ = J["w"];                     // [1]
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
        
		    vec3 adir(0.f);
        auto realized_topo = while_topo(sv, topo, [&](const auto& ni) {
          if (in_fov(self, ni.dist2, flock[ni.idx].pos, this)) {
            adir += flock[ni.idx].dir;
            return true;
          }
          return false;
        });
		    vec3 Fdir = math::save_normalize(adir, vec3(0.f)) * w_; 
        self->steering += Fdir;
      }

    public:
      int topo = 0;       // [1]
      float cfov = 0;     // [1]
      float maxdist2 = 0; // [m^2]

    private:
      float w_;           // [1]
    }; 


    template <typename Agent>
    class align_direction
    { // align to 'remembered' direction prev_exit_dir_
      make_action_from_this(align_direction);

    public:
      align_direction() {}
      align_direction(size_t, const json& J)
      {
        w_ = J["w"];                     // [1]
      }

      float assess_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
      {	// returning a probability for whether choose this in a substate context
        return 0.f;
      }

      void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
      {
        dir_ = self->prev_exit_dir;
      }

      void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
      {
        self->steering += w_ * dir_;
      }

    private:
      float w_;           // [1]
      glm::vec3 dir_;
    };


  }
}

#endif
