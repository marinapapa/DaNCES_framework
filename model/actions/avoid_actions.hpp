#ifndef AVOID_ACTIONS_HPP_INCLUDED
#define AVOID_ACTIONS_HPP_INCLUDED

#include <glmutils/ray.hpp>
#include <glmutils/random.hpp>
#include <model/while_topo.hpp>
#include "action_base.hpp"


namespace model {
  namespace actions {
    // PREY - PREY INTERACTIONS 

    template <typename Agent>
    class avoid_n_position
    { // avoid neighbors position

      make_action_from_this(avoid_n_position);

    public:
      avoid_n_position() {}
      avoid_n_position(size_t, const json& J)
      {
        topo = J["topo"]; // [1]

        float fov = J["fov"]; // [deg]
        cfov = glm::cos(glm::radians(180.0f - 0.5f * (360.0f - fov))); // [1]

        float minsep = J["minsep"];       // [m]
        minsep2 = minsep * minsep;        // [m^2]

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

        auto ofss = vec3(0);
        auto realized_topo = while_topo(sv, topo, [&](const auto& ni) {

          if (in_fov(self, ni.dist2, flock[ni.idx].pos, this))
          {
            if (ni.dist2 < minsep2)
            {
				ofss += math::ofs(flock[ni.idx].pos, self->pos);
              return true;
            }
          }
          return false;
        });

		const vec3 Fdir = math::save_normalize(ofss, vec3(0.f)) * w_;
		self->steering += Fdir;
      }

       public:
        int topo = 0;           // [1]
        float cfov = 0;         // [1]
        float minsep2 = 0;      // [m^2]
        float maxdist2 = 0;     // [m^2]
    private:
        float w_;               // [1]
      };


    template <typename Agent>
    class avoid_n_direction
    {  // avoid neighbors direction (collision point)

      make_action_from_this(avoid_n_direction);

    public:
      avoid_n_direction() {}
      avoid_n_direction(size_t, const json& J)
      {
        topo = J["topo"]; // [1]
        float fov = J["fov"];
        cfov = glm::cos(glm::radians(180.0f - 0.5f * (360.0f - fov))); // [1]

        float minsep = J["minsep"];       // [m]
        minsep2 = minsep * minsep;        // [m^2]

        float maxdist = J["maxdist"];     // [m]
        maxdist2 = maxdist * maxdist;     // [m^2]
          
        float col_dist = J["coldist"];     // [m]
        col_dist2 = col_dist * col_dist;   // [m^2]

        w_ = J["w"];                       // [1]
      }

      void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
      {
      }

      void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
      {
        const auto sv = sim.sorted_view<Tag>(idx);
        const auto& flock = sim.pop<Tag>();
        float t1, t2; // collision point 
        const auto rerr = 0.1f * glmutils::unit_vec2(reng); // error for parallel turn to neighbor

        auto adir = vec3(0);

        auto realized_topo = while_topo(sv, topo, [&](const auto& ni) {

          if (in_fov(self, ni.dist2, flock[ni.idx].pos, this))
          {
            if (ni.dist2 < minsep2) //  if too close avoid position
            {
              adir += math::ofs(flock[ni.idx].pos, self->pos);
              return true;
            }
            else if (glmutils::intersectRayRay(self->pos, self->dir, flock[ni.idx].pos, flock[ni.idx].dir, t1, t2))
            { // if intersects, turn almost parallel with neighbor
              const auto colp = (self->pos + t1 * self->dir); // point of intersections
              if (glm::distance2(self->pos, colp) < col_dist2)
              {
                adir += (flock[ni.idx].dir) + rerr;
              }
              return true;
            }
            return true;
          }
          return false;
        });

		const vec3 Fdir = math::save_normalize(adir, vec3(0.f)) * w_;
		self->steering += Fdir;
      }

      public:
        int topo = 0;           // [1]
        float cfov = 0;         // [1]
        float minsep2 = 0;      // [m^2]
        float maxdist2 = 0;     // [m^2]
        float col_dist2 = 0;     // [m^2]

    private:
        float w_;               // [1]
    };
  }
}

#endif