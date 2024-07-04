#ifndef MIXED_ACTIONS_HPP_INCLUDED
#define MIXED_ACTIONS_HPP_INCLUDED

#include <glmutils/ray.hpp>
#include <glmutils/random.hpp>
#include <model/while_topo.hpp>
#include "action_base.hpp"


namespace model {
  namespace actions {
    // PREY - PREY INTERACTIONS 

    template <typename Agent>
    class avoid_pos_or_cohere_all
    {  // avoid neighbors position, if no one to avoid, cohere with all topo

      make_action_from_this(avoid_pos_or_cohere_all);

    public:
      avoid_pos_or_cohere_all() {}
      avoid_pos_or_cohere_all(size_t, const json& J)
      {
        topo_coh = J["topo_coh"]; // [1]
        topo_sep = J["topo_sep"]; // [1]
        float fov = J["fov"];
        ffov = J["ffov"];    // [deg]
        cfov = glm::cos(glm::radians(180.0f - 0.5f * (360.0f - fov))); // [1]

        float minsep = J["minsep"];       // [m]
        minsep2 = minsep * minsep;        // [m^2]

        float maxdist = J["maxdist"];     // [m]
        maxdist2 = maxdist * maxdist;     // [m^2]

        w_ = J["w"];                       // [1]
      }

      void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
      {
      }

      void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
      {
        // from avoid_n_direction
        const auto sv = sim.sorted_view<Tag>(idx);
        const auto& flock = sim.pop<Tag>();

        auto adir = vec3(0);
        auto realized_topo_sep = while_topo(sv, topo_sep, [&](const auto& ni) {

          if (in_fov(self, ni.dist2, flock[ni.idx].pos, this))
          {
            if (ni.dist2 < minsep2) //  if too close avoid position
            {
              adir += math::ofs(flock[ni.idx].pos, self->pos);
              return true;
            }
          }
          return false;
        });

        if (realized_topo_sep)
        {
			    const vec3 Fdir = math::save_normalize(adir, vec3(0.f)) * w_;
			    self->steering += Fdir;
	    	}
        else
        { // from cohere_turn_n_all
            auto realized_topo = while_topo(sv, topo_coh, [&](const auto& ni) {

              if (in_fov(self, ni.dist2, flock[ni.idx].pos, this))
              {
                adir += math::ofs(self->pos, flock[ni.idx].pos);
                return true;
              }
              return false;
            });
			  const vec3 Fdir = math::save_normalize(adir, vec3(0.f)) * w_;
			  self->steering += Fdir;
        }
      }

    public:
      int topo_coh = 0;           // [1]
      int topo_sep = 0;           // [1]
      float cfov = 0;          // [1]
      float ffov = 0;          // [deg] front field of view
      float minsep2 = 0;       // [m^2]
      float maxdist2 = 0;      // [m^2]
    
    private:
      float w_;               // [1]
    };


    template <typename Agent>
    class avoid_dir_or_cohere_all
    {  // avoid neighbors direction (collision point), if no one to avoid, cohere with all topo

      make_action_from_this(avoid_dir_or_cohere_all);

    public:
      avoid_dir_or_cohere_all() {}
      avoid_dir_or_cohere_all(size_t, const json& J)
      {
        topo_coh = J["topo_coh"]; // [1]
        topo_sep = J["topo_sep"]; // [1]
        float fov = J["fov"];
        cfov = glm::cos(glm::radians(180.0f - 0.5f * (360.0f - fov))); // [1]

        float minsep = J["minsep"];       // [m]
        minsep2 = minsep * minsep;        // [m^2]

        float maxdist = J["maxdist"];     // [m]
        maxdist2 = maxdist * maxdist;     // [m^2]

        float col_dist = J["coldist"];     // [m]
        col_dist2 = col_dist * col_dist;     // [m^2]

        w_ = J["w"];                       // [1]
      }

      void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
      {
      }

	  void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
	  {
		  // from avoid_n_direction
		  const auto sv = sim.sorted_view<Tag>(idx);
		  const auto& flock = sim.pop<Tag>();
		  float t1, t2; // collision point 
		  const auto rerr = 0.1f * glmutils::unit_vec2(reng); // error for parallel turn to neighbor

		  auto adir = vec3(0);
		  auto realized_topo_sep = while_topo(sv, topo_sep, [&](const auto& ni) {

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
				 // return true;
			  }
			  return false;
			  });

		  if (realized_topo_sep)
		  {
			  const vec3 Fdir = math::save_normalize(adir, vec3(0.f)) * w_;
			  self->steering += Fdir;
		  }
		  else
		  { // from cohere_centroid_
			  auto realized_topo = while_topo(sv, topo_coh, [&](const auto& ni) {
				  if (in_fov(self, ni.dist2, flock[ni.idx].pos, this))
				  {
					  adir += math::ofs(self->pos, flock[ni.idx].pos);
					  return true;
				  }
				  return false;
				  });
			  const vec3 Fdir = math::save_normalize(adir, vec3(0.f)) * w_;
			  self->steering += Fdir;
		  }
	  }

    public:
      int topo_coh = 0;           // [1]
      int topo_sep = 0;           // [1]
      float cfov = 0;          // [1]
      float minsep2 = 0;       // [m^2]
      float maxdist2 = 0;      // [m^2]
      float col_dist2 = 0;     // [m^2]
   
    private:
      float w_;               // [1]
    };
  }
}


#endif