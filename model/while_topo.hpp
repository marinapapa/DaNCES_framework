#ifndef MODEL_WHILE_TOPO_HPP_INCLUDED
#define MODEL_WHILE_TOPO_HPP_INCLUDED

#include <model/model.hpp>

namespace model {

  template <typename Fun>
  inline size_t while_topo(const neighbor_info_view& v, size_t topo, Fun&& fun)
  {
    auto n = topo;
    for (auto it = v.cbegin(); n && (it != v.cend()); ++it) {
      if (fun(*it)) --n;
    }
    return topo - n;
  }

  template <typename Fun>
  inline size_t until_topo(const neighbor_info_view& v, size_t topo, Fun&& fun)
  {
      auto n = topo;
      for (auto it = v.cbegin(); n && (it != v.cend()); ++it) {
          if (fun(*it)) break;
          --n;
      }
      return topo - n;
  }

  template <typename Agent, typename Action>
  inline bool in_fov(Agent* self, const float nidist2, const vec3 nipos, const Action& act)
  {
    if (nidist2 != 0.0f && nidist2 < act->maxdist2)
    {
      const auto offs = math::save_normalize(nipos - self->pos, glm::vec3(0));
      if (glm::dot(self->dir, offs) > act->cfov)
      {
        return true;
      }
    }
    return false;
  }

}

#endif
