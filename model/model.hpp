#ifndef MODEL_MODEL_HPP_INCLUDED
#define MODEL_MODEL_HPP_INCLUDED

#include <tuple>
#include <array>
#include <memory>
#include <utility>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <libs/rndutils.hpp>
#include <agents/agents_fwd.hpp>


namespace model {

  extern thread_local rndutils::default_engine reng;
  static constexpr size_t n_species = std::tuple_size_v<species_pop>;
  
  struct neighbor_info
  {
    float dist2;      // distance square
    unsigned idx;     // index of neighbor
    float stress;   

    // For copying escape mechanism:
    state_info_t state_info;  // neighbor's state, used for copying
  };


  class neighbor_info_view
  {
  public:
    neighbor_info_view() : first_(nullptr), n_(0) {}
    neighbor_info_view(const neighbor_info* first, size_t n) : first_(first), n_(n) {}

    const neighbor_info* begin() const noexcept { return first_; }
    const neighbor_info* end() const noexcept { return first_ + n_; }
    const neighbor_info* cbegin() const noexcept { return first_; }
    const neighbor_info* cend() const noexcept { return first_ + n_; }
    const neighbor_info& operator[](size_t i) const { 
      assert(i < n_);
      return *(first_ + i); 
    }
    bool empty() const noexcept { return n_ == 0; }
    size_t size() const noexcept { return n_; }

  private:
    const neighbor_info* first_;
    const size_t n_;
  };

  
  // OpenGL stuff
  struct instance_proxy {
    glm::mat4 B;
    float speed;
    float tex;
    float alpha;
    state_info_t state;      
  };

}

#endif
