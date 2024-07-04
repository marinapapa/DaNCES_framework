#ifndef PREY_AGENTS_FWD_HPP_INCLUDED
#define PREY_AGENTS_FWD_HPP_INCLUDED

#include <type_traits>
#include <tuple>
#include <vector>
#include <limits>
#include "math.hpp"


namespace model {

  using tick_t = int64_t;
  constexpr tick_t max_tick = std::numeric_limits<tick_t>::max();

  using glm::vec3;

  // publish our species type(s) to the model core
  class Pred;
  class Prey;

  using prey_tag = std::integral_constant<size_t, 0>;
  using pred_tag = std::integral_constant<size_t, 1>;


  using species_pop = std::tuple<
    std::vector<Prey>,
    std::vector<Pred>
  >;


  class state_info_t {
  public:
    static constexpr size_t max_idx = 16383; // 2^14 - 1;

    state_info_t() : raw_{ 0u, 0u, 0u }, exit_tick_(std::numeric_limits<tick_t>::max()) {}
    state_info_t(bool copyable, size_t state, size_t sub_state, tick_t exit_tick) :
      exit_tick_(exit_tick),
      raw_{ static_cast<uint32_t>(state), static_cast<uint32_t>(sub_state), copyable }
    {}

    // conversion to main state index
    operator size_t () const noexcept { return raw_.state; }
    bool copyable() const noexcept { return raw_.copyable; }
    size_t state() const noexcept { return raw_.state; }
    size_t sub_state() const noexcept { return raw_.sub_state; }
    tick_t exit_tick() const noexcept { return exit_tick_; }

    void state(size_t val) noexcept { raw_.state = val; }
    void sub_state(size_t val) noexcept { raw_.sub_state = val; }
    void exit_tick(size_t val) noexcept { exit_tick_ = val; }

  private:
    tick_t exit_tick_;
    struct raw_ { uint32_t state : 15, sub_state : 15, copyable : 1; } raw_;
  };


  template <typename Tag>
  struct known_color_maps;
  
  template <>
  struct known_color_maps<prey_tag>;

  template <>
  struct known_color_maps<pred_tag>;


  template <typename Tag>
  struct agent_instance {};
  
  template <>
  struct agent_instance<prey_tag>;

  template <>
  struct agent_instance<pred_tag>;

  using species_instances = std::tuple<
    std::vector<agent_instance<prey_tag>>,
    std::vector<agent_instance<pred_tag>>
  >;


  // a birds head system
  class head_system {
  public:
    template <typename Agent>
    void initialize(const Agent& agent) {
      regenerateH(agent.pos, agent.dir);
      v0_ = glm::dvec3(agent.speed * agent.dir);
    }

    template <typename Agent>
    void update(const Agent& agent, float dt) {
      // Hello Newton
      const auto p0 = pos();
      const auto p1 = agent.pos;
      const auto v = (p1 - p0) / dt;
      const auto a = (v - v0_) / dt;
      const auto m = agent.ai.bodyMass;
      const auto F = m * (a + 9.81f * glm::vec3(0, -1, 0));
      auto Flat = glm::dot(side(), F);

      const auto b = this->B();
      const auto s = glm::length(v);
      const double cs = agent.sa.cruiseSpeed;
      const float L = (9.81f * m * (s * s) / (cs * cs));   // lift
      const auto Llat = glm::dot(side(), L * glm::vec3(b[1]));
      Flat = std::clamp(Flat, -L/1.1f, L/1.1f);   // hack, hack

      // ok, complete fake, needs smoothing, scaling, clamping
      if (Llat < Flat) beta_ -= dt * agent.ai.betaIn;
      else if (Llat > Flat) beta_ += dt * agent.ai.betaIn;
      regenerateH(agent.pos, agent.dir);
      v0_ = v;
    }

    // accessors, valid after 'update'
    glm::vec3 forward() const noexcept { return H_[0]; }
    glm::vec3 up() const noexcept { return H_[1]; }
    glm::vec3 side() const noexcept { return H_[2]; }
    glm::vec3 pos() const noexcept { return H_[3]; }
    glm::vec3 bside() const noexcept { return math::rotate(side(), float(beta_), forward()); }
    operator const glm::mat4& () const noexcept { return H_; }

    glm::mat4 B() const noexcept {
      const auto b = bside();
      glm::mat4 B;
      B[0] = H_[0];
      B[1] = glm::vec4(glm::cross(forward(), b), 0.f);
      B[2] = glm::vec4(b, 0.f);
      B[3] = H_[3];
      return B;
    }

    double beta() const noexcept { return beta_; }

    // positive 'rad': upwards
    glm::vec3 pitch(float rad) const { return math::rotate(forward(), rad, side()); }

    // positive 'rad': CCW
    glm::vec3 yaw(float rad) const { return math::rotate(forward(), rad, up()); }

    // transforms the position 'global' into this head_system
    glm::vec3 local_pos(const glm::vec3& global_pos) const {
      return local_vec(global_pos - pos());
    }

    // transforms the global free vector 'global_vec' into this head_system
    glm::vec3 local_vec(const glm::vec3& global_vec) const {
      return glm::vec3(
        glm::dot(forward(), global_vec),
        glm::dot(up(), global_vec),
        glm::dot(side(), global_vec));
    }

    // transforms the local position 'local' into world-system
    glm::vec3 global_pos(const glm::vec3& local_pos) const {
      return H_ * glm::vec4(local_pos, 1.f);
    }

    // transforms the local free vector 'local_vec' into world-system
    glm::vec3 global_vec(const glm::vec3& local_vec) const {
      return H_ * glm::vec4(local_vec, 0.f);
    }

  private:
    void regenerateH(const glm::vec3& pos, const glm::vec3& dir) {
      auto s = glm::normalize(glm::cross(glm::vec3(0, 1, 0), dir));
      auto u = glm::cross(dir, s);
      H_[0] = glm::vec4(dir, 0.f);
      H_[1] = glm::vec4(u, 0.f);
      H_[2] = glm::vec4(s, 0.f);
      H_[3] = glm::vec4(pos, 1.f);
    }

    glm::mat4 H_;
    glm::vec3 v0_;
    double beta_ = 0.0;   // banking angle, positive: CW
  };

}

#endif
