#ifndef MATH_UTILS_HPP_INCLUDED
#define MATH_UTILS_HPP_INCLUDED

#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glmutils/perp_dot.hpp>
#include <glmutils/plane.hpp>
#include <glmutils/ray.hpp>
#include <libs/rndutils.hpp>


#define forward_coor x
#define up_coor y
#define side_coor z


namespace math {


  template <typename T>
  inline T constexpr pi = T(3.1415926535897932384626433832795);

  template <typename T>
  constexpr T normalize_min_max(const T& a, const T& min, const T& max) noexcept
  {
    return (a - min) / (max - min);
  }

  template <typename T>
  constexpr std::vector<T> normalize_vector_min_max(const std::vector<T>& a) 
  {
    auto minp = std::min(a);
    auto maxp = std::max(a);
    auto retv = a;

    std::for_each(retv.begin(), retv.end(), [=](T& el) { math::normalize_min_max(el, minp, maxp); });
    std::for_each(retv.begin(), retv.end(), [](T& el) { assert(el <= 1); });

    return retv;
  }

  //template <typename T, typename X, size_t N>
  //constexpr void normalize_array(T (&a)[N])
  //{
  //  auto minp = std::min(a);
  //  auto maxp = std::max(a);

  //  std::for_each(a.size(), a.capacity(), [](X& el) { normalize(el, minp, maxp); });
  //  std::for_each(a.size(), a.capacity(), [](X& el) { assert(el <= 1); });
  //}

  template <typename T>
  constexpr T min_difference(const T& a, const T& b) noexcept
  {
    return std::min(a - b, b - a);
  }

  // Geometry
  
  // returns angle [rad] between a and b clamped to [-rad(maxDeg), +rad(maxDeg)].
  inline float rad_between_xz(const glm::vec3& a, const glm::vec3& b, float maxRad = pi<float>) noexcept
  {
    auto c = glmutils::perpDot(a, b);
    auto d = glm::dot(a, b);
    return glm::clamp(std::atan2(c, d), -maxRad, +maxRad);
  }


  // returns the vector 'a' rotated by 'rad' around 'axis'
  inline decltype(auto) rotate(const glm::vec3& a, float rad, const glm::vec3& axis) noexcept
  {
    return glm::vec3(glm::rotate(glm::mat4(1), rad, axis) * glm::vec4(a, 0.f));
  }


  inline decltype(auto) save_normalize(const glm::vec3& a, const glm::vec3& fallBack) noexcept
  {
    auto len2 = glm::dot(a, a);   // length2
    return (len2 > float(0.0000001)) ? a / std::sqrt(len2) : fallBack;  // covers NaN -> fallBack
  }

  inline decltype(auto) save_normalize(const glm::vec2& a, const glm::vec2& fallBack) noexcept
  {
      auto len2 = glm::dot(a, a);   // length2
      return (len2 > float(0.0000001)) ? a / std::sqrt(len2) : fallBack;  // covers NaN -> fallBack
  }


  // a + ofs == b
  template <typename T>
  constexpr T ofs(const T& a, const T& b) noexcept
  {
      return b - a;
  }

  // Interpolation functions

  template <typename T>
  constexpr T fade(T x) noexcept
  {
    return x * x * x * (x * (x * T(6) - T(15)) + T(10));
  }

  template <typename T>
  constexpr T smootherstep(T x, const T edge0, const T edge1) noexcept
  {
    // Scale, bias and saturate x to 0..1 range
    x = std::clamp((x - edge0) / (edge1 - edge0), T(0), T(1));
    return math::fade(x);
  }

  template <typename T>
  constexpr T smootherstep_bipolar(T x, const T edge0, const T edge1) noexcept
  {
    return T(-1) + T(2) * smootherstep<T>(x, edge0, edge1);
  }

  template <typename T>
  constexpr T smootherstep_bilateral(T x, const T edge0, const T edge1) noexcept
  {
    // Scale, bias and saturate x to 0..1 range
    x = std::clamp((x - edge0) / (edge1 - edge0), T(0), T(1));
    return math::fade(x);
  }

  template <typename T>
  constexpr T smoothstep(T x, const T edge0, const T edge1) noexcept
  {
    // Scale, bias and saturate x to 0..1 range
    x = std::clamp((x - edge0) / (edge1 - edge0), T(0), T(1));

    return x * x * (T(3) - T(2) * x);
  }


  // returns the time of nearest approach
  //
  // The interpretation is as follows: The two points
  // pA = origA + t * velA
  // pB = origB + t * velB
  // are building the line segment {pA, pB}
  // that is perpendicular to velA and perpendicular to velB, hence
  // we have found the two points of the closest approach
  // of the two trajectories. A real intersection
  // could be detected by checking the distance between these points
  // against zero.
  //
  // Notes: 
  // returns +/-Inf if velA == velB (t is undefined in this case). 
  // returns a negative value if time of nearest approach is in the past.
  //
  template<typename T>
  inline T nearestApproach(const glm::tvec3<T>& origA,
                           const glm::tvec3<T>& velA,
                           const glm::tvec3<T>& origB,
                           const glm::tvec3<T>& velB) 
  {
    const glm::tvec3<T> C(glm::cross(velA, velB));
    const T denom = glm::length2(C);
    if (denom < T(0.0000001)) [[unlikely]] {
      // nearly parallel - we can/must solve in 1D
      const auto a = glm::abs(velA);
      const auto i = (a[0] > a[1]) ? ((a[0] > a[2]) ? 0 : 2) : ((a[1] > a[2]) ? 1 : 2);
      return (origB[i] - origA[i]) / (velA[i] - velB[i]);
    }
    const glm::tvec3<T> D(origB - origA);
    const glm::tvec3<T> S(cross(D, velB));
    return glm::dot(S, C) / denom;
  }


  class collision_t 
  {
  public:
    template <typename AgentA, typename AgentB>
    collision_t(const AgentA& a, const AgentB& b) {
      const auto velA = a.speed * a.dir;
      const auto velB = b.speed * b.dir;
      t_ = nearestApproach(a.pos, velA, b.pos, velB);
      pA_ = a.pos + t_ * velA;
      pB_ = b.pos + t_ * velB;
      dist2_ = glm::distance2(pA_, pB_);
      float s, t;
      glmutils::intersectRayRay(a.pos, a.dir, b.pos, b.dir, s, t);
      int dummy = 0;
    }

    float t() const noexcept { return t_; }
    float dist2() const noexcept { return dist2_; }
    glm::vec3 pA() const noexcept { return pA_; }
    glm::vec3 pB() const noexcept { return pB_; }

    // return (geometrical) optimal avoidance vector for agent a
    glm::vec3 opt_avoidance_A() const noexcept { return pA_ - pB_; }

    // return (geometrical) optimal avoidance vector for agent b
    glm::vec3 opt_avoidance_B() const noexcept { return pB_ - pA_; }

  private:
    float t_;        // time of nearest approach
    float dist2_;    // distance^2 of nearest approach
    glm::vec3 pA_;
    glm::vec3 pB_;
  };

}

#endif
