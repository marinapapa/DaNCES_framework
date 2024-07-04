//
// Utilities for OpenGL Mathematics (glm)
//
// Hanno Hildenbrandt 2008
//

//! \file perp_dot.hpp Unary and binary perp dot operator
//! \ingroup misc


#ifndef glmutils_perp_dot_hpp
#define glmutils_perp_dot_hpp


#include <glm/glm.hpp>


namespace glmutils {

  using namespace glm;
  using namespace glm::detail;


  //! Unary perp dot \f$ \mathbf{a}_\perp = (-a_y, a_x) \f$
  //!
  //! Returns a vector perpendicular (CCW) to \c a
  //!
  template<typename T>
  inline tvec2<T> perpDot(const tvec2<T>& a) noexcept
  {
    return tvec2<T>(-a.y, a.x);
  }


  //! perpDot xz-plane
  template<typename T>
  inline tvec3<T> perpDot(const tvec3<T>& a) noexcept
  {
    return tvec3<T>(-a.y, T(0), a.z);
  }


  //! Binary perp dot product \f$ \mathbf{a} \perp \mathbf{b} = -a_y \, b_x + a_x \, b_y \f$
  //!
  //! The binary perp dot operator calculates the dot product of
  //! \f$\mathbf{a}_\perp\f$ and \f$\mathbf{b}\f$, thus it returns
  //! \f[ |a||b| \cdot \cos(\angle \mathbf{a_\perp}, \mathbf{b}) = |a||b| \cdot \sin(\angle \mathbf{a}, \mathbf{b})\f]
  //! See \ref perpDot(const tvec2<T>& a) "unary perp dot"
  //!
  template<typename T>
  inline T perpDot(const tvec2<T>& a, const tvec2<T>& b) noexcept
  {
    return a.x * b.y - a.y * b.x;
  }


  //! perpDot xz-plane
  template<typename T>
  inline T perpDot(const tvec3<T>& a, const tvec3<T>& b) noexcept
  {
    return a.x * b.z - a.z * b.x;
  }



}   // namespace glmutils


#endif  // glmutils_perp_dot_hpp
