#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>


class Camera
{
public:
  Camera() :
    V_(1.0), P_(1.0),
    viewport_(0.0, 0.0, 1.0, 1.0)
  {}

  glm::dvec3 eye() const {
    auto R = glm::transpose(glm::dmat3(V_));      // Camera rotation matrix
    return R * -glm::dvec3(V_[3]);
  }

  glm::dvec3 forward() const { return V_[0]; }
  glm::dvec3 up() const { return V_[1]; }
  glm::dvec3 side() const { return V_[2]; }

  glm::dmat4 R() const {    // Rotational part of camera matrix
    auto R = glm::dmat4(glm::dmat3(forward(), up(), side()));
    R[3][3] = 1.0;
    return R;
  }

  const glm::dmat4& V() const { return V_; };    // View matrix
  const glm::dmat4& P() const { return P_; };    // Perspective matrix
  const glm::dvec4& viewport() const { return viewport_; }

  void setOrthoViewport(const glm::dvec4& viewport, const glm::dvec4& world) {
    viewport_ = viewport;
    P_ = glm::ortho(world.x, world.y, world.z, world.w);
  }

  void setPerspectiveViewport(const glm::dvec4& viewport, double fovy, double clipNear, double clipFar) {
    viewport_ = viewport;
    P_ = glm::perspective(fovy, viewport_[2] / viewport_[3], clipNear, clipFar);
  }

  void setViewMatrix(const glm::dmat4& V) { V_ = V; };

  void setR(const glm::dmat4& R) {
    V_[0][0] = R[0][0];
    V_[0][1] = R[1][0];
    V_[0][2] = R[2][0];
    V_[1][0] = R[0][1];
    V_[1][1] = R[1][1];
    V_[1][2] = R[2][1];
    V_[2][0] = R[0][2];
    V_[2][1] = R[1][2];
    V_[2][2] = R[2][2];
  }

private:
  glm::dmat4 V_;          // View
  glm::dmat4 P_;          // Projection
  glm::dvec4 viewport_;   // full window viewport
};


