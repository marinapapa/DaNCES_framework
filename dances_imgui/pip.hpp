#pragma once

// picture in picture
// Hanno 2022

#include <array>
#include <glm/glm.hpp>
#include <model/model.hpp>
#include "json.hpp"
#include "camera.hpp"
#include "Renderer.h"


enum CameraMode {
  Map,
  Television,
  Low,
  Top,
  Local,
  MaxMode
};


constexpr std::array<const char*, CameraMode::MaxMode> CamModeNames = {
  "Map", "Television", "Low", "Top", "Local"
};


// picture in picture
struct pip_t {
  pip_t(const json& J, size_t idx);

  bool enabled() const { return enabled_; }
  void enable(bool val) { enabled_ = val; }

  const std::array<bool,2>& trail() const { return trail_; }
  void trail(const std::array<bool, 2>& val) { trail_ = val; }
  const Camera& cam() const { return cam_; }

  bool hit_test(int screenX, int screenY) const;

  // [-1,1][-1,1]
  glm::dvec2 viewport_coor(int screenX, int screenY) const;
  glm::dvec3 screenDirection(const glm::dvec2& viewport_coor) const;
  double fovy() const { return cam_fov; }
  double distance() const { return cam_dist_; };

  void onMouseWheel(class AppWin* appwin, float delta);
  void onMouseRClick(class AppWin* appwin, int screenX, int screenY);
  void onMouseDrag(class AppWin* appwin, int deltaX, int deltaY);
  bool onKey(class AppWin* appwin);

  void mode(CameraMode mode);
  CameraMode mode() const { return cam_mode; }
  void update(const glm::ivec4& win_area);

  void follow(AppWin* appwin, size_t tag, size_t idx);
  void follow(const nearest_instance_record& nir) noexcept;
  const nearest_instance_record& follow() const noexcept ;

  float dimm() const noexcept { return dimm_; }
  void  dimm(float val) noexcept { dimm_ = val; }

private:
  glm::dmat4 lookAt(const glm::dvec3& eye, const glm::dvec3& center, const glm::dvec3& up);

  glm::dvec4 vp_prop;       // [proportional]
  glm::ivec4 hit_rect;      // [screen coordinates]
  Camera cam_;
  CameraMode cam_mode = CameraMode::Local;
  double cam_fov = 60.0;
  double cam_distance = 10.0;
  double cam_dist_ = 10.0;
  double cam_yaw_ = 0.0;
  double cam_pitch_ = 0.0;
  nearest_instance_record cam_follow;
  bool enabled_ = true;
  std::array<bool, 2> trail_ = { false, false };    // per species
  glm::dvec3 eye_ = { 0,0,0 };
  float dimm_ = 1.f;
  friend class AppWin;
};
