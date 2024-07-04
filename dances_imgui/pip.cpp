#include <glm/gtx/euler_angles.hpp>
#include "imgui_unit.h"
#include "pip.hpp"
#include "AppWin.h"
#include "Renderer.h"


// rotation matrix based on yaw and pitch around global axis
glm::dmat3 yaw_pitch(double yaw, double pitch) {
  const double ch = glm::cos(yaw);
  const double sh = glm::sin(yaw);
  const double cp = 1.0;
  const double sp = 0.0;
  const double cb = glm::cos(pitch);
  const double sb = glm::sin(pitch);

  glm::dmat3 R{};
  R[0][0] = ch * cb + sh * sp * sb;
  R[0][1] = sb * cp;
  R[0][2] = -sh * cb + ch * sp * sb;
  R[1][0] = -ch * sb + sh * sp * cb;
  R[1][1] = cb * cp;
  R[1][2] = sb * sh + ch * sp * cb;
  R[2][0] = sh * cp;
  R[2][1] = -sp;
  R[2][2] = ch * cp;
  return R;
}


pip_t::pip_t(const json& J, size_t idx) {
  auto& jpip = J["gui"]["pip"][idx];
  vp_prop[0] = jpip["vp_prop"][0];
  vp_prop[1] = jpip["vp_prop"][1];
  vp_prop[2] = jpip["vp_prop"][2];
  vp_prop[3] = jpip["vp_prop"][3];
  {
    const std::string cam_mode_name = jpip["cam_mode"];
    int cmi = 0;
    for (auto* cmn : CamModeNames) {
      if (cam_mode_name == cmn) break;
      ++cmi;
    }
    cam_mode = static_cast<CameraMode>(std::clamp(cmi, 0, static_cast<int>(CameraMode::MaxMode)));
  }
  cam_fov = jpip["fovy"];
  cam_follow.species = jpip["target_species"];
  cam_follow.idx = jpip["target_id"];
  enabled_ = jpip["enabled"];
  trail_ = jpip["ribbon"];
  if ((cam_mode == CameraMode::Television) || (cam_mode == CameraMode::Map)) {
    eye_ = jpip.at("eye").get<glm::vec3>();
  }
  dimm_ = J["gui"]["skybox"]["dimm"];
}


void pip_t::follow(AppWin* appwin, size_t tag, size_t idx)
{
  auto nir = appwin->renderer_->instance(tag, idx);
  if (nullptr == nir.pInstance) {
    // second chance 
    nir = appwin->renderer_->instance(tag, 0);
  }
  if (nir.pInstance) {
    cam_follow = nir;
  }
}


void pip_t::follow(const nearest_instance_record& nir) noexcept {
  if (nir.pInstance) {
    cam_follow = nir;
  }
}


const nearest_instance_record& pip_t::follow() const noexcept {
  return cam_follow;
}


bool pip_t::hit_test(int screenX, int screenY) const {
  return enabled_ &&
    (nullptr != cam_follow.pInstance) &&
    (screenX >= hit_rect[0]) &&
    (screenY >= hit_rect[1]) &&
    (screenX <= hit_rect[2]) &&
    (screenY <= hit_rect[3]);
}


inline glm::dvec3 world_coor(const glm::dmat4& IMVP, const glm::dvec2& viewport_coor, double z) {
  const auto vp_coor = glm::dvec4(viewport_coor, z, 1.0);
  const auto tc = IMVP * vp_coor;
  return glm::dvec3(tc) / tc.w;
}


glm::dvec2 pip_t::viewport_coor(int screenX, int screenY) const {
  const double w = hit_rect[2] - hit_rect[0];
  const double h = hit_rect[3] - hit_rect[1];
  return glm::dvec2(
    2.0 * (screenX - hit_rect[0]) / w - 1.0,
    1.0 - 2.0 * (screenY - hit_rect[1]) / h
  );
}


glm::dvec3 pip_t::screenDirection(const glm::dvec2& viewport_coor) const {
  const auto MVP = glm::dmat4(cam_.P() * cam_.V());
  const auto IMVP = glm::inverse(MVP);
  const auto f = world_coor(IMVP, viewport_coor, 1.0);
  const auto n = world_coor(IMVP, viewport_coor, 0.0);
  return glm::normalize(f - n);
}


void pip_t::onMouseRClick(AppWin* appwin, int screenX, int screenY) {
  const auto tag = glfwGetKey(appwin->window(), GLFW_KEY_LEFT_CONTROL) ? 1 : 0;
  auto nir = appwin->renderer_->nearest_instance(tag, *this, screenX, screenY);
  if (nir.pInstance) {
    cam_follow = nir;
  }
}


void pip_t::onMouseDrag(class AppWin* appwin, int deltaX, int deltaY) {
  switch (cam_mode) {
  case CameraMode::Local: {
    cam_yaw_ += -deltaX * 0.005;
    cam_pitch_ += -deltaY * 0.005;
    cam_pitch_ = std::clamp(cam_pitch_, glm::radians(-80.0), glm::radians(+80.0));
    break;
  }
  }
}


bool pip_t::onKey(AppWin* appwin) {
  if (cam_mode == CameraMode::Television) {
    // move in plane
    if (ImGui::IsKeyDown(ImGuiKey_W)) {
      eye_.x -= 1.0f;
      return true;
    }
    if (ImGui::IsKeyDown(ImGuiKey_A)) {
      eye_.z += 1.0f;
      return true;
    }
    if (ImGui::IsKeyDown(ImGuiKey_S)) {
      eye_.x += 1.0f;
      return true;
    }
    if (ImGui::IsKeyDown(ImGuiKey_D)) {
      eye_.z -= 1.0f;
      return true;
    }
  }
  if (ImGui::IsKeyPressed(ImGuiKey_T)) {
    const size_t sp = ImGui::IsKeyDown(ImGuiKey_LeftCtrl) ? 1 : 0;
    trail_[sp] = !trail_[sp];
    return true;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_B)) {
    dimm_ *= ImGui::IsKeyDown(ImGuiKey_LeftShift) ? 1.05f : 0.95f;
    if (ImGui::IsKeyDown(ImGuiKey_LeftCtrl)) {
      dimm_ = 1.f;
    }
    return true;
  }
  if (ImGui::IsKeyDown(ImGuiKey_LeftCtrl)) {
    for (auto m = 0; m < CameraMode::MaxMode; ++m) {
      if (ImGui::IsKeyPressed(static_cast<ImGuiKey>(static_cast<int>(ImGuiKey_0) + m))) {
        mode(static_cast<CameraMode>(m));
      }
    }
    return true;
  }
  return false;
}


void pip_t::onMouseWheel(AppWin* appwin, float delta) {
  if (delta != 0.f) {
    if (ImGui::IsKeyDown(ImGuiKey_LeftAlt)) {
      cam_fov *= ((delta > 0.f) ? 1.01 : 0.99);
      cam_fov = std::clamp(cam_fov, 5.0, 80.0);
    }
    else {
      switch (cam_mode) {
      case CameraMode::Low:
      case CameraMode::Top:
      case CameraMode::Local:
        cam_distance *= ((delta > 0) ? 1.02 : 0.98);
        cam_distance = std::clamp(cam_distance, 0.5, 100.0);
        break;
      }
    }
  }
}


void pip_t::mode(CameraMode mode) {
  if (mode != cam_mode) { cam_yaw_ = cam_pitch_ = 0.0; }
  cam_mode = mode;
}

void pip_t::update(const glm::ivec4& win_area) {
  const auto vp = glm::dvec4(
    vp_prop[0] * win_area[2],
    vp_prop[1] * win_area[3],
    vp_prop[2] * win_area[2],
    vp_prop[3] * win_area[3]
  );
  auto bottom = win_area[1] + win_area[3];
  hit_rect[0] = win_area[0] + vp[0];
  hit_rect[1] = bottom - (vp[1] + vp[3]);
  hit_rect[2] = hit_rect[0] + vp[2];
  hit_rect[3] = hit_rect[1] + vp[3];

  const auto pos = (cam_follow.pInstance) ? glm::dvec3(cam_follow.pInstance->B[3]) : glm::dvec3(1, 1, 1);
  const auto dir = (cam_follow.pInstance) ? glm::dvec3(cam_follow.pInstance->B[0]) : glm::dvec3(1, 0, 0);
  switch (cam_mode) {
  case CameraMode::Map: {
    const double y = pos.y;
    const auto eye = glm::dvec3(0, 0, 0);
    const auto center = glm::dvec3(0, 0, 10);
    const auto up = glm::dvec3(1, 0, 0);
    cam_.setViewMatrix(lookAt(eye, center, up));
    cam_.setPerspectiveViewport(vp, glm::radians(cam_fov), 0.1, 1000);
    break;
  }
  case CameraMode::Television: {
    const auto eye = eye_;
    const auto center = pos;
    const auto up = glm::dvec3(0, 1, 0);
    cam_.setViewMatrix(lookAt(eye, center, up));
    cam_.setPerspectiveViewport(vp, glm::radians(cam_fov), 0.1, 1000);
    break;
  }
  case CameraMode::Low:
  case CameraMode::Top: {
    const auto center = pos;
    const auto eye = center + ((cam_mode == CameraMode::Low) ? -1.0 : 1.0) * glm::dvec3(glm::dvec4(0, cam_distance, 0.f, 0.0));
    const auto up = (cam_mode == CameraMode::Low) ? glm::dvec3(0, 0, 1) : glm::dvec3(0, 0, -1);
    cam_.setViewMatrix(lookAt(eye, center, up));
    cam_.setPerspectiveViewport(vp, glm::radians(cam_fov), 0.1, 1000);
    break;
  }
  case CameraMode::Local: {
    const auto R = yaw_pitch(cam_yaw_, cam_pitch_);
    const auto center = pos;
    const auto eye = center + cam_distance * (R * glm::dvec3(-1,0,0));
    const auto up = glm::dvec3(0, 1, 0);
    cam_.setViewMatrix(lookAt(eye, center, up));
    cam_.setPerspectiveViewport(vp, glm::radians(cam_fov), 0.1, 1000);
    break;
  }
  }
}


glm::dmat4 pip_t::lookAt(const glm::dvec3 & eye, const glm::dvec3 & center, const glm::dvec3 & up) {
  cam_dist_ = glm::distance(eye, center);
  return glm::lookAt(eye, center, up);
}

