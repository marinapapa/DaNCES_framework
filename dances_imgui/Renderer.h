#pragma once

#include <memory>
#include <mutex>
#include <model/json.hpp>
#include <agents/agents.hpp>
#include <glsl/ac3d.hpp>
#include "skybox.hpp"
#include "camera.hpp"


struct nearest_instance_record 
{
  model::instance_proxy* pInstance = nullptr;
  size_t species = 0;
  size_t idx = 0;
  float dist2 = 0.f;
};


class RendererBase
{
public:
  virtual ~RendererBase() {};

  virtual nearest_instance_record instance(size_t tag, size_t idx) const = 0;
  virtual nearest_instance_record nearest_instance(size_t tag, const class pip_t&, int screenX, int screenY) const = 0;
  virtual void flush(const class AppWin& app, const model::Simulation& sim, bool trails) = 0;
  virtual void render(const class pip_t& pip) = 0;
};


std::unique_ptr<RendererBase> create_gl_renderer(const json& J, class GLFWwindow* window);

