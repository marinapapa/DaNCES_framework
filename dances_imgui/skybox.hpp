#pragma once

#include <glsl/glsl.hpp>
#include <model/json.hpp>


class Skybox
{
public:
  explicit Skybox(const json& J);
  ~Skybox();

  void flush();
  void render(float dimm);

private:
  GLuint vao_ = GL_NONE;       // dummy one
  GLuint prog_ = GL_NONE;
  GLuint tex_ = GL_NONE;
};
