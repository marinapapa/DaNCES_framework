#include <string>
#include <exception>
#include <filesystem>
#include <stdexcept>
#include <glm/gtc/matrix_transform.hpp>
#include <glsl/stb_image.h>
#include <glsl/shader.h>
#include <libs/exe_path.hpp>
#include "skybox.hpp"


Skybox::Skybox(const json& J) {
  std::filesystem::path sbf = exe_path::get() / "media" / "skybox" / std::string(J["gui"]["skybox"]["textures"]);
  if (!std::filesystem::is_directory(sbf)) throw std::runtime_error("skybox folder doesn't exists");
  std::vector<std::filesystem::path> faces;
  for (size_t i = 0; i < 6; ++i) {
    faces.emplace_back(sbf / (std::to_string(i) + ".png"));
  }
  glGenVertexArrays(1, &vao_);
  tex_ = shader::LoadCubeMapTexture(GL_TEXTURE0 + 3, faces);
  prog_ = shader::ProgFromLiterals(shader::skyboxVertex, shader::skyboxFragment, nullptr);
}


Skybox::~Skybox() {
  glUseProgram(0);
  glDeleteVertexArrays(1, &vao_);
  glDeleteProgram(prog_);
  glDeleteTextures(1, &tex_);
}


void Skybox::flush() {
}


void Skybox::render(float dimm) {
  glUseProgram(prog_);
  glUniform1f(0, dimm);
  glBindVertexArray(vao_);
  glActiveTexture(GL_TEXTURE0 + 3);
  glDepthFunc(GL_ALWAYS);
  glDrawArrays(GL_TRIANGLES, 0, 12*3);
  glDepthFunc(GL_LEQUAL);
  glBindVertexArray(0);
}
