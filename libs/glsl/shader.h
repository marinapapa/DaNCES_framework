#ifndef SHADER_H_INCLUDED
#define SHADER_H_INCLUDED

#include <filesystem>
#include <vector>
#include <filesystem>
#include <glm/glm.hpp>
#include <glsl/glsl.hpp>


namespace shader {


  // from glsl/ac3d.h
  struct material_t
  {
    glm::vec3 rgb;
    glm::vec3 amb;
    glm::vec3 emis;
    glm::vec3 spec;
    float shi;
    float trans;
  };


  struct uniformBlock_t
  {
    glm::mat4 V;
    glm::mat4 P;
    glm::mat4 MV;
    glm::mat4 MVP;
    glm::mat4 ITMV;
    glm::mat4 Ortho;      // ortho
  };


  extern const char* uniformBlock;

  // e.g. birds
  extern const char* instanceVertex;
  extern const char* instanceFragment;
  
  extern const char* immVertexInst;
  extern const char* immVertex;
  extern const char* immFragment;
  
  extern const char* ribbonVertex;
  extern const char* ribbonGeo;
  extern const char* ribbonFragment;

  extern const char* skyboxVertex;
  extern const char* skyboxFragment;


  GLuint ProgFromLiterals(const char* vertexShader, const char* fragmentShader, const char* geometyShader = nullptr);
  GLuint LoadTexture(GLenum texUnit, const std::filesystem::path& path);

  // Filenames expected to be +X,-X,+Y,-Y,+Z,-Z
  GLuint LoadCubeMapTexture(GLenum texUnit, const std::vector<std::filesystem::path>& FileNames);

}

#endif
