#define _USE_MATH_DEFINES

#include <cmath>
#include <exe_path.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glmutils/perp_dot.hpp>
#include <glmutils/ray.hpp>
#include <glmutils/homogeneous.hpp>
#include <glsl/ac3d.hpp>
#include <glsl/debug.h>
#include <glsl/imm.hpp>
#include <glsl/shader.h>
#include <GLFW/glfw3.h>
#include "camera.hpp"
#include "Renderer.h"
#include "AppWin.h"
#include "skybox.hpp"


namespace filesystem = std::filesystem;
using namespace shader;
using model::instance_proxy;


namespace gl {

  namespace {

    constexpr GLbitfield flush_map_flags = GL_MAP_WRITE_BIT | GL_MAP_PERSISTENT_BIT | GL_MAP_COHERENT_BIT;


    void WaitForSync(GLsync sync) {
      if (!sync || !glIsSync(sync)) return;
      GLenum ws = ws = glClientWaitSync(sync, GL_SYNC_FLUSH_COMMANDS_BIT, 0);
      while (!(ws == GL_ALREADY_SIGNALED || ws == GL_CONDITION_SATISFIED)) {
        ws = glClientWaitSync(sync, 0, 10000000);
        if (ws == GL_ALREADY_SIGNALED || ws == GL_CONDITION_SATISFIED) break;
        if (ws == GL_WAIT_FAILED) throw std::runtime_error("glClientWaitSync failed");
      }
    }


    void DeleteSync(GLsync& sync) {
      if (sync) glDeleteSync(sync);
      sync = nullptr;
    }


    struct trail_config
    {
      trail_config() = default;
      explicit trail_config(const json& J);

      size_t length;
      size_t interval;   // [ticks]
      float width;
      float tickInterval;
      float tickWidth;
      glm::vec2 fade;
      float ambient;
    };


    struct Trail
    {
      Trail() = default;
      Trail(const Trail& rhs) = default;
      Trail(Trail&& rhs) = default;
      Trail& operator=(Trail&& rhs) noexcept;

      Trail(const char* name, const trail_config& tconfig, const json& J);
      ~Trail();
      void push_back(GLuint src_vbo);
      void render() const;

      GLsizeiptr N = 0;
      GLsizeiptr size = 0;
      GLsizeiptr capacity = 0;
      std::array<GLuint, 2> ssbo = { GL_NONE, GL_NONE };    // ping pong
      GLuint vao = GL_NONE;   // dummy
      size_t interval = 0;
    };


    struct species
    {
      species(const species&) = delete;
      species(species&&) noexcept;
      species& operator=(species&&) noexcept;

      species() = default;
      species(const std::filesystem::path& binPath, const trail_config& tconfig, const char* name, const json& J);
      ~species();

      model::instance_proxy* proxy(size_t idx) const noexcept { return pInstance + idx; }
      void render_trail() const;
      void render(shader::material_t* pMaterial) const;
        
      GLsizeiptr size = 0;
      GLuint tex = GL_NONE;
      float scale = 1.f;
      GLuint vao = GL_NONE;
      GLuint vbo_vert = GL_NONE;
      GLuint vbo_idx = GL_NONE;
      GLuint vbo_inst = GL_NONE;
      model::instance_proxy* pInstance = nullptr;
      Trail trail;
      glsl::ac3d_model ac;
    };


    trail_config::trail_config(const json& J) {
      double dt = double(J["Simulation"]["dt"]);
      const auto jt = J["gui"]["Trails"];
      double tdt = jt["interval"];
      interval = static_cast<size_t>(tdt / dt);
      length = static_cast<model::tick_t>(double(jt["length"]) * double(J["gui"]["fps"]["gui"]));
      width = jt["width"];
      tickInterval = jt["tickInterval"];
      tickWidth = jt["tickWidth"];
      fade.x = jt["fade"][0];
      fade.y = jt["fade"][1];
      ambient = jt["ambient"];
    }


    Trail& Trail::operator=(Trail&& rhs) noexcept {
      N = rhs.N; rhs.N = 0;
      size = rhs.size; rhs.size = 0;
      capacity = rhs.capacity; rhs.capacity = 0;
      ssbo = rhs.ssbo; rhs.ssbo = { GL_NONE, GL_NONE };
      vao = rhs.vao; rhs.vao = GL_NONE;
      return *this;
    }


    Trail::Trail(const char* name, const trail_config& tconfig, const json& J) {
      N = J[name]["N"];
      size = 0;
      capacity = static_cast<GLsizeiptr>(tconfig.length);
      glGenBuffers(2, ssbo.data());
      const auto elem_size = static_cast<GLsizeiptr>(sizeof(instance_proxy));
      const auto row_size = static_cast<GLsizeiptr>(N * sizeof(instance_proxy));
      glGenVertexArrays(1, &vao);   // dummy vertex array object
      for (auto& ss : ssbo) {
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, ss);
        glNamedBufferStorage(ss, capacity * row_size, nullptr, GL_NONE);
      }
      glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    }


    Trail::~Trail() {
      glDeleteVertexArrays(1, &vao);
      glDeleteBuffers(2, ssbo.data());
    }


    void Trail::push_back(GLuint src_vbo) {
      const auto row_size = static_cast<GLsizeiptr>(N * sizeof(instance_proxy));
      glCopyNamedBufferSubData(
        ssbo[0],
        ssbo[1],
        0,
        row_size,
        (capacity - 1) * row_size
      );
      glCopyNamedBufferSubData(
        src_vbo,
        ssbo[1],
        0,
        0,
        row_size 
      );
      size = std::min(size + 1, capacity);
      std::swap(ssbo[0], ssbo[1]);
    }


    void Trail::render() const {
      glBindVertexArray(vao);
      glDisable(GL_CULL_FACE);
      glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo[0]);
      glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, ssbo[0]);
      glUniform1i(0, GLint(N));
      glUniform1f(1, GLfloat(size));
      glDrawArraysInstanced(GL_LINE_STRIP_ADJACENCY, 0, size, N);
      glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
      glEnable(GL_CULL_FACE);
    }


    species::species(species&& rhs) noexcept :
      species()
    {
      *this = std::move(rhs);
    }


    species& species::operator=(species&& rhs) noexcept {
      this->~species();
      size = rhs.size; rhs.size = 0;
      tex = rhs.tex; rhs.tex = GL_NONE;
      scale = rhs.scale;
      vao = rhs.vao; rhs.vao = GL_NONE;
      vbo_vert = rhs.vbo_vert; rhs.vbo_vert = GL_NONE;
      vbo_idx = rhs.vbo_idx; rhs.vbo_idx = GL_NONE;
      vbo_inst = rhs.vbo_inst; rhs.vbo_inst = GL_NONE;
      pInstance = rhs.pInstance; rhs.pInstance = nullptr;
      trail = std::move(rhs.trail);
      ac = std::move(rhs.ac);
      return *this;
    }


    species::species(const std::filesystem::path& binPath, const trail_config& tconfig, const char* name, const json& J) :
      trail(name, tconfig, J)
    {
      size = J[name]["N"];
      scale = float(J[name]["scale"]) * float(J[name]["aero"]["wingSpan"]);
      ac = glsl::ImportAC3D((binPath / "media" / "ac3d" / "objects" / std::string(J[name]["shape"])).string());
      if (ac.texFile.empty()) {
        ac.texFile = "whitePix.png";
      }
      // flip coordinate system
      auto F = glm::rotate(glm::mat4(1), glm::radians(90.f), glm::vec3(1, 0, 0));
      F = glm::rotate(F, glm::radians(90.0f), glm::vec3(0, 1, 0));
      for (auto& p : ac.vertices) {
        p.v = F * p.v;
        p.n = F * glm::vec4(p.n, 0.f);
      }
      tex = LoadTexture(GL_TEXTURE0 + 1, binPath / "media" / "ac3d" / "objects" / ac.texFile);
      glGenVertexArrays(1, &vao);
      glGenBuffers(1, &vbo_vert);
      glGenBuffers(1, &vbo_idx);
      glGenBuffers(1, &vbo_inst);

      glBindVertexArray(vao);
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo_idx);
      glBufferStorage(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLint) * ac.indices.size(), ac.indices.data(), GL_NONE);
      glBindBuffer(GL_ARRAY_BUFFER, vbo_vert);
      glBufferStorage(GL_ARRAY_BUFFER, sizeof(glsl::V4F_N3F_T2S) * ac.vertices.size(), ac.vertices.data(), GL_NONE);
      glEnableVertexAttribArray(0);
      glEnableVertexAttribArray(1);
      glEnableVertexAttribArray(2);
      glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(glsl::V4F_N3F_T2S), (void*)0);
      glVertexAttribPointer(1, 3, GL_FLOAT, GL_TRUE, sizeof(glsl::V4F_N3F_T2S), (void*)(offsetof(struct glsl::V4F_N3F_T2S, n)));
      glVertexAttribPointer(2, 2, GL_SHORT, GL_TRUE, sizeof(glsl::V4F_N3F_T2S), (void*)(offsetof(struct glsl::V4F_N3F_T2S, t)));

      glBindBuffer(GL_ARRAY_BUFFER, vbo_inst);
      glBufferStorage(GL_ARRAY_BUFFER, size * sizeof(instance_proxy), nullptr, flush_map_flags);
      pInstance = (instance_proxy*)glMapBufferRange(GL_ARRAY_BUFFER, 0, size * sizeof(instance_proxy), flush_map_flags);
      glEnableVertexAttribArray(3);
      glEnableVertexAttribArray(4);
      glEnableVertexAttribArray(5);
      glEnableVertexAttribArray(6);
      glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(instance_proxy), (void*)0);                            // forward
      glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, sizeof(instance_proxy), (void*)(2ull * sizeof(glm::vec4)));   // side
      glVertexAttribPointer(5, 4, GL_FLOAT, GL_FALSE, sizeof(instance_proxy), (void*)(3ull * sizeof(glm::vec4)));   // pos
      glVertexAttribPointer(6, 4, GL_FLOAT, GL_FALSE, sizeof(instance_proxy), (void*)(4ull * sizeof(glm::vec4)));   // [speed,tex,alpha,_]
      glVertexAttribDivisor(3, 1);
      glVertexAttribDivisor(4, 1);
      glVertexAttribDivisor(5, 1);
      glVertexAttribDivisor(6, 1);
      glBindVertexArray(0);
    }


    species::~species() {
      glDeleteVertexArrays(1, &vao);
      glDeleteBuffers(1, &vbo_vert);
      glDeleteBuffers(1, &vbo_idx);
      glDeleteBuffers(1, &vbo_inst);
      glDeleteTextures(1, &tex);
      pInstance = nullptr;        // debug
    }


    void species::render(shader::material_t* pMaterial) const {
      std::memcpy(pMaterial, &ac.material, sizeof(ac.material));
      if (ac.twoSided) {
        glDisable(GL_CULL_FACE);
      }
      glBindVertexArray(vao);
      glActiveTexture(GL_TEXTURE0 + 1);
      glBindTexture(GL_TEXTURE_2D, tex);
      glUniform1f(0, scale);
      glUniform1f(1, ac.material.amb[0]);
      glDrawElementsInstanced(GL_TRIANGLES,
        static_cast<GLsizeiptr>(ac.indices.size()),
        GL_UNSIGNED_INT, (GLvoid*)(0),
        size);
      glBindVertexArray(0);
      if (ac.twoSided) {
        glEnable(GL_CULL_FACE);
      }
    }


    void species::render_trail() const {
      trail.render();
    }


    template <size_t I, typename SA>
    void init_species(const std::filesystem::path& binPath, const trail_config& tconfig, const json& J, SA& out) {
      using agent_type = typename std::tuple_element_t<I, model::species_pop>::value_type;
      out[I] = std::move(species(binPath, tconfig, agent_type::name(), J));
      if constexpr ((I + 1) < std::tuple_size_v<SA>) {
        init_species<I + 1>(binPath, tconfig, J, out);
      }
    }

  }


  class Renderer : public RendererBase
  {
  public:
    Renderer(const json& J, class GLFWwindow* window);
    ~Renderer() override;

    nearest_instance_record instance(size_t tag, size_t idx) const override;
    nearest_instance_record nearest_instance(size_t tag, const pip_t& pip, int screenX, int screenY) const override;
    void flush(const class AppWin& app, const model::Simulation& sim, bool trials) override;
    void render(const pip_t& pip) override;

    static constexpr size_t n_species = std::tuple_size_v<model::species_pop>;
    using species_array = std::array<class gl::species, n_species>;

  private:
    enum GLVBO {
      VBO_UNIFORM,    // uniform buffer block: shader::uniformBlock_t
      VBO_MATERIAL,   // uniform buffer block: shader::material_t
      VBO_MAX
    };

    GLFWwindow* window_ = nullptr;
    std::array<GLuint, VBO_MAX> vbo_;
    GLuint spectrumTex_ = 0;
    GLuint envTex_ = 0;
    const float dt_;
    model::tick_t tick_;

    GLuint instanceProg_ = GL_NONE;
    GLuint immProg_ = GL_NONE;
    GLuint ribbonProg_ = GL_NONE;
    GLsync flush_sync_ = nullptr;
    shader::uniformBlock_t* pUniBlk_ = nullptr;
    shader::material_t* pMaterial_ = nullptr;

    // species
    species_array species_;
    gl::trail_config trail_config_;

    std::unique_ptr<Skybox> skybox_;
    glm::ivec2 displayExt_;
  };


  Renderer::Renderer(const json& J, class GLFWwindow* window) :
    window_(window),
    vbo_{ GL_NONE }, 
    dt_(J["Simulation"]["dt"]),
    displayExt_(100,100)
  {
    // Uniform buffer blocks
    glGenBuffers(static_cast<GLsizeiptr>(vbo_.size()), vbo_.data());
    glBindBuffer(GL_UNIFORM_BUFFER, vbo_[VBO_UNIFORM]);
    glBufferStorage(GL_UNIFORM_BUFFER, sizeof(uniformBlock_t), nullptr, flush_map_flags);
    glBindBufferBase(GL_UNIFORM_BUFFER, 1, vbo_[VBO_UNIFORM]);
    pUniBlk_ = (shader::uniformBlock_t*)glMapBufferRange(GL_UNIFORM_BUFFER, 0, sizeof(shader::uniformBlock_t), flush_map_flags);

    // Uniform buffer blocks
    glBindBuffer(GL_UNIFORM_BUFFER, vbo_[VBO_MATERIAL]);
    glBufferStorage(GL_UNIFORM_BUFFER, sizeof(uniformBlock_t), nullptr, flush_map_flags);
    glBindBufferBase(GL_UNIFORM_BUFFER, 3, vbo_[VBO_MATERIAL]);
    pMaterial_ = (shader::material_t*)glMapBufferRange(GL_UNIFORM_BUFFER, 0, sizeof(shader::material_t), flush_map_flags);

    // Species
    const auto binPath = exe_path::get();
    trail_config_ = gl::trail_config(J);
    instanceProg_ = ProgFromLiterals(instanceVertex, instanceFragment);
    init_species<0>(binPath, trail_config_, J, species_);

    // Ribbons
    ribbonProg_ = ProgFromLiterals(ribbonVertex, ribbonFragment, ribbonGeo);
    glUseProgram(ribbonProg_);
    glUniform2f(2, trail_config_.fade[0], trail_config_.fade[1]);
    glUniform1f(3, trail_config_.ambient);
    glUniform1f(4, 0.5f * trail_config_.width);

    // other stuff
    spectrumTex_ = LoadTexture(GL_TEXTURE0 + 2, binPath / "media" / "spectrum.png");
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_CULL_FACE);

    // skybox
    skybox_.reset(new Skybox(J));
  }


  Renderer::~Renderer() {
    glUseProgram(0);
    skybox_.reset(nullptr);
    glDeleteBuffers(static_cast<GLsizeiptr>(vbo_.size()), vbo_.data());
    glDeleteTextures(1, &spectrumTex_);
    glDeleteTextures(1, &envTex_);
    glDeleteProgram(instanceProg_);
    glDeleteProgram(immProg_);
    glDeleteProgram(ribbonProg_);
    DeleteSync(flush_sync_);
  }


  nearest_instance_record Renderer::instance(size_t tag, size_t idx) const {
    auto nnr = nearest_instance_record();
    if (tag < species_.size() && idx < species_[tag].size) {
      auto* pi = species_[tag].pInstance + idx;
      nnr = { pi, tag, idx, 0 };
    }
    return nnr;
  }


  nearest_instance_record Renderer::nearest_instance(size_t tag, const pip_t& pip, int screenX, int screenY) const {
    auto nnr = nearest_instance_record();
    const auto& camera = pip.cam();
    const auto CSD = pip.screenDirection(pip.viewport_coor(screenX, screenY));
    auto mindd = std::numeric_limits<double>::max();
    auto pi = species_[tag].pInstance;
    for (size_t i = 0; i < species_[tag].size; ++i, ++pi) {
      float dd = glmutils::distanceSqRayPoint(camera.eye(), CSD, glm::dvec3(pi->B[3]));
      if (dd < mindd) {
        nnr = { pi, tag, i, dd };
        mindd = dd;
      }
    }
    return nnr;
  }


  template <size_t I>
  void flush_species(Renderer* self, const model::Simulation& sim, Renderer::species_array& gls) {
    using Tag = std::integral_constant<size_t, I>;
    auto pInst = gls[I].pInstance;
    gls[I].size = static_cast<GLsizeiptr>(sim.visit_all<std::integral_constant<size_t, I>>([psim = &sim, p = pInst](const auto& ind, size_t idx) mutable {
      *p = ind.instance_proxy(idx, psim);
      p->alpha = 1.f;
      ++p;
    }));
    if constexpr ((I + 1) < model::n_species) {
      flush_species<I + 1>(self, sim, gls);
    }
  }


  void Renderer::flush(const AppWin& app, const model::Simulation& sim, bool trails) {
    WaitForSync(flush_sync_);
    DeleteSync(flush_sync_);
    tick_ = sim.tick();
    flush_species<0>(this, sim, species_);
    if (trails) {
      for (auto& sp : species_) sp.trail.push_back(sp.vbo_inst);
    }
    flush_sync_ = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
  }


  void Renderer::render(const pip_t& pip) {
    WaitForSync(flush_sync_);
    DeleteSync(flush_sync_);
    const auto& camera = pip.cam();
    auto vp = glm::ivec4(camera.viewport());
    glViewport(vp[0], vp[1], vp[2], vp[3]);
    const glm::mat4 MV = glm::mat4(camera.V());
    auto uniformBlk = uniformBlock_t{
      glm::mat4(camera.V()),
      glm::mat4(camera.P()),
      glm::mat4(MV),
      glm::mat4(camera.P()) * MV,
      glm::mat4(glm::transpose(glm::inverse(MV))),
      glm::mat4(glm::ortho(0.f, float(displayExt_.x), float(displayExt_.y), 0.f))
    };
    *pUniBlk_ = uniformBlk;
    skybox_->render(pip.dimm());
    glActiveTexture(GL_TEXTURE0 + 2);
    glBindTexture(GL_TEXTURE_2D, spectrumTex_);
    glEnable(GL_DEPTH_TEST);
    glUseProgram(instanceProg_);
    for (const auto& sp : species_) {
      sp.render(pMaterial_);
    }
    glDisable(GL_DEPTH_TEST);
    const auto trails = pip.trail();
    if (trails[0] || trails[1]) {
      glEnable(GL_BLEND);
      glUseProgram(ribbonProg_);
      for (size_t i = 0; i < species_.size(); ++i) {
        if (trails[i]) species_[i].render_trail();
      }
      glDisable(GL_BLEND);
    }
    flush_sync_ = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
  }

}


std::unique_ptr<RendererBase> create_gl_renderer(const json& J, class GLFWwindow* window) {
  return std::unique_ptr<RendererBase>(new gl::Renderer(J, window));
}

