#include <fstream>
#include "imgui_guard.hpp"


namespace {

  double mouse_wheel_xoffset = 0.0;
  double mouse_wheel_yoffset = 0.0;

  static void glfw_scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    mouse_wheel_xoffset = xoffset;
    mouse_wheel_yoffset = yoffset;
  }

  void glfw_error_callback(int error, const char* description) {
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
  }

}


double imgui_guard::mouse_wheel() noexcept {
  return std::exchange(mouse_wheel_yoffset, 0.0);
}


imgui_guard::imgui_guard() {
  if (std::filesystem::exists(exe_path::get() / "imgui.json")) {
    auto J = json::parse(std::ifstream(exe_path::get() / "imgui.json"));
    if (!(headless_ = J["gui"]["headless"])) {
      // Setup window
      glfwSetErrorCallback(glfw_error_callback);
      if (!glfwInit()) {
        throw std::runtime_error("failed to initialize GLFW");
      }

      // GL 4.5 + GLSL 440
      const char* glsl_version = "#version 450";
      glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
      glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
      glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
      glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_FALSE);
      int gl_debug_level = std::clamp(int(J["gui"]["opengl_debug_level"]), 0, int(glsl::GLSL_DEBUG_MSG_LEVEL::NOTIFICATION));
      if (0 != gl_debug_level) {
        glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GLFW_TRUE);
      }
      bool msaa = J["gui"]["opengl_msaa"];
      if (msaa) {
        glfwWindowHint(GLFW_SAMPLES, 4);
      }
      // Create window with graphics context
      glm::ivec4 wr = {};
      for (size_t i = 0; i < 4; ++i) {
        wr[i] = std::max(J["gui"]["win_rect"][i].get<int>(), 256);
      }
      window_ = glfwCreateWindow(wr[2], wr[3], "DaNCES Prey demo", NULL, NULL);
      if (window_ == nullptr) {
        throw std::runtime_error("failed to create OpneGL context");
      }
      glfwSetWindowPos(window_, wr[0], wr[1]);
      glfwMakeContextCurrent(window_);
      if (!gladLoadGL()) {
        ;
        throw std::runtime_error("failed to initialize OpenGL");
      };
      glsl::SetDebugCallback(static_cast<glsl::GLSL_DEBUG_MSG_LEVEL>(gl_debug_level));
      glfwSwapInterval(0); // Disable vsync
      if (msaa) {
        glEnable(GL_MULTISAMPLE);   // just in case
      }
      glfwSetScrollCallback(window_, glfw_scroll_callback);

      // dear ImGui
      //IMGUI_CHECKVERSION();
      ImGui::CreateContext();
      ImGuiIO& io = ImGui::GetIO(); (void)io;
      io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;       // Enable Keyboard Controls
      //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
      io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;           // Enable Docking
      io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;         // Enable Multi-Viewport / Platform Windows
      //io.ConfigViewportsNoAutoMerge = true;
      //io.ConfigViewportsNoTaskBarIcon = true;

      // Setup Dear ImGui style
      ImGui::StyleColorsDark();
      //ImGui::StyleColorsClassic();

      ImGuiStyle& style = ImGui::GetStyle();
      if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
        style.WindowRounding = 8.0f;
        style.Colors[ImGuiCol_WindowBg].w = 1.0f;
      }

      // Setup Platform/Renderer backends
      ImGui_ImplGlfw_InitForOpenGL(window_, true);
      ImGui_ImplOpenGL3_Init(glsl_version);

      // Load Fonts
      const auto font_path = exe_path::get() / "media" / "fonts";
      const auto cs = optional_json<float>(J["gui"]["fonts"], "content_scale").value_or(1.f);
      const auto& jf = J["gui"]["fonts"]["descr"];
      for (size_t i = 0; i < jf.size(); ++i) {
        const auto ttf = font_path / std::string(jf[i]["ttf"]);
        add_font({ ttf, float(jf[i]["pt"]), cs, std::string(jf[i]["moniker"])});
      }
      io.Fonts->AddFontDefault();
      io.Fonts->Build();
    }
  }
  // else headless_ = true
}


imgui_guard::~imgui_guard() {
  if (!headless_) {
    const auto file_path = exe_path::get() / "imgui.json";
    if (std::filesystem::exists(file_path)) {
      auto J = json::parse(std::ifstream(file_path));
      auto& ji = J["gui"];
      if (true == ji["save_on_exit"]) {
        auto ws = win_size();
        auto& jr = ji["win_rect"];
        jr[0] = ws[0];
        jr[1] = ws[1];
        jr[2] = ws[2];
        jr[3] = ws[3];
        auto os = std::ofstream(file_path);
        os << J;
      }
    }
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window_);
    glfwTerminate();
  }
}


void imgui_guard::add_font(font_descr font) {
  const auto dpi = this->dpi();
  const double scale = font.content_scale * this->content_scale() * dpi / 72.0;
  ImGuiIO& io = ImGui::GetIO();
  const auto fsize = static_cast<float>(scale * font.pt);
  const auto font_file = font.ttf;
  ImFontConfig config;
  if (fonts_.end() == fonts_.find(font.name)) {
    auto im_font = io.Fonts->AddFontFromFileTTF(font_file.string().c_str(), fsize, &config);
    if (im_font) {
      fonts_.emplace(font.name, std::make_pair(im_font, font));
    }
  }
}


ImFont* imgui_guard::get_font(const std::string& name) {
  auto it = fonts_.find(name);
  if (it != fonts_.end()) {
    return it->second.first;
  }
  return nullptr;
}


double imgui_guard::dpi() const {
  int height_mm;
  const auto monitor = glfwGetPrimaryMonitor();
  const auto video_mode = glfwGetVideoMode(monitor);
  glfwGetMonitorPhysicalSize(monitor, nullptr, &height_mm);
  const auto height_inch = 0.03937007874 * height_mm;
  return video_mode->height / height_inch;
}


double imgui_guard::content_scale() const {
  float xscale = 1.0f;
  glfwGetWindowContentScale(window_, &xscale, nullptr);
  return double{ xscale };
}


glm::ivec4 imgui_guard::win_size() const noexcept {
  glm::ivec4 ws = {};
  if (!headless_) {
    glfwGetWindowPos(window_, &ws[0], &ws[1]);
    glfwGetWindowSize(window_, &ws[2], &ws[3]);
  }
  return ws;
}


namespace {
  std::unique_ptr<imgui_guard> globalImgg;
}

void imgui_guard::init() {
  globalImgg.reset(new imgui_guard());
}


imgui_guard* imgui_guard::gImgg() {
  return globalImgg.get();
}