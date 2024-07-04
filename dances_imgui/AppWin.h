#ifndef APPWIN_H_INCLUDED
#define APPWIN_H_INCLUDED

#ifndef WIN32_LEAN_AND_MEAN
  #define WIN32_LEAN_AND_MEAN
#endif

#ifdef __INTEL_COMPILER
#define ATL
(expr)   // avoid 'too many categories' assertion
#endif

#include <string>
#include <vector>
#include <map>
#include <functional>
#include <filesystem>
#include <model/simulation.hpp>
#include <model/agents/agents_fwd.hpp>
#include <model/observer.hpp>
#include <libs/game_watches.hpp>
#include "imgui_handler.hpp"
#include "imgui_unit.h"
#include "imgui_guard.hpp"
#include "pip.hpp"
#include "Renderer.h"


class AppWin
{
public:
  using ImGuiFun = std::function<void(model::Simulation* sim)>;
  static std::vector<ImGuiFun> ImGuiHandler;

public:
  AppWin(class imgui_guard* imgg, const json& J);
  virtual ~AppWin();

  bool is_paused() const noexcept { return paused_; }

  void run_simulation(model::Simulation* sim,
                      const model::species_instances& ss,
                      model::Observer* observer,
                      const json& J);

  pip_t* interactive_pip() { return interactive_pip_; }
  GLFWwindow* window() noexcept { return imgg_->window(); }

  // [left, top, right, bottom] in screen-coordinates 
  glm::ivec4 win_rect() const noexcept { return win_rect_; }

private:
  friend class pip_t;

  void update();
  void toggle_pause();
  bool gui_frame(model::Simulation* sim, model::Observer* observer);
  void sim_gui_mouse(model::Simulation* sim);
  void sim_gui_key(model::Simulation* sim);

  std::unique_ptr<class RendererBase> renderer_;
  glm::ivec4 win_rect_;    // main window [top, left, width, height] [screen coordinates]

  std::vector<pip_t> pips_;
  pip_t* interactive_pip_ = nullptr;

  bool paused_ = false;
  bool single_ = false;
  bool realtime_ = true;
  bool throttle_ = true;
  bool debug_ = false;
  double speedup_ = 1.0;
  bool show_gui_ = true;    // global switch

  // profiling
  game_watches::stop_watch<> update_watch_;
  game_watches::stop_watch<> sim_watch_;
  game_watches::stop_watch<> app_watch_;
  size_t sim_fps_ = 0;
  size_t gui_fps_ = 0;
  std::vector<std::unique_ptr<imgui_handler>> imgui_handlers_;
  imgui_guard* imgg_;
};


#endif
