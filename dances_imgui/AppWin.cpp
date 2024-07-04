#include <iostream>
#include <stdio.h>
#include <thread>
#include <glsl/debug.h>
#include <libs/exe_path.hpp>
#include <implot.h>
#include "imgui_guard.hpp"
#include "AppWin.h"


using namespace std::chrono;


AppWin::AppWin(imgui_guard* imgg, const json& J) : imgg_(imgg) {
  ImPlot::CreateContext();
  glfwGetFramebufferSize(window(), &win_rect_[2], &win_rect_[3]);
  glfwGetWindowPos(window(), &win_rect_[0], &win_rect_[1]);
  renderer_ = create_gl_renderer(J, imgg_->window());

  const auto& jg = J["gui"];
  const size_t nh = jg["handler"].size();
  for (size_t i = 0; i < nh; ++i) {
    imgui_handlers_.emplace_back(create_imgui_handler(this, jg["handler"][i]));
  }
  paused_ = jg["fps"]["paused"];
  speedup_ = jg["fps"]["speedup"];
  realtime_ = jg["fps"]["realtime"];
  throttle_ = jg["fps"]["throttle"];
  debug_ = optional_json<bool>(jg["fps"], "debug").value_or(false);
  const auto& npips = jg["pip"].size();
  for (size_t pip = 0; pip < npips; ++pip) {
    pips_.emplace_back(J, pip);
  }
}


AppWin::~AppWin()
{
  renderer_.reset(nullptr);
  ImPlot::DestroyContext();
}


void AppWin::run_simulation(model::Simulation* sim,
                            const model::species_instances& ss,
                            model::Observer* observer,
                            const json& J) {
  interactive_pip_ = &pips_[0];
  game_watches::stop_watch watch;
  watch.start();
  const auto gui_dt = duration_cast<microseconds>(duration<double>(1.0 / double(J["gui"]["fps"]["gui"])));
  const auto fps_dt = 1'000'000us;  
  const auto flush_ticks = static_cast<size_t>(double(J["gui"]["Trails"]["interval"]) / sim->dt());
  auto next_sim_time = watch.elapsed<microseconds>();
  auto next_flush_tick = 0;
  auto next_gui_time = watch.elapsed<microseconds>();
  auto next_fps_time = watch.elapsed<microseconds>();
  auto Tmax = sim->time2tick(double(J["Simulation"]["Tmax"]));
  sim->initialize(observer, ss);
  renderer_->flush(*this, *sim, true);

  // select any camera target
  for (auto& pip : pips_) {
    pip.update(win_rect_);
    pip.follow(renderer_->nearest_instance(model::prey_tag::value, pip, 0, 0));
  }
  update_watch_.reset();
  game_watches::stop_watch<> fps_watch{};
  fps_watch.start();
  size_t sim_ticks = 0;
  size_t gui_ticks = 0;
  app_watch_.start();
  auto* window = imgg_->window();
  while (!glfwWindowShouldClose(window)) {
    auto now = watch.elapsed<microseconds>();
    bool worked = false;
    if (paused_) {
      next_sim_time = now;
    }
    if (single_ || (!paused_ && (now >= next_sim_time))) {
      ++sim_ticks;
      update_watch_.start();
      sim->update(observer);
      update_watch_.stop();
      if (sim->tick() == Tmax) {
        break;
      }
      if (sim->tick() >= next_flush_tick) {
        renderer_->flush(*this, *sim, true);
        next_flush_tick += flush_ticks;
      }
      if (throttle_) {
        const auto sim_dt = duration_cast<microseconds>(duration<double>(sim->dt() / speedup_));
        next_sim_time += sim_dt;
      }
      worked = true;
      single_ = false;
    }
    if (now >= next_gui_time) {
      if (!worked) {
        renderer_->flush(*this, *sim, false);
      }
      ++gui_ticks;
      auto old_app_time = app_watch_.elapsed();
      if (gui_frame(sim, observer) || (debug_ && !paused_)) {
        // dragging or resizing of main window occurred
        // we try to 'forget' the time lapsed for this
        app_watch_.sync_with(old_app_time);
        watch.sync_with(now);
        if (!paused_) {
          app_watch_.start();
          watch.start();
        }
      }
      next_gui_time += gui_dt;
      worked = true;
    }
    if (!worked) {
      if (paused_ || ((next_sim_time - now) > 10ms)) {
        std::this_thread::sleep_for(5ms);
      }
      else {
        std::this_thread::yield();
      }
    }
    if (now >= next_fps_time) {
      fps_watch.stop();
      const auto elapsed = fps_watch.elapsed<microseconds>().count();
      sim_fps_ = (1'000'000 * sim_ticks) / elapsed;
      gui_fps_ = (1'000'000 * gui_ticks) / elapsed;
      sim_ticks = gui_ticks = 0;
      fps_watch.restart();
      next_fps_time = now + fps_dt;
    }
  }
  observer->notify(model::Simulation::Finished, *sim);
}


// returns true if window size was altered
bool AppWin::gui_frame(model::Simulation* sim, model::Observer* observer) {
  auto old_ws = imgg_->win_size();
  auto* window = imgg_->window();
  glfwGetFramebufferSize(window, &win_rect_[2], &win_rect_[3]);
  glfwGetWindowPos(window, &win_rect_[0], &win_rect_[1]);

  for (auto& pip : pips_) {
    pip.update(win_rect_);
  }
  glfwPollEvents();
  ImGui_ImplGlfw_NewFrame();
  ImGui_ImplOpenGL3_NewFrame();
  ImGui::NewFrame();

  auto& io = ImGui::GetIO();
  if (glfwGetWindowAttrib(window, GLFW_FOCUSED)) {
    if (!io.WantCaptureMouse) sim_gui_mouse(sim);
    if (!io.WantCaptureKeyboard) sim_gui_key(sim);
  }

  if (show_gui_) {
    ImGui::Begin("Dances ImGui", &show_gui_);
    if (ImGui::CollapsingHeader("Control panel")) {
      auto p = paused_;
      ImGui::Checkbox("pause", &p);
      if (p != paused_) {
        toggle_pause();
      }
      if (paused_) {
        ImGui::SameLine();
        single_ = ImGui::Button("single step");
      }
      ImGui::Checkbox("realtime", &realtime_);
      if (realtime_) {
        speedup_ = 1.0;
        throttle_ = true;
      }
      else {
        ImGui::Checkbox("throttle", &throttle_);
        if (throttle_) {
          float s = speedup_;
          ImGui::SliderFloat("speedup", &s, 0.1f, 10.0f);
          speedup_ = s;
        }
      }
      auto fovy = static_cast<float>(interactive_pip_->fovy());
      ImGui::SliderFloat("camera fovy", &fovy, 5.0f, 70.0f);
      interactive_pip_->cam_fov = fovy;
      if (ImGui::CollapsingHeader("Profiling")) {
        const auto awt = app_watch_.elapsed_seconds();
        const auto swt = sim->tick2time(sim->tick());
        ImGui::Text("App wall time: %.2f s", awt);
        if ((awt/swt) > 1.01) {
          ImGui::TextColored(ImVec4(1.0f, 0.0f, 1.0f, 1.0f), "Sim wall time: %.2f s", swt);
        }
        else {
          ImGui::Text("Sim wall time: %.2f s", swt);
        }
        if (ImGui::Button("re-sync wall times")) {
          app_watch_.sync_with(std::chrono::duration<double>(sim->tick2time(sim->tick())));
          if (!paused_) app_watch_.start();
        }
        auto us = update_watch_.elapsed<std::chrono::microseconds>().count();
        auto tt = us / sim->tick();
        auto ss = sim_watch_.elapsed<std::chrono::microseconds>().count();
        auto st = ss / sim->tick();
        ImGui::Text("Sim update time: %ld us", tt);
        ImGui::Text("Sim FPS: %ld", sim_fps_);
        ImGui::Text("Gui FPS: %ld", gui_fps_);
      }
    }
    if (ImGui::CollapsingHeader("Handler")) {
      for (auto& h : imgui_handlers_) {
        bool enabled = h->enabled();
        ImGui::Checkbox(h->title().c_str(), &enabled);
        h->enable(enabled);
      }
      for (auto& h : imgui_handlers_) {
        h->pre_display(sim);
      }
      for (auto* obs = observer; obs; obs = obs->next()) {
        if (auto* gh = obs->gui_handler()) { gh->pre_display(sim); }
      }
      for (auto& h : imgui_handlers_) {
        h->display(sim);
      }
      for (auto* obs = observer; obs; obs = obs->next()) {
        if (auto* gh = obs->gui_handler()) { gh->display(sim); }
      }
    }
    ImGui::End();
  }

  glfwGetFramebufferSize(window, &win_rect_[2], &win_rect_[3]);
  glfwGetWindowPos(window, &win_rect_[0], &win_rect_[1]);
  glViewport(0, 0, win_rect_[2], win_rect_[3]);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  for (const auto&  pip : pips_) {
    if (pip.enabled()) renderer_->render(pip);
  }
  ImGui::Render();

  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

  // Update and Render additional Platform Windows
  // (Platform functions may change the current OpenGL context, so we save/restore it to make it easier to paste this code elsewhere.
  //  For this specific demo app we could also call glfwMakeContextCurrent(window) directly)
  if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
  {
    GLFWwindow* backup_current_context = glfwGetCurrentContext();
    ImGui::UpdatePlatformWindows();
    ImGui::RenderPlatformWindowsDefault();
    glfwMakeContextCurrent(backup_current_context);
  }
  glfwSwapBuffers(window);
  return (old_ws != imgg_->win_size());
}


// called if main window has focus and ImGui isn't interested
// returns true if window size was altered
void AppWin::sim_gui_mouse(model::Simulation* sim) {
  auto* window = imgg_->window();
  glm::dvec2 wpos;
  glfwGetCursorPos(window, &wpos.x, &wpos.y);
  glm::ivec2 spos{ win_rect_[0] + wpos.x, win_rect_[1] + wpos.y };
  for (auto& pip : pips_) {
    for (auto it = pips_.rbegin(); it != pips_.rend(); ++it) {
      if (it->hit_test(spos.x, spos.y)) {
        interactive_pip_ = &*it;
        break;
      }
    }
  }
  if (interactive_pip_->hit_test(spos.x, spos.y)) {
    auto& io = ImGui::GetIO();
    interactive_pip_->onMouseWheel(this, static_cast<float>(imgg_->mouse_wheel()));
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
      interactive_pip_->onMouseRClick(this, spos.x, spos.y);
    }
    if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
      interactive_pip_->onMouseDrag(this, ImGui::GetIO().MouseDelta.x, ImGui::GetIO().MouseDelta.y);
    }
  }
}


void AppWin::toggle_pause() {
  paused_ = !paused_;
  if (paused_) {
    app_watch_.stop();
  }
  else {
    app_watch_.start();
  }
}


// called if main window has focus and ImGui isn't interested
void AppWin::sim_gui_key(model::Simulation* sim)
{
  if (interactive_pip_->onKey(this)) {
    return;   // handled
  }
  if (ImGui::IsKeyPressed(ImGuiKey_Space)) {
    toggle_pause();
  }
  if (ImGui::IsKeyPressed(ImGuiKey_F1)) {
    show_gui_ = true;
  }
}
