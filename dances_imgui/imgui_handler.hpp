#pragma once

#include <string>
#include <memory>
#include <model/json.hpp>
#include <model/simulation.hpp>


class imgui_handler {
public:
  virtual ~imgui_handler() {}
  virtual std::string title() const = 0;
  void pre_display(const model::Simulation* sim) { if (enabled_) do_pre_display(sim); }
  void display(const model::Simulation* sim) { if (enabled_) do_display(sim); }
  
  bool enable(bool val) { enabled_ = val; return enabled_; }
  bool enabled() const { return enabled_; }

protected:
  virtual void do_pre_display(const model::Simulation* sim) {};
  virtual void do_display(const model::Simulation* sim) = 0;
  bool enabled_ = false;
};


std::unique_ptr<imgui_handler> create_imgui_handler(class AppWin* appwin, const json& jh);

