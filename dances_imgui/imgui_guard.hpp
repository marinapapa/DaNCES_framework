#pragma once

#include <map>
#include <string>
#include <filesystem>
#include <stdio.h>
#include <memory>
#include <libs/exe_path.hpp>
#include <glsl/debug.h>
#include <glm/glm.hpp>
#include "imgui_unit.h"
#include "json.hpp"


struct font_descr 
{
  std::filesystem::path ttf;  // path of the *.ttf file
  float pt;                   // font size in points [dpi/72]
  float content_scale;
  std::string name;           // name for retrieving the font via AppWin::get_font
};


class imgui_guard 
{
  imgui_guard();
public:
  static void init();
  static imgui_guard* gImgg();

public:
  ~imgui_guard();

  GLFWwindow* window() noexcept { return window_; }
  ImFont* get_font(const std::string& name);
  double dpi() const;
  double content_scale() const;
  bool headless() const noexcept { return headless_; }
  
  // [left, top, right, bottom] in screen-coordinates 
  // including decoration
  glm::ivec4 win_size() const noexcept;

  // returns mouse wheel offset
  double mouse_wheel() noexcept;

private:
  void add_font(font_descr font);
  GLFWwindow* window_ = nullptr;
  std::map<std::string, std::pair<ImFont*, font_descr>> fonts_;
  bool headless_ = true;
};

