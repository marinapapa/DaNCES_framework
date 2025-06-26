#include <iostream>
#include <sstream>
#include <string_view>
#include <model/json.hpp>
#include <exe_path.hpp>


std::optional<json> from_path(const std::filesystem::path& path) {
  try {
    auto is = std::ifstream(path);
    return json::parse(is, nullptr, true, true);    
  }
  catch (...) {
    return {};
  }
}


void save_json(const json& J, const std::filesystem::path& path)
{
  std::ofstream os(path);
  os << J;
}
