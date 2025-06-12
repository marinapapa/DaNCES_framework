#include <iostream>
#include <sstream>
#include <string_view>
#include <model/json.hpp>
#include <exe_path.hpp>


namespace {

  std::stringstream uncomment_ss(std::istream& is)
  {
    std::stringstream ss;
    std::string line;
    while (!!is) {
      line.clear();
      std::getline(is, line);
      for (auto chr : line) {
        if (chr == '#') break;
        ss << chr;
      }
    }
    return ss;
  }


  json uncomment(std::istream&& is)
  {
    std::stringstream ss = uncomment_ss(is);
    auto j = json{};
    ss >> j;
    return j;
  }

}


json uncomment_json(const std::filesystem::path& path)
{
  return uncomment(std::ifstream(path));
}


json uncomment_json(const std::string& jstr)
{
  return uncomment(std::stringstream(jstr));
}


json compose_json(const std::filesystem::path& project_dir, const std::filesystem::path& composed_config)
{
    std::vector<std::filesystem::path> paths;

    if (composed_config == "") {
      auto ic = std::ifstream(project_dir / "config.json");
      auto jc = json{};
      ic >> jc;
      paths.assign(1, project_dir / "config.json");
      paths.emplace_back(std::filesystem::path(std::string(jc["Simulation"]["species_config"]["prey"])));
      paths.emplace_back(std::filesystem::path(std::string(jc["Simulation"]["species_config"]["predator"])));
        
      if (std::filesystem::exists(exe_path::get() / "imgui.json")) {
        paths.emplace_back(exe_path::get() / "imgui.json");
      }
    }
    else {
        paths.assign(1, composed_config);
    }

  std::string str("{");
  for (size_t i = 0; i < paths.size(); ++i) {
    std::ifstream is(paths[i]);
    std::string s(uncomment_ss(is).str());
    s.erase(s.find('{'), 1);
    s.erase(s.rfind('}'));
    if (i != paths.size() - 1) {
      s += ',';
    }
    str += s;
  }
  str += "}";
  json j{};
  std::stringstream ss;
  ss << str;
  ss >> j;
  return j;
}

void save_json(const json& J, const std::filesystem::path& json_file)
{
  std::ofstream os(json_file);
  os << J;
}


