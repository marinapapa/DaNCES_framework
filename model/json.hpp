#ifndef JSON_HPP_IMPORT_INCLUDED
#define JSON_HPP_IMPORT_INCLUDED

#include <string>
#include <fstream>
#include <filesystem>
#include <vector>
#include <optional>
#include <glm/glm.hpp>
#include <nlohmann/json.hpp>  // nlohmann.github.io/json/


using json = nlohmann::json;

namespace glm {

  inline void from_json(const json& j, glm::vec4& v) {
    j.at(0).get_to(v.x);
    j.at(1).get_to(v.y);
    j.at(2).get_to(v.z);
    j.at(3).get_to(v.w);
  }

  inline void from_json(const json& j, glm::vec3& v) {
    j.at(0).get_to(v.x);
    j.at(1).get_to(v.y);
    j.at(2).get_to(v.z);
  }

  inline void from_json(const json& j, glm::vec2& v) {
    j.at(0).get_to(v.x);
    j.at(1).get_to(v.y);
  }

}


std::optional<json> from_path(const std::filesystem::path& path);
void save_json(const json& J, const std::filesystem::path& path);


template <typename T, typename K>
inline std::optional<T> optional_json(const json& J, const K& key) {
  return (J.count(key) != 0) ? std::optional<T>(J.at(key).template get<T>()) : std::nullopt;
}

#endif
