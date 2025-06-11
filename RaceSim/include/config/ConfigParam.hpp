/* Represents a config parameter */

#pragma once

#include <stdlib.h>
#include <yaml-cpp/yaml.h>

#include <string>
#include <unordered_map>
#include <memory>
#include <filesystem>
#include <utility>

#include "utils/Utilities.hpp"
#include "utils/CustomTime.hpp"

template<typename T, typename R>
class Config_Param {
 private:
  T value;
  std::string name;

 public:
  R get_value() {
    if constexpr (std::is_same<std::unique_ptr<Time>, T>::value && std::is_same<Time, R>::value) {
      return *(value.get());
    } else {
      return value;
    }
  }
  Config_Param<T, R>(std::string param_name, T default_value, std::unordered_map<std::string, YAML::Node> key_values,
                     const char* STRAT_ROOT) {
    name = param_name;
    if (key_values.find(name) != key_values.end()) {
      if constexpr (std::is_same<Coord, T>::value) {
        value = create_coord(key_values[name].template as<std::string>());
      } else if constexpr (std::is_same<std::unordered_set<size_t>, T>::value) {
        value = convert_string_to_int_set(key_values[name].template as<std::string>());
      } else if constexpr (std::is_same<std::unique_ptr<Time>, T>::value) {
        value = std::make_unique<Time>(key_values[name].template as<std::string>(),
                                        key_values["utc_adjustment"].template as<double>());
      } else if constexpr (std::is_same<std::filesystem::path, T>::value) {
        value = std::filesystem::path(STRAT_ROOT) / std::filesystem::path(key_values[name].template as<std::string>());
      } else {
        value = key_values[name].template as<T>();
      }
    } else {
      if constexpr(std::is_same<std::unique_ptr<Time>, T>::value) {
        value = std::move(default_value);
      } else if constexpr (std::is_same<std::filesystem::path, T>::value) {
        value = std::filesystem::path(STRAT_ROOT) / default_value;
      } else {
        value = default_value;
      }
    }
  }
  Config_Param<T, R>() {}
};
