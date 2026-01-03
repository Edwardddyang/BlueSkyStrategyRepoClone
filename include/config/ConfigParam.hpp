/* Represents a config parameter */

#pragma once

#include <stdlib.h>
#include <yaml-cpp/yaml.h>

#include <concepts>
#include <type_traits>
#include <string>
#include <unordered_map>
#include <memory>
#include <filesystem>
#include <utility>

#include "SimUtils/Types.hpp"

// For parameters of type unique pointer, return const reference to
// dereferenced object
template <typename T>
struct ConfigReturnType {
  using type = T;
};

template <typename T>
struct ConfigReturnType<std::unique_ptr<T>> {
  using type = const T&;
};

template <typename T>
using ParamReturnType = typename ConfigReturnType<T>::type;

template <typename T>
concept NotRawPointer = !std::is_pointer_v<T>;

template<NotRawPointer T>
class ConfigParam {
 private:
  T value;          // Actual data value
  std::string name; // Name of the parameter

 public:
  ParamReturnType<T> get_value() {
    if constexpr (std::is_same_v<ParamReturnType<T>, T>) {
      return value;
    } else {  // Path for unique pointer - return a const reference
      return *value;
    }
  }
  ConfigParam<T>(std::string param_name, T default_value,
                 const std::unordered_map<std::string, YAML::Node>& key_values,
                 const char* STRAT_ROOT) : name(std::move(param_name)) {
    if (key_values.find(name) != key_values.end()) {
      if constexpr (std::is_same<util::type::Coord, T>::value) {
        value = util::create_coord(key_values.at(name).template as<std::string>());
      } else if constexpr (std::is_same<std::unordered_set<int>, T>::value) {
        value = util::convert_string_to_int_set(key_values.at(name).template as<std::string>());
      } else if constexpr (std::is_same<util::type::Time, T>::value) {
        value = util::type::Time(key_values.at(name).template as<std::string>());
      } else if constexpr (std::is_same<std::filesystem::path, T>::value) {
        value = std::filesystem::path(STRAT_ROOT) / std::filesystem::path(key_values.at(name).template as<std::string>());
      } else {
        value = key_values.at(name).template as<T>();
      }
    } else {
      if constexpr(std::is_same<std::unique_ptr<util::type::Time>, T>::value) {
        value = std::move(default_value);
      } else if constexpr (std::is_same<std::filesystem::path, T>::value) {
        value = std::filesystem::path(STRAT_ROOT) / default_value;
      } else {
        value = default_value;
      }
    }
  }
ConfigParam(ConfigParam&&) noexcept = default;
    ConfigParam& operator=(ConfigParam&&) noexcept = default;
  ConfigParam<T>() = default;
  ~ConfigParam<T>() = default;
};
