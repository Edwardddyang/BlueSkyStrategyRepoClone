#include <stdlib.h>
#include <stdio.h>

#include <cstring>
#include <string>
#include <filesystem>
#include <unordered_map>
#include <memory>

#include "config/Config.hpp"
#include "spdlog/spdlog.h"
#include "yaml-cpp/yaml.h"

Config& Config::get_instance() {
  static Config instance;
  return instance;
}

#define PARAM(name, type, default_value)\
  name = std::move(ConfigParam<type>(#name, default_value, key_values, STRAT_ROOT));

void Config::load(std::filesystem::path file_path, char* strat_root) {
  RUNTIME_EXCEPTION(strat_root != nullptr, "No STRAT_ROOT variable detected.");

  /* Parse config file */
  config_file_path = (std::filesystem::path(strat_root) / file_path);
  STRAT_ROOT = strat_root;
  try {
    YAML::Node config = YAML::LoadFile(config_file_path.string());
    get_config_leaf_nodes(config);
  } catch (const YAML::ParserException) {
    RUNTIME_EXCEPTION(false, "Config file {} could not be parsed. Check yaml for correct syntax. Exiting",
                      config_file_path.string());
  } catch (const YAML::BadFile) {
    RUNTIME_EXCEPTION(false, "Config file {} could not be loaded. Check that file exists. Exiting",
                      config_file_path.string());
  } catch (const std::exception) {
    RUNTIME_EXCEPTION(false, "Config file {} could not be loaded. Exiting", config_file_path.string());
  }

  /* Initialize all parameters */
  CONFIG_PARAMETERS

  initialized = true;
}

void Config::get_config_leaf_nodes(YAML::Node node, int current_depth) {
  RUNTIME_EXCEPTION(current_depth <= MAX_RECURSION_DEPTH,
    "Maximum recursion depth reached when parsing configuration file."
    "If this is expected, increase the MAX_RECUSION_DEPTH variable.");

  if (node.IsMap()) {
    /* Iterate through all key/value pairs of the map */
    for (auto it = node.begin(); it != node.end(); ++it) {
      if (it->second.IsScalar()) {
        key_values[it->first.as<std::string>()] = it->second;
      } else {
        get_config_leaf_nodes(it->second, current_depth + 1);
      }
    }
  }
}
