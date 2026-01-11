#include "config/ConfigParser.hpp"

#include <yaml-cpp/yaml.h>  // IWYU pragma: keep

#include <exception>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <unordered_map>

#include "SimUtils/Defines.hpp"
#include "config/ConfigParam.hpp"

#define PARAM(name, type, default_value) \
  name = ConfigParam<type>(#name, default_value, key_values, STRAT_ROOT);

ConfigParser::ConfigParser(const std::filesystem::path& path) {
  // Load the STRAT_ROOT variable
  char* strat_root = std::getenv("STRAT_ROOT");  // NOLINT temp fix
  RUNTIME_EXCEPTION(strat_root != nullptr,
                    "STRAT_ROOT environment variable not set");

  STRAT_ROOT = strat_root;

  config_file_path = (std::filesystem::path(STRAT_ROOT)) / path;

  try {
    const YAML::Node config =                       // NOLINT
        YAML::LoadFile(config_file_path.string());  // NOLINT
    get_config_leaf_nodes(config);                  // Fill kv pairs
  } catch (const YAML::ParserException&) {          // NOLINT
    RUNTIME_EXCEPTION(false,
                      "Config file {} could not be parsed. Check yaml for "
                      "correct syntax. Exiting",
                      config_file_path.string());
  } catch (const YAML::BadFile&) {  // NOLINT
    RUNTIME_EXCEPTION(
        false,
        "Config file {} could not be loaded. Check that file exists. Exiting",
        config_file_path.string());
  } catch (const std::exception&) {
    RUNTIME_EXCEPTION(false, "Config file {} could not be loaded. Exiting",
                      config_file_path.string());
  }

  /* Initialize all parameters */
  CONFIG_PARAMETERS
}

void ConfigParser::get_config_leaf_nodes(YAML::Node node, int current_depth) {
  RUNTIME_EXCEPTION(
      current_depth <= MAX_RECURSION_DEPTH,
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
