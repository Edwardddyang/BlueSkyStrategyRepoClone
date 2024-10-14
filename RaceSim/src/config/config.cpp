#include <cstring>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <filesystem>

#include "Utilities.hpp"
#include "Config.hpp"
#include "Luts.hpp"
#include "CustomTime.hpp"
#include "spdlog/spdlog.h"
#include "yaml-cpp/yaml.h"

/* Static member declaration */
std::unique_ptr<Config> Config::instance_ptr = nullptr;
YAML::Node Config::config;
std::unordered_map<std::string, YAML::Node> Config::key_values;
std::string Config::config_file_path = "";
std::string Config::STRAT_ROOT = "";
bool Config::initialized = false;

Config* Config::get_instance() {
    if (instance_ptr == NULL) {
		RUNTIME_EXCEPTION(config_file_path != "", "No config file path set. Add Config::initialize(file_path, strat_root) before get_instance(). Exiting");
		RUNTIME_EXCEPTION(STRAT_ROOT != "", "No STRAT_ROOT variable detected. Add Config::initialize(file_path, strat_root) before get_instance(). Exiting");

		/* Parse config file */
		std::string full_config_file_path = (std::filesystem::path(STRAT_ROOT) / config_file_path).string();
		try {
			config = YAML::LoadFile(full_config_file_path);
			get_config_leaf_nodes(config);
		} catch (const YAML::ParserException& e) {
			RUNTIME_EXCEPTION(false, "Config file {} could not be parsed. Check yaml for correct syntax. Exiting", full_config_file_path);
		} catch (const YAML::BadFile& e) {
			RUNTIME_EXCEPTION(false, "Config file {} could not be loaded. Check that file exists. Exiting", full_config_file_path);		
		} catch (const std::exception& e) {
			RUNTIME_EXCEPTION(false, "Config file {} could not be loaded. Exiting", full_config_file_path);
		}
        instance_ptr = std::make_unique<Config>(Config());
    } 
    return instance_ptr.get();

}

void Config::initialize(std::string file_path, std::string strat_root_path) {
	RUNTIME_EXCEPTION(!initialized && config_file_path == "" && STRAT_ROOT == "" && !instance_ptr,
					  "Can only initialize Config class once. Exiting.");
	config_file_path = file_path;
	STRAT_ROOT = strat_root_path;
	initialized = true;
	return;
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
