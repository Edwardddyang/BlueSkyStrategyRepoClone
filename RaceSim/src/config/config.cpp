#include <config.h>
#include <string.h>
#include <utilities.h>
#include <cstring>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <Luts.h>
#include <custom_time.h>
#include <unordered_map>
#include "spdlog/spdlog.h"
#include "yaml-cpp/yaml.h"
#include <filesystem>

/* Static member declaration */
Config* Config::instance_ptr = nullptr;
YAML::Node Config::config;
std::unordered_map<std::string, YAML::Node> Config::key_values;
std::string Config::config_file_path = "";
std::string Config::STRAT_ROOT = "";
bool Config::initialized = false;

Config* Config::get_instance() {
    if (instance_ptr == NULL) {
		if (config_file_path == "") {
			spdlog::critical("No config file path set. Add Config::initialize(file_path, strat_root) before get_instance(). Exiting");
			exit(0);
		}

		if (STRAT_ROOT == "") {
			spdlog::critical("No STRAT_ROOT variable detected. Add Config::initialize(file_path, strat_root) before get_instance(). Exiting");
			exit(0);
		}
		/* Parse config file */
		std::string full_config_file_path = (std::filesystem::path(STRAT_ROOT) / config_file_path).string();
		try {
			config = YAML::LoadFile(full_config_file_path);
			get_config_leaf_nodes(config);
		} catch (const YAML::ParserException& e) {
			spdlog::critical("Config file could not be parsed. Check yaml for correct syntax. Exiting");
			exit(0);
		} catch (const YAML::BadFile& e) {
			spdlog::critical("Config file could not be loaded. Check path relative to executable. Exiting");
			exit(0);
		} catch (const std::exception& e) {
			spdlog::critical("Config file could not be loaded. Exiting");
			exit(0);
		}
        instance_ptr = new Config();
    } 
    return instance_ptr;

}

void Config::initialize(std::string file_path, std::string strat_root_path) {
	if (initialized || config_file_path != "" || STRAT_ROOT != "" || instance_ptr != nullptr) {
		spdlog::error("Can only initialize config class once. Exiting.");
		exit(0);
	} else{
		config_file_path = file_path;
		STRAT_ROOT = strat_root_path;
		initialized = true;
	}
	return;
}
void Config::get_config_leaf_nodes(YAML::Node node) {
	if (node.IsMap()) {
		/* Iterate through all key/value pairs of the map */
        for (auto it = node.begin(); it != node.end(); ++it) {
			if (it->second.IsScalar()) {
				key_values[it->first.as<std::string>()] = it->second;
			} else {
				get_config_leaf_nodes(it->second);
			}
        }
    }
}

Config::~Config() {
	// if (current_date_time != nullptr) {
	// 	delete current_date_time;
	// }

	// if (end_date_time != nullptr) {
	// 	delete end_date_time;
	// }
}
