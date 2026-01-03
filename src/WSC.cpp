/* Optimize for WSC */

#include <stdio.h>
#include <stdlib.h>

#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "config/ConfigParser.hpp"
#include "model/Car.hpp"
#include "route/Route.hpp"
#include "route/RacePlan.hpp"
#include "sim/FSGPSimulator.hpp"
#include "spdlog/spdlog.h"
#include "SimUtils/Defines.hpp"
#include "SimUtils/Luts.hpp"

int main(int argc, char *argv[]) {
  spdlog::set_level(spdlog::level::debug);
  RUNTIME_EXCEPTION(
      argc == 2, "Exactly one argument is required for the config file path");

  char *strat_root = std::getenv("STRAT_ROOT");
  RUNTIME_EXCEPTION(strat_root != nullptr,
                    "No STRAT_ROOT environment variable detected."
                    "Set it to the full path to gen12_strategy/RaceSim.");

  std::filesystem::path config_file_path(argv[1]);
  Config::get_instance().load(config_file_path, strat_root);

  // Create car, simulator, route objects
  Car car;
  WSCSimulator simulator;
  WSCRoute route(Config::get_instance().get_base_route_path());

  return 0;
}
