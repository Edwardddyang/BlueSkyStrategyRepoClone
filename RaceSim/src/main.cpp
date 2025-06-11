/* Starting point of a race simulation */

#include <stdlib.h>
#include <stdio.h>

#include <string>
#include <utility>
#include <vector>
#include <filesystem>
#include <cstdlib>
#include <memory>

#include "spdlog/spdlog.h"
#include "sim/Simulator.hpp"
#include "config/Config.hpp"
#include "model/CarFactory.hpp"
#include "sim/SimulatorFactory.hpp"
#include "route/Route.hpp"
#include "opt/OptimizerFactory.hpp"
#include "utils/Defines.hpp"
#include "opt/V2Optimizer.hpp"
#include "sim/FSGPSimulator.hpp"


int main(int argc, char* argv[]) {
  spdlog::set_level(spdlog::level::debug);
  RUNTIME_EXCEPTION(argc == 2, "Exactly one argument is required for the config file path");

  char* strat_root = std::getenv("STRAT_ROOT");
  RUNTIME_EXCEPTION(strat_root != nullptr, "No STRAT_ROOT environment variable detected."
                                           "Set it to the full path to gen12_strategy/RaceSim.");

  std::filesystem::path config_file_path(argv[1]);
  Config::initialize(config_file_path, strat_root);

  /* Create a model of the car */
  std::shared_ptr<Car> car = CarFactory::get_car(Config::get_instance()->get_model());

  /* Create route */
  std::shared_ptr<Route> route = std::make_shared<Route>(Config::get_instance()->get_base_route_path(),
                                                         Config::get_instance()->get_init_control_stops(),
                                                         Config::get_instance()->get_corners_path(),
                                                         Config::get_instance()->get_precomputed_distances_path(),
                                                         Config::get_instance()->get_calculate_distances());

  /* Create simulator */
  std::shared_ptr<Simulator> sim = std::make_shared<FSGPSimulator>(car);
  spdlog::info("Forced creation of FSGP Simulator from main.cpp");
  spdlog::info("Config simulator: '{}'", Config::get_instance()->get_simulator());

  /* Create optimizer */
  std::shared_ptr<Optimizer> opt = OptimizerFactory::get_optimizer(Config::get_instance()->get_optimizer(),
                                                                   route, sim);

  /* Run optimizer based on run_type */
  RacePlan viable_race_plan;
  if (Config::get_instance()->get_run_type() == RunType::SimWithTelem) {
    spdlog::info("Run type: sim_with_telem");
    RUNTIME_EXCEPTION(Config::get_instance()->get_optimizer() == "Acceleration", 
                      "sim_with_telem mode requires optimizer to be of type 'Acceleration'");
                      
    auto v2opt = std::dynamic_pointer_cast<V2Optimizer>(opt);
    RUNTIME_EXCEPTION(v2opt != nullptr, "Failed to cast optimizer to V2Optimizer.");

    const auto& coords = Config::get_instance()->get_telem_coords();
    const auto& times = Config::get_instance()->get_telem_times();

    viable_race_plan = v2opt->optimize_telem();
  } else {
    spdlog::info("Run type: normal");
    viable_race_plan = opt->optimize();
  }

  /* Print the optimal speed profile */

  return 0;
}
