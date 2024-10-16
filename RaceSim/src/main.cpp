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
#include "route/Route.hpp"
#include "opt/OptimizerFactory.hpp"
#include "utils/Defines.hpp"

#include "libIntegrate/Integrate.hpp"

int main(int argc, char* argv[]) {
  spdlog::set_level(spdlog::level::debug);
  RUNTIME_EXCEPTION(argc == 2, "Exactly one argument is required for the config file path");

  char* strat_root = std::getenv("STRAT_ROOT");
  RUNTIME_EXCEPTION(strat_root != nullptr, "No STRAT_ROOT environment variable detected."
                                           "Set it to the full path to gen12_strategy/RaceSim.");

  std::filesystem::path config_file_path(argv[1]);
  Config::initialize(config_file_path, strat_root);

  /* Create a model of the car */
  std::unique_ptr<Car> car = CarFactory::get_car(Config::get_instance()->get_model());

  /* Create route */
  std::unique_ptr<Route> route = std::make_unique<Route>();

  /* Create simulator */
  std::unique_ptr<Simulator> sim = std::make_unique<Simulator>(std::move(car));

  /* Create optimizer */
  std::unique_ptr<Optimizer> opt = OptimizerFactory::get_optimizer(Config::get_instance()->get_optimizer(),
                                                                    std::move(route), std::move(sim));

  /* Run optimizer */
  std::vector<double> result_speed_profile_km = opt->optimize();

  spdlog::info("Viable Speed Profile: {}", result_speed_profile_km[0]);

  return 0;
}
