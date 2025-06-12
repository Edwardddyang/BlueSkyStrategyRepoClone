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
#include "sim/TelemetrySimulator.hpp"
#include "utils/Luts.hpp"


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

  const SIMULATORS sim_type = SimulatorFactory::get_sim_type(Config::get_instance()->get_simulator());
  std::shared_ptr<Route> route;
  if (sim_type != SIMULATORS::TELEMETRY) {
    /* Optimization flow */
    /* Create route */
    route = std::make_shared<Route>(Config::get_instance()->get_base_route_path(),
                                    false,
                                    Config::get_instance()->get_init_control_stops(),
                                    Config::get_instance()->get_corners_path(),
                                    Config::get_instance()->get_precomputed_distances_path(),
                                    Config::get_instance()->get_calculate_distances());

    /* Create simulator */
    std::shared_ptr<Simulator> sim = SimulatorFactory::get_simulator(Config::get_instance()->get_simulator(), car);

    /* Create optimizer */
    std::shared_ptr<Optimizer> opt = OptimizerFactory::get_optimizer(Config::get_instance()->get_optimizer(),
                                                                    route, sim);

    /* Run optimizer */
    RacePlan viable_race_plan = opt->optimize();
  } else {
    /* Pure simulation flow using telemetry data*/
    // Create car and route
    route = std::make_shared<Route>(Config::get_instance()->get_base_route_path(), true);

    std::shared_ptr<TelemetrySimulator> telem_sim = std::dynamic_pointer_cast<TelemetrySimulator>(
      SimulatorFactory::get_simulator(Config::get_instance()->get_simulator(), car)
    );
    RUNTIME_EXCEPTION(telem_sim != nullptr, "Telemetry simulator instantiation failed");

    spdlog::info("Starting simulation");
    std::shared_ptr<ResultsLut> result = std::make_shared<ResultsLut>();
    telem_sim->run_sim(route, result);
    spdlog::info("Finished simulation");

    // Save the logs
    bool save_csv = Config::get_instance()->get_save_csv();
    std::filesystem::path results_folder;
    if (save_csv) {
      const std::string strat_root = Config::get_instance()->get_strat_root();
      results_folder = std::filesystem::path(strat_root) / "TelemetrySimResults";
      std::filesystem::create_directory(results_folder);
      result->write_logs((results_folder / "sim.csv").string());
    }
  }

  /* Print the optimal speed profile */

  return 0;
}
