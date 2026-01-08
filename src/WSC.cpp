/* Optimize for WSC */

#include <filesystem>

#include "spdlog/spdlog.h"
#include "config/ConfigParser.hpp"
#include "model/Car.hpp"
#include "route/Route.hpp"
#include "route/RacePlan.hpp"
#include "sim/WSCSimulator.hpp"
#include "opt/WSCOptimizer.hpp"
#include "SimUtils/Defines.hpp"

int main(int argc, char *argv[]) {
  spdlog::set_level(spdlog::level::debug);
  RUNTIME_EXCEPTION(argc == 2, "Exactly one argument is required for the config file path");
  std::filesystem::path config_file_path(argv[1]);
  ConfigParser config(config_file_path);

  // Create objects from config
  CarParams car_params = get_car_parameters(&config);
  WSCSimulatorParams sim_params = get_wsc_simulator_params(&config);
  WSCOptimizerParams opt_params = get_wsc_optimizer_params(&config);
  WSCRouteParams route_params = get_wsc_route_params(&config);

  Car car(car_params);
  WSCSimulator sim(sim_params, car);
  WSCRoute route(route_params, config.get_base_route_path());
  WSCOptimizer opt(opt_params, sim, route);
  WSCRacePlan optimal_race_plan = opt.optimize();

  if (!optimal_race_plan.is_empty()) {
    spdlog::info("Optimal race plan found");
    optimal_race_plan.print_plan();
  } else {
    spdlog::info("No viable race plan");
  }

  return 0;
}
