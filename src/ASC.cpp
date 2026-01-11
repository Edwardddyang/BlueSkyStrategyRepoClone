/* Optimize for ASC */

#include <filesystem>

#include "SimUtils/Defines.hpp"
#include "config/ConfigParser.hpp"
#include "model/Car.hpp"
#include "opt/ASCOptimizer.hpp"
#include "route/RacePlan.hpp"
#include "route/Route.hpp"
#include "sim/ASCSimulator.hpp"
#include "spdlog/spdlog.h"

int main(int argc, char *argv[]) {
  spdlog::set_level(spdlog::level::debug);
  RUNTIME_EXCEPTION(
      argc == 2, "Exactly one argument is required for the config file path");
  std::filesystem::path config_file_path(argv[1]);
  ConfigParser config(config_file_path);

  // Create objects from config
  CarParams car_params = get_car_parameters(&config);
  ASCSimulatorParams sim_params = get_asc_simulator_params(&config);
  ASCOptimizerParams opt_params = get_asc_optimizer_params(&config);
  ASCRouteParams route_params = get_asc_route_params(&config);

  Car car(car_params);
  ASCSimulator sim(sim_params, car);
  ASCRoute route(route_params, config.get_base_route_path());
  ASCOptimizer opt(opt_params, sim, route);
  ASCRacePlan optimal_race_plan = opt.optimize();

  if (!optimal_race_plan.is_empty()) {
    spdlog::info("Optimal race plan found");
    optimal_race_plan.print_plan();
  } else {
    spdlog::info("No viable race plan");
  }

  return 0;
}
