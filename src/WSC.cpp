/* Optimize for WSC */

#include <iostream>
#include <exception>
#include <filesystem>

#include "SimUtils/Defines.hpp"
#include "config/ConfigParser.hpp"
#include "model/Car.hpp"
#include "opt/WSCOptimizer.hpp"
#include "route/RacePlan.hpp"
#include "route/Route.hpp"
#include "sim/WSCSimulator.hpp"
#include "spdlog/common.h"
#include "spdlog/spdlog.h"

int main(int argc, char *argv[]) {
  try {
    spdlog::set_level(spdlog::level::debug);
    RUNTIME_EXCEPTION(
        argc == 2, "Exactly one argument is required for the config file path");
    const std::filesystem::path config_file_path(argv[1]);
    ConfigParser config(config_file_path);

    // Create objects from config
    const CarParams car_params = get_car_parameters(&config);
    const WSCSimulatorParams sim_params = get_wsc_simulator_params(&config);
    const WSCOptimizerParams opt_params = get_wsc_optimizer_params(&config);
    const WSCRouteParams route_params = get_wsc_route_params(&config);

    const Car car(car_params);
    const WSCSimulator sim(sim_params, car);
    const WSCRoute route(route_params, config.get_base_route_path());
    WSCOptimizer opt(opt_params, sim, route);
    const WSCRacePlan optimal_race_plan = opt.optimize();

    if (!optimal_race_plan.is_empty()) {
      spdlog::info("Optimal race plan found");
      optimal_race_plan.print_plan();
    } else {
      spdlog::info("No viable race plan");
    }
  } catch (const std::exception &) {
    std::cout << e.what() << std::endl;
    return 1;
  }
  return 0;
}
