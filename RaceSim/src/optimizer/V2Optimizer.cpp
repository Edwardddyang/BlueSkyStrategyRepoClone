#include <pthread.h>

#include <string>
#include <memory>
#include <utility>
#include <vector>
#include <filesystem>
#include <iostream>
#include <random>

#include "spdlog/spdlog.h"
#include "opt/V2Optimizer.hpp"
#include "route/Route.hpp"
#include "config/Config.hpp"
#include "utils/Defines.hpp"

RacePlan V2Optimizer::optimize() {
  /* Initialize cornering bounds */
  route->init_cornering_bounds(Config::get_instance()->get_corners_path(),
                               kph2mps(Config::get_instance()->get_max_speed()));
  // Create a random device to seed the random number generator
  std::random_device rd;

  // Use the Mersenne Twister engine to generate random numbers
  std::mt19937 gen(rd());

  // Define the range for the random numbers
  std::uniform_int_distribution<unsigned int> dis(0, std::numeric_limits<unsigned int>::max());

  /* Create race plan */
  bool race_plan_is_valid = false;
  RacePlan race_plan;
  int idx = 0;
  while (!race_plan_is_valid) {
    unsigned int idx_seed = dis(gen);
    unsigned int speed_seed = dis(gen);
    race_plan = route->segment_route_acceleration(idx_seed, speed_seed);
    race_plan.print_plan();
    simulator->run_sim(route, &race_plan);
    race_plan_is_valid = race_plan.is_viable();
    idx++;
    spdlog::info("Tried {} race plans", idx);

  bool save_csv = Config::get_instance()->get_save_csv();
  std::filesystem::path results_folder;
  if (save_csv) {
    const std::string strat_root = Config::get_instance()->get_strat_root();
    results_folder = std::filesystem::path(strat_root) / "Acceleration_Results";
    std::filesystem::create_directory(results_folder);
    simulator->write_result((results_folder / ("Acceleration.csv")).string());
  }

    if (!race_plan_is_valid) {
      std::cout << "Reason for inviability: " << race_plan.get_inviability_reason() << std::endl;
    }
    exit(0);
  }



  return race_plan;
}

V2Optimizer::V2Optimizer(std::shared_ptr<Simulator> simulator, std::shared_ptr<Route> route)
    : Optimizer(simulator, route) {}
