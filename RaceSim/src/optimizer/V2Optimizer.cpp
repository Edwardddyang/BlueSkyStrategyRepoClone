#include <pthread.h>

#include <string>
#include <memory>
#include <chrono>
#include <limits>
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
  // Create a random device to seed the random number generator
  std::random_device rd;

  // Use the Mersenne Twister engine to generate random numbers
  std::mt19937 gen(rd());

  // Define the range for the random numbers
  std::uniform_int_distribution<unsigned int> dis(0, std::numeric_limits<unsigned int>::max());

  /* Create race plan */
  bool race_plan_is_valid = false;
  RacePlan race_plan;
  std::shared_ptr<ResultsLut> result_lut = std::make_shared<ResultsLut>();
  int idx = 0;
  route->init_longest_straight();
  while (!race_plan_is_valid) {
    unsigned int idx_seed = dis(gen);
    unsigned int speed_seed = dis(gen);
    unsigned int acceleration_seed = dis(gen);
    unsigned int loop_seed = dis(gen);
    unsigned int skip_seed = dis(gen);
    race_plan = route->segment_route_acceleration(idx_seed, speed_seed, acceleration_seed,
                                                  skip_seed, loop_seed,
                                                  Config::get_instance()->get_max_num_loops(),
                                                  kph2mps(Config::get_instance()->get_max_car_speed()),
                                                  Config::get_instance()->get_max_deceleration());
    race_plan.print_plan();
    auto start = std::chrono::high_resolution_clock::now();
    simulator->run_sim(route, &race_plan, result_lut);
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Function execution time: " << duration.count() << " microseconds" << std::endl;
    race_plan_is_valid = race_plan.is_viable();
    idx++;
    spdlog::info("Tried {} race plans", idx);
    if (!race_plan_is_valid) {
      std::cout << "Reason for inviability: " << race_plan.get_inviability_reason() << std::endl;
    }
  bool save_csv = Config::get_instance()->get_save_csv();
  std::filesystem::path results_folder;
  if (save_csv) {
    const std::string strat_root = Config::get_instance()->get_strat_root();
    results_folder = std::filesystem::path(strat_root) / "Acceleration_Results";
    std::filesystem::create_directory(results_folder);
    result_lut->write_logs((results_folder / ("Acceleration.csv")).string());
  }
  }

  bool save_csv = Config::get_instance()->get_save_csv();
  std::filesystem::path results_folder;
  if (save_csv) {
    const std::string strat_root = Config::get_instance()->get_strat_root();
    results_folder = std::filesystem::path(strat_root) / "Acceleration_Results";
    std::filesystem::create_directory(results_folder);
    result_lut->write_logs((results_folder / ("Acceleration.csv")).string());
  }

  return race_plan;
}

V2Optimizer::V2Optimizer(std::shared_ptr<Simulator> simulator, std::shared_ptr<Route> route)
    : Optimizer(simulator, route) {}
