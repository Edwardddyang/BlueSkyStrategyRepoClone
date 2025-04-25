#include <pthread.h>

#include <string>
#include <memory>
#include <chrono>
#include <limits>
#include <utility>
#include <vector>
#include <filesystem>
#include <iostream>
#include <thread>
#include <algorithm>
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

  // Create results folder
  bool save_csv = Config::get_instance()->get_save_csv();
  std::filesystem::path results_folder;
  if (save_csv) {
    const std::string strat_root = Config::get_instance()->get_strat_root();
    results_folder = std::filesystem::path(strat_root) / "Acceleration_Results";
    std::filesystem::create_directory(results_folder);
  }
  const int num_race_plans = Config::get_instance()->get_num_iters();
  spdlog::info("Using {} threads for simulation", num_threads);
  spdlog::info("Simulating {} race plans", num_race_plans);

  ThreadManager thread_manager(num_threads);
  std::vector<std::thread> threads;
  result_luts.clear();
  race_plans.clear();
  race_plan_creation.clear();
  const Time curr_time = Config::get_instance()->get_current_date_time();
  const Time day_one_start_time = Config::get_instance()->get_day_one_start_time();
  const Time day_one_end_time = Config::get_instance()->get_day_one_end_time();
  const Time day_end_time = Config::get_instance()->get_day_end_time();  // Second and third day
  const bool is_first_day = curr_time >= day_one_start_time && curr_time < day_one_end_time;
  const Time race_plan_end_time = is_first_day ? day_one_end_time : day_end_time;

  auto race_plan_creation_start = std::chrono::high_resolution_clock::now();
  for (int i=0; i < num_race_plans; i++) {
    unsigned int idx_seed = dis(gen);
    unsigned int speed_seed = dis(gen);
    unsigned int acceleration_seed = dis(gen);
    unsigned int aggressive_seed = dis(gen);
    unsigned int loop_seed = dis(gen);
    unsigned int skip_seed = dis(gen);
    if (Config::get_instance()->get_fix_seeds()) {
      idx_seed = Config::get_instance()->get_idx_seed();
      speed_seed = Config::get_instance()->get_speed_seed();
      acceleration_seed = Config::get_instance()->get_acceleration_seed();
      aggressive_seed = Config::get_instance()->get_aggressive_seed();
      loop_seed = Config::get_instance()->get_loop_seed();
      skip_seed = Config::get_instance()->get_skip_seed();
    }
    auto start = std::chrono::high_resolution_clock::now();
    RacePlan race_plan = route->segment_route_corners(Config::get_instance()->get_max_num_loops(),
                                                      Config::get_instance()->get_fix_num_loops(),
                                                      Config::get_instance()->get_car_mass(),
                                                      kph2mps(Config::get_instance()->get_max_route_speed()),
                                                      kw2watts(Config::get_instance()->get_max_motor_power()),
                                                      Config::get_instance()->get_max_acceleration(),
                                                      Config::get_instance()->get_max_deceleration(),
                                                      Config::get_instance()->get_min_acceleration(),
                                                      Config::get_instance()->get_average_speed(),
                                                      &curr_time, &race_plan_end_time,
                                                      Config::get_instance()->get_preferred_acceleration(),
                                                      Config::get_instance()->get_preferred_deceleration(),
                                                      speed_seed, loop_seed, aggressive_seed, idx_seed,
                                                      acceleration_seed, skip_seed,
                                                      Config::get_instance()->get_corner_speed_min(),
                                                      Config::get_instance()->get_corner_speed_max(),
                                                      Config::get_instance()->get_aggressive_straight_threshold(),
                                                      Config::get_instance()->get_num_repetitions(),
                                                      Config::get_instance()->get_acceleration_power_budget(),
                                                      1000, Config::get_instance()->get_log_segmenting());
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    race_plan_creation.emplace_back(duration.count() / 1'000'000.0);
    result_luts.emplace_back(std::make_shared<ResultsLut>());
    race_plans.emplace_back(race_plan);
  }
  auto race_plan_creation_end = std::chrono::high_resolution_clock::now();
  auto race_plan_duration = std::chrono::duration_cast<std::chrono::microseconds>(race_plan_creation_end -
                                                                                  race_plan_creation_start);
  spdlog::info("Race plan creation all took {} seconds", race_plan_duration.count() / 1'000'000.0);

  if (num_race_plans > 1) {
    for (int i=0; i < num_race_plans; i++) {
      threads.emplace_back(thread_run_sim, simulator, route, result_luts[i], &race_plans[i], &thread_manager);
    }
    for (int i=0; i < num_race_plans; i++) {
      threads[i].join();
    }
  } else {
    simulator->run_sim(route, &race_plans[0], result_luts[0]);
  }

  // Process results
  double best_average_speed = 0.0;
  RacePlan best_race_plan;
  ResultsLut best_race_result;
  for (int i=0; i < num_race_plans; i++) {
    spdlog::info("Race plan {} took {} seconds to create", i, race_plan_creation[i]);
    if (!race_plans[i].is_viable()) {
      spdlog::info("Race plan {} was not valid. Reason: {}", i, race_plans[i].get_inviability_reason());
    } else {
      const double average_speed = meters2km(race_plans[i].get_accumulated_distance()) /
                                   secs2hours(race_plans[i].get_driving_time());
      spdlog::info("Race plan {} is viable. Average Speed: {} kph", i, average_speed);
      if (best_average_speed < average_speed) {
        best_average_speed = average_speed;
        best_race_plan = race_plans[i];
        best_race_result = *result_luts[i].get();
      }
    }
  }
  best_race_plan.print_plan();
  spdlog::info("Highest Average Speed: {} kph", best_average_speed);
  spdlog::info("Num Loops: {}", best_race_plan.get_num_loops());

  if (save_csv) {
    const std::string strat_root = Config::get_instance()->get_strat_root();
    best_race_result.write_logs((results_folder / ("Acceleration.csv")).string());
  }

  return best_race_plan;
}

V2Optimizer::V2Optimizer(std::shared_ptr<Simulator> simulator, std::shared_ptr<Route> route)
    : Optimizer(simulator, route), num_threads(std::max(1u, static_cast<unsigned int>(
      std::thread::hardware_concurrency() * Config::get_instance()->get_threads()))) {}
