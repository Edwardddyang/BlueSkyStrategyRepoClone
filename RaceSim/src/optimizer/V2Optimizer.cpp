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
    // We create a separate RacePlan object for each day
    const Time curr_time = Config::get_instance()->get_current_date_time();
    const Time day_one_start_time = Config::get_instance()->get_day_one_start_time();
    const Time day_one_end_time = Config::get_instance()->get_day_one_end_time();
    const Time day_end_time = Config::get_instance()->get_day_end_time();  // Second and third day
    const bool is_first_day = curr_time >= day_one_start_time && curr_time < day_one_end_time;
    const Time race_plan_end_time = is_first_day ? day_one_end_time : day_end_time;
    race_plan = route->segment_route_corners(Config::get_instance()->get_max_num_loops(),
                                             Config::get_instance()->get_car_mass(),
                                             kph2mps(Config::get_instance()->get_max_route_speed()),
                                             kw2watts(Config::get_instance()->get_max_motor_power()),
                                             Config::get_instance()->get_max_acceleration(),
                                             Config::get_instance()->get_max_deceleration(),
                                             &curr_time, &race_plan_end_time,
                                             Config::get_instance()->get_preferred_acceleration(),
                                             Config::get_instance()->get_preferred_deceleration(),
                                             speed_seed, loop_seed, aggressive_seed, idx_seed, acceleration_seed,
                                             Config::get_instance()->get_corner_speed_min(),
                                             Config::get_instance()->get_corner_speed_max(),
                                             Config::get_instance()->get_aggressive_straight_threshold(),
                                             Config::get_instance()->get_num_repetitions(),
                                             Config::get_instance()->get_acceleration_power_budget(),
                                             1000, Config::get_instance()->get_log_segmenting());
    race_plan.print_plan();
    auto start = std::chrono::high_resolution_clock::now();
    simulator->run_sim(route, &race_plan, result_lut);
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Function execution time: " << duration.count() << " microseconds" << std::endl;
    race_plan_is_valid = race_plan.is_viable();
    const std::string strat_root = Config::get_instance()->get_strat_root();
    std::filesystem::path results_folder = std::filesystem::path(strat_root) / "Acceleration_Results";
    std::filesystem::create_directory(results_folder);
    result_lut->write_logs((results_folder / ("Acceleration.csv")).string());
    if (!race_plan_is_valid) {
      spdlog::info("Race plan was not valid. Reason: {}", race_plan.get_inviability_reason());
    } else {
      spdlog::info("Race plan is valid somehow");
    }
    exit(0);
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
