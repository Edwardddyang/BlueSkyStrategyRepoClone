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
#include <unordered_set>

#include "spdlog/spdlog.h"
#include "opt/V2Optimizer.hpp"
#include "route/Route.hpp"
#include "config/Config.hpp"
#include "utils/Defines.hpp"
#include "utils/Units.hpp"

// Sort function
bool comp_race_plan(const RacePlan& a, const RacePlan& b) {
  return a.get_score() > b.get_score();
}

RacePlan V2Optimizer::optimize() {
  // Create results folder
  bool save_csv = Config::get_instance()->get_save_csv();
  std::filesystem::path results_folder;
  if (save_csv) {
    const std::string strat_root = Config::get_instance()->get_strat_root();
    results_folder = std::filesystem::path(strat_root) / "Acceleration_Results";
    std::filesystem::create_directory(results_folder);
  }

  spdlog::info("Using {} threads for simulation", num_threads);
  init_params();

  auto race_plan_creation_start = std::chrono::high_resolution_clock::now();

  // Create initial population
  create_initial_population();

  for (size_t i=0; i < num_generations; i++) {
    // Simulate race plans
    simulate_population();

    // Evaluate race plans and gather fitness scores
    evaluate_population();

    // Crossover fittest parents
  }

  // Process results
  RacePlan best_race_plan = population[0];
  double best_average_speed = mps2kph(best_race_plan.get_average_speed());
  ResultsLut best_race_result;

  return best_race_plan;
}

void V2Optimizer::crossover_population() {
  const int num_parents = initial_population_size * (parents_percentage / 100.0);
  std::sort(population.begin(), population.end(), comp_race_plan);
  std::vector<RacePlan> new_generation(initial_population_size);

  // Note that std::uniform_int_distribution is inclusive on both sides
  std::uniform_int_distribution<size_t> parent_indices_dist(0, num_parents - 1);
  std::unordered_set<size_t> crossed_over_parent_indices;
  std::random_device rd;
  std::mt19937 gen(rd());

  int num_crossovers = 0;
  while (num_crossovers < num_parents / 2) {
    // Select two parents to crossover
    const size_t parent_a_idx = parent_indices_dist(gen);
    const size_t parent_b_idx = parent_indices_dist(gen);
    if (parent_a_idx == parent_b_idx) {
      continue;
    }
    num_crossovers = num_crossovers + 1;
    RacePlan parent_a = population[parent_a_idx];
    RacePlan parent_b = population[parent_b_idx];

    RacePlan child_plan = crossover_parents(parent_a, parent_b);
  }
  // TODO(KEVIN): Mutate the remaining plans
  // while (idx < num_pop_remaining) {
  // mutate_parent(parent)
  // }
}

RacePlan V2Optimizer::crossover_parents(RacePlan parent_a, RacePlan parent_b) {
  RUNTIME_EXCEPTION(!parent_a.is_empty() && !parent_b.is_empty(), "Parent race plans cannot be empty");
  // Randomly select loop from parent a to combine
  const size_t num_loops_a = parent_a.get_num_loops();
  const size_t num_loops_b = parent_b.get_num_loops();

  return RacePlan();
  // Find common ending corners
}

void V2Optimizer::init_params() {
  // Create genetic optimizer
  RUNTIME_EXCEPTION(initial_population_size > 0, "Initial population size must be greater than 0");
  RUNTIME_EXCEPTION(num_generations > 0, "Number of generations must be greater than 0");
  RUNTIME_EXCEPTION(survival_percentage > 0, "Survival percentage must be greater than 0");
  RUNTIME_EXCEPTION((parents_percentage / 100.0) * initial_population_size > 1, "Parents percent must select "
                    "at least two parents to breed each generation");
  spdlog::info("----Genetic Optimizer Parameters:-----");
  spdlog::info("Initial Population Size: {}", initial_population_size);
  spdlog::info("Number of Generations: {}", num_generations);
  spdlog::info("Survival Percentage: {}", survival_percentage);

  thread_manager.set_max_threads(num_threads);
  result_luts.clear();
  threads.clear();
  race_plan_creation.clear();
  population.clear();

  population.resize(initial_population_size);
  result_luts.resize(initial_population_size);
  threads.resize(initial_population_size);
  race_plan_creation.resize(initial_population_size);
}

void V2Optimizer::create_initial_population() {
  RUNTIME_EXCEPTION(initial_population_size > 1, "Initial population size must be greater than 1");
  RUNTIME_EXCEPTION(population.size() == initial_population_size, "Population vector not resized");
  RUNTIME_EXCEPTION(result_luts.size() == initial_population_size, "Results lut vector not resized");
  RUNTIME_EXCEPTION(race_plan_creation.size() == initial_population_size, "Race plan creation vector not resized");
  // Create a random device to seed the random number generator
  std::random_device rd;

  // Use the Mersenne Twister engine to generate random numbers
  std::mt19937 gen(rd());

  // Define the range for the random numbers
  std::uniform_int_distribution<unsigned int> dis(0, std::numeric_limits<unsigned int>::max());
  for (int i=0; i < initial_population_size; i++) {
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

    const Time curr_time = Config::get_instance()->get_current_date_time();
    const Time day_one_start_time = Config::get_instance()->get_day_one_start_time();
    const Time day_one_end_time = Config::get_instance()->get_day_one_end_time();
    const Time day_end_time = Config::get_instance()->get_day_end_time();  // Second and third day
    const bool is_first_day = curr_time >= day_one_start_time && curr_time < day_one_end_time;
    const Time race_plan_end_time = is_first_day ? day_one_end_time : day_end_time;
    auto start = std::chrono::high_resolution_clock::now();
    // TODO(Someone): Sweep parameters to make population more diverse
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
    race_plan_creation[i] = duration.count() / 1'000'000.0;
    result_luts[i] = std::make_shared<ResultsLut>();
    population[i] = race_plan;
  }
}

void V2Optimizer::evaluate_population() {
  for (int i=0; i < initial_population_size; i++) {
    if (fix_num_loops) {
      population[i].set_score(mps2kph(population[i].get_average_speed()));
    } else {
      population[i].set_score(mps2kph(population[i].get_num_loops()));
    }
  }
}
void V2Optimizer::simulate_population() {
  RUNTIME_EXCEPTION(population.size() > 0, "Population is empty");

  if (initial_population_size > 1) {
    for (int i=0; i < initial_population_size; i++) {
      threads[i] = std::thread(thread_run_sim, simulator, route, result_luts[i], &population[i], &thread_manager);
    }
    for (int i=0; i < initial_population_size; i++) {
      threads[i].join();
    }
  } else {
    simulator->run_sim(route, &population[0], result_luts[0]);
  }
}


V2Optimizer::V2Optimizer(std::shared_ptr<Simulator> simulator, std::shared_ptr<Route> route)
    : Optimizer(simulator, route), num_threads(std::max(1u, static_cast<unsigned int>(
      std::thread::hardware_concurrency() * Config::get_instance()->get_threads()))),
      initial_population_size(Config::get_instance()->get_initial_population()),
      num_generations(Config::get_instance()->get_num_generations()),
      survival_percentage(Config::get_instance()->get_survival_percentage()),
      fix_num_loops(Config::get_instance()->get_fix_num_loops()),
      parents_percentage(Config::get_instance()->get_parents_percentage()) {}

