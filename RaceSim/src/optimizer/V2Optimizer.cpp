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
#include "opt/GeneticUtilities.hpp"

// Sort function
bool comp_race_plan(const RacePlan& a, const RacePlan& b) {
  return a.get_score() > b.get_score();
}

// Thread function for running a simulation
void thread_create_plan(std::shared_ptr<RacePlanCreator> generator,
                        RacePlan* space,
                        ThreadManager* thread_manager) {
  thread_manager->acquire();
  *space = generator->create_plan();
  thread_manager->release();
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

  spdlog::info("Using {} threads", num_threads);
  init_params();

  // Create initial population
  create_initial_population();

  for (size_t i=0; i < num_generations; i++) {
    // Simulate race plans
    simulate_population();

    // Evaluate race plans and gather fitness scores
    evaluate_population();

    // Sort in order of descending fitness scores
    std::sort(population.begin(), population.end(), comp_race_plan);

    // Crossover fittest parents
    // crossover_population();

    // Mutate the remaining parents
    mutate_population();
  }

  // Process results
  RacePlan best_race_plan = population[0];
  double best_average_speed = mps2kph(best_race_plan.get_average_speed());
  best_race_plan.print_plan();
  ResultsLut best_race_result;
  std::cout << "Best Race Plan Average Speed: " << mps2kph(best_race_plan.get_average_speed()) << "kph" << std::endl;
  std::cout << "Best Race Plan Number of Loops: " << best_race_plan.get_num_loops() << std::endl;

  return best_race_plan;
}

void V2Optimizer::mutate_population() {
  std::uniform_int_distribution<size_t> indices_dist(crossover_num, population_size-1);
  std::unordered_set<size_t> mutated_indices;
  std::random_device rd;
  std::mt19937 gen(rd());

  int num_mutated = 0;
  while (num_mutated < mutation_num) {
    size_t idx = indices_dist(gen);
    if (mutated_indices.find(idx) != mutated_indices.end()) {
      continue;
    }
    mutated_indices.insert(idx);
    std::cout << "Using index " << idx << std::endl;
    RacePlan mutated_plan = Genetic::mutate_plan(population[idx], route);
    num_mutated += 1;
  }
}

void V2Optimizer::crossover_population() {
  // Note that std::uniform_int_distribution is inclusive on both sides
  std::uniform_int_distribution<size_t> parent_indices_dist(0, crossover_num - 1);
  std::unordered_set<size_t> crossed_over_parent_indices;
  std::random_device rd;
  std::mt19937 gen(rd());

  int num_crossovers = 0;
  while (num_crossovers < crossover_num / 2) {
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
  RUNTIME_EXCEPTION(population_size > 0, "Initial population size must be greater than 0");
  RUNTIME_EXCEPTION(num_generations > 0, "Number of generations must be greater than 0");
  RUNTIME_EXCEPTION(crossover_percentage + survival_percentage + mutation_percentage == 100.0, "Survival, "
    "mutation and crossover percentages must add to 100%");
  if (crossover_percentage != 0.0) {
    RUNTIME_EXCEPTION((crossover_percentage / 100.0) * population_size >= 2, "Parents percent must select "
    "at least two parents to breed each generation");
  }
  if (mutation_percentage != 0.0) {
    RUNTIME_EXCEPTION((mutation_percentage / 100.0) * population_size >= 1, "Mutation percentage must "
    "select at least one parent to mutate each generation");
  }

  // Round num to the biggest even number smaller than num
  auto round2Even = [](double num) -> int {
    int rounded_down = static_cast<int>(std::floor(num));
    if (rounded_down % 2 != 0) {
      rounded_down--;
    }
    return rounded_down;
  };

  crossover_num = round2Even((crossover_percentage / 100.0) * population_size);
  mutation_num = static_cast<int>(std::ceil((mutation_percentage / 100.0) * (population_size)));
  survival_num = population_size - crossover_num - mutation_num;

  spdlog::info("----Genetic Optimizer Parameters:-----");
  spdlog::info("Population Size: {}", population_size);
  spdlog::info("Number of Generations: {}", num_generations);
  spdlog::info("Survival Percentage: {}, Number of Population {}", survival_percentage, survival_num);
  spdlog::info("Mutation Percentage: {}, Number of Population: {}", mutation_percentage, mutation_num);
  spdlog::info("Crossover Percentage: {}, Number of Population: {}", crossover_percentage, crossover_num);

  thread_manager = ThreadManager(num_threads);
  result_luts.clear();
  threads.clear();
  population.clear();

  population.resize(population_size);
  result_luts.resize(population_size);
  threads.resize(population_size);
}

void V2Optimizer::create_initial_population() {
  RUNTIME_EXCEPTION(population_size > 0, "Initial population size must be greater than 1");
  RUNTIME_EXCEPTION(population.size() == population_size, "Population vector not resized");
  RUNTIME_EXCEPTION(result_luts.size() == population_size, "Results lut vector not resized");
  // Create a random device to seed the random number generator
  std::random_device rd;

  // Use the Mersenne Twister engine to generate random numbers
  std::mt19937 gen(rd());

  // Define the range for the random numbers
  std::uniform_int_distribution<unsigned int> dis(0, std::numeric_limits<unsigned int>::max());

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
  threads.clear();
  threads.resize(population_size);
  generator = std::make_shared<RacePlanCreator>(route, speed_seed, loop_seed, aggressive_seed,
                                                idx_seed, acceleration_seed, skip_seed);

  auto start = std::chrono::high_resolution_clock::now();
  if (population_size > 1) {
    if (Config::get_instance()->get_log_segmenting()) {
      spdlog::warn("Population size is greater than 1 with logging set to true. "
                    "This will dramatically slow down population creation.");
    }
    for (int i=0; i < population_size; i++) {
      threads[i] = std::thread(thread_create_plan, generator, &population[i], &thread_manager);
    }
    for (int i=0; i < population_size; i++) {
      threads[i].join();
    }
  } else {
    population[0] = generator->create_plan();
  }
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

  for (int i=0; i < population_size; i++) {
    result_luts[i] = std::make_shared<ResultsLut>();
  }
}

void V2Optimizer::evaluate_population() {
  for (int i=0; i < population_size; i++) {
    if (fix_num_loops) {
      population[i].set_score(mps2kph(population[i].get_average_speed()));
    } else {
      population[i].set_score(mps2kph(population[i].get_num_loops()));
    }
  }
}

void V2Optimizer::simulate_population() {
  RUNTIME_EXCEPTION(population.size() > 0, "Population is empty");
  threads.clear();
  threads.resize(population_size);
  if (population_size > 1) {
    for (int i=0; i < population_size; i++) {
      threads[i] = std::thread(thread_run_sim, simulator, route, result_luts[i], &population[i], &thread_manager);
    }
    for (int i=0; i < population_size; i++) {
      threads[i].join();
    }
  } else {
    simulator->run_sim(route, &population[0], result_luts[0]);
  }
}


V2Optimizer::V2Optimizer(std::shared_ptr<Simulator> simulator, std::shared_ptr<Route> route)
    : Optimizer(simulator, route), num_threads(std::max(1u, static_cast<unsigned int>(
      std::thread::hardware_concurrency() * Config::get_instance()->get_threads()))),
      population_size(Config::get_instance()->get_population_size()),
      num_generations(Config::get_instance()->get_num_generations()),
      survival_percentage(Config::get_instance()->get_survival_percentage()),
      fix_num_loops(Config::get_instance()->get_fix_num_loops()),
      crossover_percentage(Config::get_instance()->get_crossover_percentage()),
      mutation_percentage(Config::get_instance()->get_mutation_percentage()),
      mutation_strategy(Config::get_instance()->get_mutation_strategy()) {}

