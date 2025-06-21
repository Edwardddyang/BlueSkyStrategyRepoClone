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
#include <queue>

#include "spdlog/spdlog.h"
#include "opt/V2Optimizer.hpp"
#include "route/Route.hpp"
#include "config/Config.hpp"
#include "utils/Defines.hpp"
#include "utils/Units.hpp"
#include "opt/PopulationGenerator.hpp"

// Sort function
bool comp_race_plan(const RacePlan& a, const RacePlan& b) {
  return a.get_score() > b.get_score();
}

// Thread function for creating a race plan
void thread_create_plan(std::shared_ptr<RacePlanCreator> generator,
                        RacePlan* space,
                        ThreadManager* thread_manager) {
  thread_manager->acquire();
  *space = generator->create_plan();
  thread_manager->release();
}

RacePlan V2Optimizer::optimize() {
  const std::filesystem::path dump_dir = Config::get_instance()->get_dump_dir();
  std::filesystem::create_directories(dump_dir);

  // Create results folder
  bool save_csv = Config::get_instance()->get_save_csv();
  std::filesystem::path results_folder;
  if (save_csv) {
    const std::string strat_root = Config::get_instance()->get_strat_root();
    results_folder = dump_dir / "Acceleration_Results";
    std::filesystem::create_directories(results_folder);
  }

  spdlog::info("Using {} threads", num_threads);
  init_params();

  // Create initial population
  create_initial_population();

  for (size_t i=0; i < num_generations + 1; i++) {
    // Simulate race plans
    simulate_population();

    // Evaluate race plans and gather fitness scores
    evaluate_population();

    // Sort in order of descending fitness scores
    std::sort(population.begin(), population.end(), comp_race_plan);

    this->print_population_status(i);
  
    if (i == num_generations) {
      break;
    }
    // Crossover fittest parents
    crossover_population();

    // Mutate the remaining parents
    mutation_logger("//////////////////////////////////////////");
    mutation_logger("// Mutating population in generation " + std::to_string(i) + " //");
    mutation_logger("//////////////////////////////////////////\n");

    mutate_population();
  }

  // Process results
  RacePlan best_race_plan = population[0];
  ResultsLut best_race_plan_result = *result_luts[0];
  size_t best_average_speed = mps2kph(best_race_plan.get_average_speed());
  if (save_csv) {
    best_race_plan_result.write_logs((results_folder / "Acceleration.csv").string());
  }

  if (best_race_plan.is_viable()) {
    best_race_plan.print_plan();
    ResultsLut best_race_result;
    std::cout << "Best Race Plan Average Speed: " << mps2kph(best_race_plan.get_average_speed()) << "kph" << std::endl;
    std::cout << "Best Race Plan Number of Loops: " << best_race_plan.get_num_loops() << std::endl;
  } else {
    std::cout << "No plan is viable" << std::endl;
    std::cout << "Inviability reason: " << best_race_plan.get_inviability_reason() << std::endl;
  }
  
  best_race_plan.export_json();

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
    mutate_plan(&population[idx]);
    num_mutated += 1;
  }
}

void V2Optimizer::crossover_population() {
  // Note that std::uniform_int_distribution is inclusive on both sides
  if (crossover_num == 0) {
    return;
  }
  std::uniform_int_distribution<size_t> parent_indices_dist(0, crossover_num - 1);
  std::unordered_set<size_t> crossed_over_parent_indices;
  std::random_device rd;
  std::mt19937 gen(rd());

  int num_crossovers = 0;
  RUNTIME_EXCEPTION(crossover_num != 0 && static_cast<int>(crossover_num) % 2 == 0,
                    "Number of the population to crossover is not divisible by 2 or not greater than 0");
  while (num_crossovers < crossover_num / 2) {
    // Select two parents to crossover
    const size_t parent_a_idx = parent_indices_dist(gen);
    const size_t parent_b_idx = parent_indices_dist(gen);
    crossover_logger("Selected parent a index " + std::to_string(parent_a_idx) +
                     ", parent b index " + std::to_string(parent_b_idx));
    if (parent_a_idx == parent_b_idx) {
      continue;
    }
    num_crossovers = num_crossovers + 1;
    RacePlan parent_a = population[parent_a_idx];
    RacePlan parent_b = population[parent_b_idx];

    RacePlan child_plan = crossover_parents(parent_a, parent_b);
    population.push_back(child_plan);
  }
}

RacePlan V2Optimizer::crossover_parents(RacePlan parent_a, RacePlan parent_b){
  RUNTIME_EXCEPTION(!parent_a.is_empty() && !parent_b.is_empty(), "Parents of crossover cannot be empty");
  const std::string crossover_type = Config::get_instance()->get_crossover_strategy();
  RUNTIME_EXCEPTION(CROSSOVER_STRATEGIES.find(crossover_type) != CROSSOVER_STRATEGIES.end(),
                    "Unrecognized crossover strategy {}", crossover_type);

  if (crossover_type == "LoopCross") {
    return loop_cross(&parent_a, &parent_b, &rng_collection);
  }
  return parent_a;
}

// RacePlan V2Optimizer::splice_loop(RacePlan* parent_a, RacePlan* parent_b, RacePlanCreator::Gen* rng) {
//   RUNTIME_EXCEPTION(parent_a != nullptr, "No parent a loop");
//   RUNTIME_EXCEPTION(parent_b != nullptr, "No parent b loop");
//   RUNTIME_EXCEPTION(rng != nullptr, "rng struct is null for splice_loop");
//   const RacePlan::PlanData a_orig_segments = parent_a->get_orig_segments();
//   const RacePlan::PlanData b_orig_segments = parent_b->get_orig_segments();
//   RUNTIME_EXCEPTION(parent_a->get_num_loops() == parent_b->get_num_loops(),
//                     "Parent a and b must have the same number of loops to use LoopCross crossover strategy");
//   RUNTIME_EXCEPTION(a_orig_segments.size() == b_orig_segments.size(),
//                     "Both parents must have the same number of original segments");
//   std::uniform_int_distribution<int> parent_dist(0, 1);
//   RacePlan::PlanData new_raw_plan;
//   RacePlanCreator::PlanAttributes att;
//   const std::unordered_map<size_t, size_t> corner_end_to_corner_idx = this->route->get_corner_end_map();

//   const size_t num_blocks = a_orig_segments.size();
//   for (size_t block_idx=0; block_idx < num_blocks; block_idx++) {
//     RUNTIME_EXCEPTION(a_orig_segments[block_idx].size() == b_orig_segments[block_idx].size(),
//                       "Loop {} has different number of loops", block_idx);
//     const size_t num_segments = a_orig_segments[block_idx].size();
//     // Find segments where the loop ends at the same corner
//     std::vector<size_t> overlapping_corners;
//     for (size_t i=0; i < num_segments; i++) {
//       const size_t end_idx = a_orig_segments[block_idx][i].end_idx;
//       if (b_orig_segments[block_idx][i].end_idx == end_idx &&
//           corner_end_to_corner_idx.find(end_idx) != corner_end_to_corner_idx.end()) {
//         overlapping_corners.emplace_back(end_idx);
//       }
//     }
//     if (overlapping_corners.size() == 0) {
//       crossover_logger("No overlapping corners, returning parent a");
//       return *parent_a;
//     }

//     // Sample same random corner to splice from
//     std::uniform_int_distribution<int> idx_dist(0, overlapping_corners.size() - 1);
//     const size_t splice_idx = idx_dist(rng->idx_rng);
//     if (block_idx == 0) {
//       // Is empty
//       new_raw_plan.push_back({});
//     } else [
//       new_raw_plan[block_idx] = {};
//     ]

//     // Insert splice from parent A
//     new_raw_plan[block_idx].insert(new_raw_plan[block_idx].end(),
//                     a_orig_segments[block_idx].begin(), a_orig_segments[block_idx].begin() + splice_idx + 1);

//     // Insert splice from parent B
//     new_raw_plan[block_idx].insert(new_raw_plan[block_idx].end(),
//                         b_orig_segments[block_idx].begin() + splice_idx + 1,
//                         b_orig_segments[block_idx].end());

//     // ----A------|-----B------: Have to modify the connection segment between A and B by matching the
//     // starting speed from parent B to the end speed of A
//     const double end_speed = new_raw_plan[block_idx][splice_idx].end_speed;
//     new_raw_plan[block_idx][splice_idx+1].start_speed = end_speed;
//     new_raw_plan[block_idx][splice_idx+1].acceleration_value = calc_acceleration(
//       new_raw_plan[block_idx][splice_idx+1].start_speed,
//       new_raw_plan[block_idx][splice_idx+1].end_speed,
//       new_raw_plan[block_idx][splice_idx+1].distance
//     );
//     // Insert next loop as plan B

//     // We have to insert the next loop from parent B to interface with legalize_loop correctly
//     if (!legalize_loop(&new_raw_plan, ))
//   }

// }


RacePlan V2Optimizer::loop_cross(RacePlan* parent_a, RacePlan* parent_b, RacePlanCreator::Gen* rng) {
  RUNTIME_EXCEPTION(parent_a != nullptr && !parent_a->is_empty(), "Parent a cannot be null and must not be empty");
  RUNTIME_EXCEPTION(parent_b != nullptr && !parent_b->is_empty(), "Parent b cannot be null and must not be empty");
  RUNTIME_EXCEPTION(rng != nullptr, "rng struct is null for loop_cross");
  const RacePlan::PlanData a_orig_segments = parent_a->get_orig_segments();
  const RacePlan::PlanData b_orig_segments = parent_b->get_orig_segments();
  RUNTIME_EXCEPTION(parent_a->get_num_loops() == parent_b->get_num_loops(),
                    "Parent a and b must have the same number of loops to use LoopCross crossover strategy");
  RUNTIME_EXCEPTION(a_orig_segments.size() == b_orig_segments.size(),
                    "Both parents must have the same number of original segments");
  std::uniform_int_distribution<int> parent_dist(0, 1);
  RacePlan::PlanData new_raw_plan;
  RacePlanCreator::PlanAttributes att;

  const size_t num_blocks = a_orig_segments.size();
  crossover_logger("Starting loop crossover technique");
  crossover_logger("Parent A: " + parent_a->get_plan_string());
  crossover_logger("Parent B: " + parent_b->get_plan_string());

  // Whether the previous loop was taken from parent a (0) or parent b (1)
  bool prev_parent_loop = 0;
  // The speed at the last real corner from the previous loop (connection segments start here)
  double prev_parent_last_speed;
  // Next loop of the selected parent
  RacePlan::LoopData next_loop_of_prev_parent;
  for (size_t block_idx = 0; block_idx < num_blocks; block_idx++) {
    const int parent_idx = parent_dist(rng->idx_rng);
    double orig_speed;
    RacePlan::LoopData prev_loop = next_loop_of_prev_parent;

    if (parent_idx == 0) {  // Take loop from parent A
      crossover_logger("Picked parent A for block " + std::to_string(block_idx));
      new_raw_plan.push_back(a_orig_segments[block_idx]);
      orig_speed = a_orig_segments[block_idx][0].start_speed;
      if (block_idx == 0) {
        prev_loop = a_orig_segments[block_idx];
      }
      if (block_idx < num_blocks - 1) {
        next_loop_of_prev_parent = a_orig_segments[block_idx + 1];
      }
    } else {  // Take loop from parent B
      crossover_logger("Picked parent B for block " + std::to_string(block_idx));
      new_raw_plan.push_back(b_orig_segments[block_idx]);
      orig_speed = b_orig_segments[block_idx][0].start_speed;
      if (block_idx == 0) {
        prev_loop = b_orig_segments[block_idx];
      }
      if (block_idx < num_blocks - 1) {
        next_loop_of_prev_parent = b_orig_segments[block_idx + 1];
      }
    }
    if (block_idx != 0) {
      // Correct connection segments to start of parent a loop
      new_raw_plan[block_idx][0].start_speed = prev_parent_last_speed;
      new_raw_plan[block_idx][0].acceleration_value = calc_acceleration(
        new_raw_plan[block_idx][0].start_speed, new_raw_plan[block_idx][0].end_speed,
        new_raw_plan[block_idx][0].distance
      );
      if (!legalize_loop(&new_raw_plan, block_idx, 0, &crossover_logger)) {
        crossover_logger("Legalization of loop failed, adding loop of the previous parent");
        new_raw_plan[block_idx] = prev_loop;
      }
    }

    att.raw_segments.push_back(new_raw_plan[block_idx]);
    generator->create_loop_block(&new_raw_plan[block_idx], this->num_loops_in_block,
      block_idx == num_blocks - 1, block_idx == 0, &att, nullptr
    );

    // Assign the speed of the connection corner
    if (block_idx != num_blocks - 1) {
      const RacePlan::LoopData plan = parent_idx == 0 ? a_orig_segments[block_idx+1] :
                                      b_orig_segments[block_idx+1];
      const size_t last_corner_end_route_idx = plan[0].start_idx;
      const size_t num_segments = new_raw_plan[block_idx].size();
      for (int i = num_segments - 1; i >= 0; i--) {
        if (last_corner_end_route_idx == new_raw_plan[block_idx][i].end_idx) {
          prev_parent_last_speed = new_raw_plan[block_idx][i].end_speed;
        }
      }
    }
  }
  RacePlan new_plan(att.all_segments, att.raw_segments, this->num_loops_in_block);
  return new_plan;
}

void V2Optimizer::init_params() {
  // Create genetic optimizer
  RUNTIME_EXCEPTION(population_size > 0, "Initial population size must be greater than 0");
  RUNTIME_EXCEPTION(num_generations > 0, "Number of generations must be greater than 0");
  RUNTIME_EXCEPTION(crossover_percentage + survival_percentage + mutation_percentage == 100.0, "Survival, "
    "mutation and crossover percentages must add to 100%");
  size_t raw_crossover_num = (crossover_percentage / 100.0) * population_size;
  size_t raw_mutation_num = (mutation_percentage / 100.0) * population_size;
  if (crossover_percentage != 0.0) {
    RUNTIME_EXCEPTION(raw_crossover_num >= 2, "Parents percent must select at least two parents to breed each generation. "
      "(crossover_percentage / 100.0) * population_size is {} which is less than 2", raw_crossover_num);
  }
  if (mutation_percentage != 0.0) {
    RUNTIME_EXCEPTION(raw_mutation_num >= 1, "Mutation percentage must select at least one "
      "parent to mutate each generation. (mutation_percentage / 100.0) * population_size is {} which is less than 1",
      raw_mutation_num);
  }

  // Round num to the biggest even number smaller than num
  auto round2Even = [](size_t num) -> int {
    int rounded_down = static_cast<int>(std::floor(num));
    if (rounded_down % 2 != 0) {
      rounded_down--;
    }
    return rounded_down;
  };

  crossover_num = round2Even(raw_crossover_num);
  mutation_num = static_cast<int>(std::ceil(raw_mutation_num));
  survival_num = population_size - crossover_num - mutation_num;

  spdlog::info("----Genetic Optimizer Parameters:-----");
  spdlog::info("Population Size: {}", population_size);
  spdlog::info("Number of Generations: {}", num_generations);
  spdlog::info("Survival Percentage: {}, Number of Population {}", survival_percentage, survival_num);
  spdlog::info("Mutation Percentage: {}, Number of Population: {}", mutation_percentage, mutation_num);
  spdlog::info("Crossover Percentage: {}, Number of Population: {}", crossover_percentage, crossover_num);

  if (log_optimization && population_size > 1) {
    spdlog::warn("Population size is greater than 1 with optimization logging set to true. "
                  "This will dramatically slow down population creation."); 
  }
  mutation_logger = FileLogger("mutation.log", log_optimization);
  crossover_logger = FileLogger("crossover.log", log_optimization);

  thread_manager = ThreadManager(num_threads);
  result_luts.clear();
  threads.clear();
  population.clear();

  population.resize(population_size);
  result_luts.resize(population_size);
  threads.resize(population_size);
  // Create a random device to seed the random number generator
  std::random_device rd;

  // Use the Mersenne Twister engine to generate random numbers
  std::mt19937 gen(rd());

  // Define the range for the random numbers
  std::uniform_int_distribution<unsigned int> dis(0, std::numeric_limits<unsigned int>::max());
  idx_seed = dis(gen);
  speed_seed = dis(gen);
  acceleration_seed = dis(gen);
  aggressive_seed = dis(gen);
  loop_seed = dis(gen);
  skip_seed = dis(gen);
  if (Config::get_instance()->get_fix_seeds()) {
    idx_seed = Config::get_instance()->get_idx_seed();
    speed_seed = Config::get_instance()->get_speed_seed();
    acceleration_seed = Config::get_instance()->get_acceleration_seed();
    aggressive_seed = Config::get_instance()->get_aggressive_seed();
    loop_seed = Config::get_instance()->get_loop_seed();
    skip_seed = Config::get_instance()->get_skip_seed();
  }
  rng_collection = RacePlanCreator::Gen(
    speed_seed, loop_seed, aggressive_seed, idx_seed, acceleration_seed, skip_seed
  );
}

void V2Optimizer::create_initial_population() {
  RUNTIME_EXCEPTION(population_size > 0, "Initial population size must be greater than 1");
  RUNTIME_EXCEPTION(population.size() == population_size, "Population vector not resized");
  RUNTIME_EXCEPTION(result_luts.size() == population_size, "Results lut vector not resized");

  threads.clear();
  threads.resize(population_size);
  generator = std::make_shared<RacePlanCreator>(route, speed_seed, loop_seed, aggressive_seed,
                                                idx_seed, acceleration_seed, skip_seed);

  auto start = std::chrono::high_resolution_clock::now();
  if (population_size > 1) {
    for (int i=0; i < population_size; i++) {
      threads[i] = std::thread(thread_create_plan, generator, &population[i], &thread_manager);
    }
    for (int i=0; i < population_size; i++) {
      threads[i].join();
    }
  } else {
    // Population size 1 is really only used for debugging
    population[0] = generator->create_plan();
  }
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

  for (int i=0; i < population_size; i++) {
    result_luts[i] = std::make_shared<ResultsLut>();
  }
}

void V2Optimizer::print_population_status(int generation) {
  if (generation > -1) {
    std::cout << "#############################################" << std::endl;
    std::cout << "########## Generation " << generation << " ####################" << std::endl;
    std::cout << "#############################################" << std::endl;
  }
  std::cout << "Population Average Speed: " << mps2kph(population_average_speed) << "kph" << std::endl;
  std::cout << "Number of Viable Plans: " << num_viable_plans << std::endl;
  std::cout << "Best Race Plan Average Speed: " << mps2kph(population[0].get_average_speed()) << std::endl;
  std::cout << "Best Loop Plans: " << population[0].get_num_loops() << std::endl;
  if (Config::get_instance()->get_print_population()) {
    int i = 0;
    for (const auto& plan : population) {
      std::cout << "Average Speed of " << i << ": " << mps2kph(plan.get_average_speed()) << std::endl;
      i++;
    }
  }
  std::cout << std::endl;
}

void V2Optimizer::evaluate_population() {
  num_viable_plans = 0;
  double total_speed = 0.0;
  for (int i=0; i < population_size; i++) {
    if (fix_num_loops) {
      population[i].set_score(mps2kph(population[i].get_average_speed()));
    } else {
      population[i].set_score(mps2kph(population[i].get_num_loops()));
    }
    if (!population[i].is_viable()) {
      population[i].set_score(-1.0);
    } else {
      num_viable_plans += 1;
      total_speed += population[i].get_average_speed();
    }
  }
  population_average_speed = total_speed / num_viable_plans;
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
      mutation_strategy(Config::get_instance()->get_mutation_strategy()),
      max_motor_power(kw2watts(Config::get_instance()->get_max_motor_power())),
      acceleration_power_budget(Config::get_instance()->get_acceleration_power_budget() *
                                kw2watts(Config::get_instance()->get_max_motor_power())),
      car_mass(Config::get_instance()->get_car_mass()),
      max_acceleration(Config::get_instance()->get_max_acceleration()),
      max_deceleration(Config::get_instance()->get_max_deceleration()),
      log_optimization(Config::get_instance()->get_log_optimization()),
      route_distances(route->get_precomputed_distances()),
      num_loops_in_block(Config::get_instance()->get_num_repetitions()) {}
