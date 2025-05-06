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
  size_t best_average_speed = mps2kph(best_race_plan.get_average_speed());
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

RacePlan V2Optimizer::crossover_parents(RacePlan parent_a, RacePlan parent_b, const std::shared_ptr<Route> route) {

  RUNTIME_EXCEPTION(!parent_a.is_empty() && !parent_b.is_empty(), "Parent race plans cannot be empty");

  // Variables to be fed to raceplan constructor on return
  std::vector<std::vector<std::pair<size_t, size_t>>> all_segments;
  std::vector<std::vector<std::pair<size_t, size_t>>> all_segment_speeds;
  std::vector<std::vector<bool>> all_acceleration_segments;
  std::vector<std::vector<double>> all_acceleration_values;
  
  // Variables for current loop
  std::vector<std::pair<size_t, size_t>> loop_segments;
  std::vector<std::pair<size_t, size_t>> loop_segment_speeds;
  std::vector<bool> loop_acceleration_segments;
  std::vector<double> loop_acceleration_values;

  // Segment specific variables
  std::pair<size_t, size_t> segment;
  std::pair<size_t, size_t> segment_speed;
  bool acceleration_segment; // CLARIFICATION - tells if this is an acceleration segment or not
  double acceleration_value;

  // REFACTORING TGT
  auto add_segment = [&]() {
    loop_segments.emplace_back(segment);
    loop_segment_speeds.emplace_back(segment_speed);
    loop_acceleration_segments.emplace_back(acceleration_segment);
    loop_acceleration_values.emplace_back(acceleration_value);
  };

  auto print_segment = [&]() {
    std::cout << "Segment: [" << segment.first << "," << segment.second << "]" << std::endl;
    std::cout << "Segment Speed: [" << segment_speed.first << "," << segment_speed.second << "]" << std::endl;
    std::cout << "Acceleration: " << (acceleration_segment ? "True" : "False") << std::endl;
    std::cout << "Acceleration Value: " << acceleration_value << std::endl;
  };
  // REFACTORING TGT

  // COMPILE ALL THE GLOBAL VARIABLES HERE AND FIND OUT HOW TO ACTUALLY GET THEM
  size_t largest_index{0};
  size_t mass{0};
  size_t power_allowance{0};
  double minimum_acceleration{0};
  double maximum_deceleration{0};
  // COMPILE ALL THE GEOGRAPHIC VARIABLES HERE AND FIND OUT HOW TO ACTUALLY GET THEM
  
  const size_t num_loops = parent_a.get_num_loops(); // assumes the same for both
  const size_t num_blocks = parent_a.get_num_blocks();
  
  auto corner_indxs = route->get_cornering_segment_bounds();
  auto speed_corner_limits = route->get_cornering_speed_bounds();
  size_t num_corners = corner_idxs.size();
  const int total_num_corners = corner_indxs.size();

  std::uniform_int_distribution<size_t> distn(0, num_loops - 1);
  std::mt19937 cross_gen(9090); // seed

  std::uniform_real_distribution<size_t> parent_distn(0, 1);
  std::mt19937 parent_gen(1984); // seed

  // Preload segment data
  auto segments_a = parent_a.get_segments();
  auto segments_b = parent_b.get_segments();
  auto segment_speeds_a = parent_a.get_segment_speeds();
  auto segment_speeds_b = parent_b.get_segment_speeds();
  auto is_acc_segment_a = parent_a.get_acceleration_segments();
  auto is_acc_segment_b = parent_b.get_acceleration_segments();

  struct consideration {
    double acceleration;
    size_t idx;
  };

  auto acc_segment_checker = [route](size_t cur_corner_speed, size_t next_corner_speed, size_t cur_acceleration_start_idx, size_t cur_acceleration_end_idx, size_t acc_end_limit_idx){
    /*
    Returns an acceleration for the segment if it finds one or -10000 if it does not.
    Acceleration is investigated in compliance with:
    - mav heuristic
    - min_acceleration
    - acceleration distance to next corner
    */

    size_t considered_end_idx = cur_acceleration_end_idx - 1;
    double considered_acceleration;
    size_t considered_distance;
    double considered_power_consumption;

    do {
      considered_end_idx = considered_end_idx + 1;
      considered_distance = route->get_precomputed_distances().get_value(cur_acceleration_start_idx, considered_end_idx);
      considered_acceleration = calc_acceleration(cur_corner_speed, next_corner_speed, considered_distance);
      considered_power_consumption = considered_acceleration * next_corner_speed * mass;

    } while (
      (considered_power_consumption > power_allowance || considered_acceleration < minimum_acceleration) && considered_end_idx <= acc_end_limit_idx
    );

    size_t ret = -10000;
    if (considered_end_idx <= acc_end_limit_idx){
      ret = considered_acceleration;
    }
    
    struct consideration ret = {considered_acceleration, considered_end_idx};
    return ret;
  };

  auto aggressive_dec_segment_checker = [route](size_t cur_corner_speed, size_t next_corner_speed, size_t cur_deceleration_start_idx, size_t cur_deceleration_end_idx, size_t dec_start_limit_idx){
    /*
    Returns a deceleration for the segment if it finds one or -10000 if it does not.
    Deceleration is investigated in compliance with:
    - min_deceleration and max_deceleration
    - deceleration distance
    Two deceleration segment checkers are made - one for aggressive straignt, in which the start of the deceleration segment is investigated - and one for normal straight, in which the end of the deceleration segment is investigated.
    The convention used in this function is that deceleration is NEGATIVE SIGNED in the variable.
    */

   size_t considered_start_idx = cur_deceleration_start_idx + 1;
   double considered_deceleration;
   size_t considered_distance;
   double considered_power_consumption;

   do {
     considered_start_idx = considered_start_idx - 1;
     considered_distance = route->get_precomputed_distances().get_value(considered_start_idx, cur_deceleration_end_idx);
     considered_deceleration = calc_acceleration(cur_corner_speed, next_corner_speed, considered_distance);
     considered_power_consumption = considered_deceleration * next_corner_speed * mass;

   } while (
    (considered_deceleration < 0 && (considered_power_consumption > power_allowance || considered_deceleration > minimum_acceleration) && considered_start_idx >= dec_start_limit_idx) || // if deceleration turns out to be positive, aka speed going up
    (considered_deceleration > 0 && considered_deceleration > maximum_deceleration && considered_start_idx >= dec_start_limit_idx) // if deceleration turns out to be negative, aka speed going down
  );

   size_t ret = -10000;
   if (considered_start_idx >= dec_start_limit_idx){
     ret = considered_deceleration;
   }
   
   struct consideration ret = {considered_deceleration, considered_start_idx};
   return ret;
  };

  auto normal_dec_segment_checker = [route](size_t cur_corner_speed, size_t next_corner_speed, size_t cur_deceleration_start_idx, size_t cur_deceleration_end_idx, size_t dec_end_limit_idx){
    /*
    Returns a deceleration for the segment if it finds one or -10000 if it does not.
    Deceleration is investigated in compliance with:
    - min_deceleration and max_deceleration
    - deceleration distance
    Two deceleration segment checkers are made - one for aggressive straignt, in which the start of the deceleration segment is investigated - and one for normal straight, in which the end of the deceleration segment is investigated.
    The convention used in this function is that deceleration is NEGATIVE SIGNED in the variable.
    */

    size_t considered_end_idx = cur_deceleration_start_idx - 1;
    double considered_deceleration;
    size_t considered_distance;
    double considered_power_consumption;

    do {
      considered_end_idx = considered_end_idx + 1;
      considered_distance = route->get_precomputed_distances().get_value(cur_deceleration_start_idx, considered_end_idx);
      considered_deceleration = calc_acceleration(cur_corner_speed, next_corner_speed, considered_distance);
      considered_power_consumption = considered_deceleration * next_corner_speed * mass;

    } while (
      (considered_deceleration < 0 && (considered_power_consumption > power_allowance || considered_deceleration > minimum_acceleration) && considered_end_idx <= dec_end_limit_idx) || // if deceleration turns out to be positive, aka speed going up
      (considered_deceleration > 0 && considered_deceleration > maximum_deceleration && considered_end_idx <= dec_end_limit_idx) // if deceleration turns out to be negative, aka speed going down  
    );

    size_t ret = -10000;
    if (considered_end_idx <= acc_end_limit_idx){
      ret = considered_acceleration;
    }
    
    struct consideration ret = {considered_acceleration, considered_end_idx};
    return ret;
  };

  // Holding variables for current between-corner-section
  struct loop_extract{ // can refactor to auto?
    std::vector<std::pair<size_t, size_t>> speeds;
    std::vector<std::pair<size_t, size_t>> indexes;
    std::vector<bool> is_acc_segment;
  };
  // TODO Crosscheck if there are other segment details we need to get

  auto zwischen_zwei_index = [](RacePlan raceplan, size_t loop_nummer, size_t start, size_t end, size_t largest_index){ // largetst_index might need to go in [].
    // Given a raceplan and a loop number, obtain all the start/end speeds and start/end indexes associated between indexes start and end.

    // For the programmer: this depends on consecutive segments being of the form [x1, x2], [x2, x3] - the segment index between consecutive segments is shared.

    assert(start != end);

    std::vector<std::pair<size_t, size_t>> extract_indexes;
    std::vector<std::pair<size_t, size_t>> extract_speeds;
    std::vector<bool> extract_is_acc_segments; 
    auto loop_indexes = raceplan.get_segments()[loop_nummer];
    auto loop_speeds = raceplan.get_speed_profile()[loop_nummer];
    auto loop_is_acc_segments = raceplan.get_acceleration_segments()[loop_nummer];
    size_t loop_size = loop_indexes.size();
    size_t cntr;

    if (start < end){
      for (cntr = 0; cntr < loop_size; cntr++){
        if (loop_indexes[cntr].first >= start && loop_index[cntr].second <= end){
          extract_indexes.push_back(loop_indexes[cntr]);
          extract_speeds.push_back(loop_speeds[cntr]);
          extract_is_acc_segments.push_back(loop_is_acc_segments);
        }
      } 

    } else {
      for (cntr = 0; cntr < loop_size; cntr++){
        if ((loop_indexes[cntr].first >= start && loop_index[cntr].second <= largest_index) || (loop_indexes[cntr].first >= start && loop_index[cntr].second <= end) || (loop_indexes[cntr].first >= 0 && loop_index[cntr].second <= end)){
          extract_indexes.push_back(loop_indexes[cntr]);
          extract_speeds.push_back(loop_speeds[cntr]);
          extract_is_acc_segments.push_back(loop_is_acc_segments);
        }
      }

    }

    struct loop_extract ret = {extract_indexes, extract_speeds, loop_is_acc_segments};
    return ret;
  };

  loop_extract considered_extract;
  size_t parent_choice;
  std::vector<std::pair<size_t, size_t>> considered_speeds, considered_idxs;
  std::vector<bool> considered_is_acc_segments; // crosscheck if this is actually correct
  struct consideration considered;
  double considered_acceleration;
  size_t considered_start_index, considered_end_index;

  size_t cur_block_idx, cur_loop_idx, cur_abs_loop_idx;
  size_t last_corner_idx, cur_corner_idx;
  size_t loops_per_block = num_loops / num_blocks;
  size_t last_speed;
  
  bool meets_contingency_1 = true, meets_contingency_2 = true;
  int loop_num_sample_a, loop_num_sample_b;

  for (cur_block_idx=0; cur_block_idx<num_blocks; cur_block_idx++){
    for (cur_loop_idx=0, cur_loop_idx<loops_per_block; cur_loop_idx++){
      cur_abs_loop_idx = cur_block_idx*loops_per_block + cur_loop_idx;

      // obtain velocity exiting corner
      if (cur_abs_loop_idx != 0){
        last_speed = all_segment_speeds[cur_abs_loop_idx-1];
      } else last_speed = 0;

      if (cur_abs_loop_idx != 0){ // sample any loop
        loop_num_sample_a = distn(cross_gen); // random var outcome parent A
        loop_num_sample_b = distn(cross_gen); // random var outcome parent B  
      } else { // sample ONLY the first one from both parents
        loop_num_sample_a = 0;
        loop_num_sample_b = 0;
      }      

      for (cur_corner_idx = 1; cur_corner_idx < num_corners; cur_corner_idx++){
        last_corner_idx = num_corners - 1;

        parent_choice = parent_distn(parent_gen); // sample which parent

        /*
        Contingency management:
        [1]- if acceleration is not achieveable, vary accelerating distance, attempting to maximise acceleration (with safety factor?)
        [2]- if above unsuccessful, switch parent.
        [3]- if above unsuccessful, follow random calculation order observed in SegmentRoute(?)

        Contingency [1] and [2] are being put in a while loop for concision.
        */

        do {
          if (!meets_contingency_1){
            if (parent_choice < 0.5){ // parent A
              considered_extract = zwischen_zwei_index(parent_a, loop_num_sample_a, corner_indxs[last_corner_idx], corner_indxs[cur_corner_idx], largest_index);
              
            } else { // parent B
              considered_extract = zwischen_zwei_index(parent_b, loop_num_sample_b, corner_indxs[last_corner_idx], corner_indxs[cur_corner_idx], largest_index);
            }

          } else {
            if (parent_choice < 0.5){ // parent B
              considered_extract = zwischen_zwei_index(parent_b, loop_num_sample_b, corner_indxs[last_corner_idx], corner_indxs[cur_corner_idx], largest_index);
            
            } else { // parent A
              considered_extract = zwischen_zwei_index(parent_a, loop_num_sample_a, corner_indxs[last_corner_idx], corner_indxs[cur_corner_idx], largest_index);
            }
          }

          // perform calculation order
          if (considered_extract.indexes.size() == 4){ // we pulled an aggressive straight
            // calculation order is to try preserve the aggressive cruising speed at all costs
            // evaluate acceleration segment first
            considered = acc_segment_checker(last_speed, considered_speeds[0].second, considered_idxs[0].first, considered_idxs[3].first, corner_indxs[cur_corner_idx].first);
            if (considered.acceleration == -10000){
              meets_contingency_1 = false;
            } else {
              considered_acceleration = considered.acceleration;
              considered_end_index = considered.idx;  
            }
  
            // then evaluate deceleration segment
            if (meets_contingency_1){
              considered = aggressive_dec_segment_checker(considered_speeds[1].second, considered_speeds[2].second, considered_idxs[2].first, considered_idxs[2].second, considered_end_index);
            }
            if (considered.acceleration == -10000){
              if (meets_contingency_2) meets_contingency_1 = false;
              else meets_contingency_2 = false;

            } else {
              // meets all the requirements, put all four segments into the vector we build the loops with
              // TODO
            }
            
          } else if (considered_extract.indexes.size() == 2) { // we pulled a normal straight which may need to be converted to a corner straight
  
            if (last_speed < considered_speeds[0].second){ // are we accelerating?
              considered = acc_segment_checker(last_speed, considered_speeds[0].second, considered_idxs[0].first, considered_idxs[0].second, corner_indxs[cur_corner_idx].first);
  
            } else if (last_speed > considered_speeds[0].second){ // or are we decelerating?
              considered = normal_dec_segment_checker(last_speed, considered_speeds[0].second, considered_idxs[0].first, considered_idxs[0].second, corner_indxs[cur_corner_idx].first);
  
            } else { // or our speed is not changing?
              considered.acceleration = 0;
            }
  
            if (considered.acceleration == -10000){
              if (meets_contingency_2) meets_contingency_1 = false;
              else meets_contingency_2 = false;
              
            } else {
              // meets all the requirements, put both segments into the vector we build the loops with
              // TODO            
            }
  
          } else { // we pulled a single length corner straight which might need to be converted to an acceleration straight
            
            if (last_speed < considered_speeds[0].second){ // are we accelerating? if we are, then evaluate the acceleration segment
              considered = acc_segment_checker(last_speed, considered_speeds[0].second, considered_idxs[0].first, corner_idxns[cur_corner_idx].first, corner_indxs[cur_corner_idx].first);
  
            } else if (last_speed > considered_speeds[0].second){ // or are we decelerating? then evaluate the deceleration segment
              considered = normal_dec_segment_checker(last_speed, considered_speeds[0].second, considered_idxs[0].first, corner_idxns[cur_corner_idx].first, corner_indxs[cur_corner_idx].first);
  
            } else { // or our speed is not changing?
              considered.acceleration = 0;
            }
  
            if (considered.acceleration == -10000){
              if (meets_contingency_2) meets_contingency_1 = false;
              else meets_contingency_2 = false;
            } else {
              // meets all the requirements, put both segments into the vector we build the loops with
              // TODO            
            }
          }
        } while (!meets_contingency_1);
  
        if (!meets_contingency_2){
  
          // perform calculation order
  
        }
      }
    }
  }

  return RacePlan(all_segments, all_segment_speeds, all_acceleration_segments, all_acceleration_values);
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
  auto round2Even = [](size_t num) -> int {
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
    // Population size 1 is really only used for debugging
    population[0] = generator->create_plan();
    population[0].print_plan();
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

