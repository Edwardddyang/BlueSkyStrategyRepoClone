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
    // mutate_population();
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
  }

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
//    mutate_plan(&population[idx]);
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

// possible to refactor both blend and point routines into single function to reuse code

RacePlan V2Optimizer::crossover_parents(RacePlan parent_a, RacePlan parent_b){
  RUNTIME_EXCEPTION(!parent_a.is_empty() && !parent_b.is_empty(), "Parent race plans cannot be empty");

  // Variables to be fed to raceplan constructor on return
  std::vector<std::vector<std::pair<size_t, size_t>>> all_segments;
  std::vector<std::vector<std::pair<double, double>>> all_segment_speeds;
  std::vector<std::vector<bool>> all_acceleration_segments;
  std::vector<std::vector<double>> all_acceleration_values;
  
  // Variables for current loop
  std::vector<std::pair<size_t, size_t>> loop_segments;
  std::vector<std::pair<double, double>> loop_segment_speeds;
  std::vector<bool> loop_acceleration_segments;
  std::vector<double> loop_acceleration_values;

  // Segment specific variables
  std::pair<size_t, size_t> segment;
  std::pair<double, double> segment_speed;
  bool acceleration_segment; // CLARIFICATION - tells if this is an acceleration segment or not
  double acceleration_value;

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

  auto add_loop = [&]() {
    all_segments.push_back(loop_segments);
    all_segment_speeds.push_back(loop_segment_speeds);
    all_acceleration_segments.push_back(loop_acceleration_segments);
    all_acceleration_values.push_back(loop_acceleration_values);
  };

  auto clear_loop = [&]() {
    loop_segments.clear();
    loop_segment_speeds.clear();
    all_acceleration_segments.clear();
    all_acceleration_values.clear();
  };

  size_t largest_index = route->get_num_points() - 1;
  size_t mass = Config::get_instance()->get_car_mass();
  size_t power_allowance = Config::get_instance()->get_max_motor_power();
  const double minimum_acceleration = Config::get_instance()->get_min_acceleration();
  const double maximum_acceleration = Config::get_instance()->get_min_acceleration();
  const double maximum_deceleration = Config::get_instance()->get_max_deceleration();
  const double tgt_average_speed = Config::get_instance()->get_average_speed();
  const double corner_speed_min_ratio = Config::get_instance()->get_corner_speed_min();
  const double corner_speed_max_ratio = Config::get_instance()->get_corner_speed_max();
  const double blend_alpha = 0.3; // need to move this to config

  const size_t num_loops = parent_a.get_num_loops(); // assumes the same for both
  const size_t num_blocks = parent_a.get_num_blocks();
  
  auto corner_indxs = route->get_cornering_segment_bounds();
  auto speed_corner_limits = route->get_cornering_speed_bounds();
  size_t num_corners = corner_indxs.size();
  const int total_num_corners = corner_indxs.size();

  std::uniform_real_distribution<double> skip_dist(0.0, 1.0);
  std::mt19937 skip_gen(Config::get_instance()->get_skip_seed());

  std::uniform_int_distribution<size_t> distn(0, num_loops - 1);
  std::mt19937 cross_gen(9090); // seed

  std::uniform_real_distribution<double> parent_distn(0, 1);
  std::mt19937 parent_gen(1984); // seed

  // Preload segment data
  auto segments_a = parent_a.get_segments();
  auto segments_b = parent_b.get_segments();
  auto segment_speeds_a = parent_a.get_speed_profile();
  auto segment_speeds_b = parent_b.get_speed_profile();
  auto is_acc_segment_a = parent_a.get_acceleration_segments();
  auto is_acc_segment_b = parent_b.get_acceleration_segments();

  auto precomputed_distances = route->get_precomputed_distances();

  struct consideration {
    double acceleration;
    size_t idx;
  };

  auto acc_segment_checker = [mass, power_allowance, minimum_acceleration, precomputed_distances, largest_index](size_t cur_corner_speed, size_t next_corner_speed, size_t cur_acceleration_start_idx, size_t cur_acceleration_end_idx, size_t acc_end_limit_idx){
    /*
    Returns an acceleration for the segment if it finds one or -10000 if it does not.
    Acceleration is investigated in compliance with:
    - mav heuristic
    - min_acceleration
    - acceleration distance to next corner
    */
    size_t considered_end_idx = cur_acceleration_end_idx == 0 ? largest_index - 1 : cur_acceleration_end_idx - 1;
    double considered_acceleration;
    size_t considered_distance;
    double considered_power_consumption;

    do {
      considered_end_idx = considered_end_idx + 1 % largest_index;
      considered_distance = precomputed_distances.get_value(cur_acceleration_start_idx, considered_end_idx);
      considered_acceleration = calc_acceleration(cur_corner_speed, next_corner_speed, considered_distance);
      considered_power_consumption = considered_acceleration * next_corner_speed * mass;

    } while (
      (considered_power_consumption > power_allowance || considered_acceleration < minimum_acceleration) && considered_end_idx <= acc_end_limit_idx
    );

    if (considered_end_idx > acc_end_limit_idx){
      considered_acceleration = -10000;
    }
    
    struct consideration ret = {considered_acceleration, considered_end_idx};
    return ret;
  };

  auto aggressive_dec_segment_checker = [mass, power_allowance, minimum_acceleration, maximum_deceleration, precomputed_distances, largest_index](size_t cur_corner_speed, size_t next_corner_speed, size_t cur_deceleration_start_idx, size_t cur_deceleration_end_idx, size_t dec_start_limit_idx){
    /*
    Returns a deceleration for the segment if it finds one or -10000 if it does not.
    Deceleration is investigated in compliance with:
    - min_deceleration and max_deceleration
    - deceleration distance
    Two deceleration segment checkers are made - one for aggressive straignt, in which the start of the deceleration segment is investigated - and one for normal straight, in which the end of the deceleration segment is investigated.
    The convention used in this function is that deceleration is NEGATIVE SIGNED in the variable.
    */

    size_t considered_start_idx = cur_deceleration_start_idx == largest_index - 1 ? 0 : cur_deceleration_start_idx + 1;
    double considered_deceleration;
    size_t considered_distance;
    double considered_power_consumption;

    do {
      considered_start_idx = considered_start_idx == 0 ? largest_index : considered_start_idx - 1;
      considered_distance = precomputed_distances.get_value(considered_start_idx, cur_deceleration_end_idx);
      considered_deceleration = calc_acceleration(cur_corner_speed, next_corner_speed, considered_distance);
      considered_power_consumption = considered_deceleration * next_corner_speed * mass;

    } while (
      (considered_deceleration < 0 && (considered_power_consumption > power_allowance || considered_deceleration > minimum_acceleration) && considered_start_idx >= dec_start_limit_idx) || // if deceleration turns out to be positive, aka speed going up
      (considered_deceleration > 0 && considered_deceleration > maximum_deceleration && considered_start_idx >= dec_start_limit_idx) // if deceleration turns out to be negative, aka speed going down
    );

    if (considered_start_idx < dec_start_limit_idx){
      considered_deceleration = -10000;
    }
   
    struct consideration ret = {considered_deceleration, considered_start_idx};
    return ret;
  };

  auto normal_dec_segment_checker = [mass, power_allowance, minimum_acceleration, maximum_deceleration, precomputed_distances, largest_index](size_t cur_corner_speed, size_t next_corner_speed, size_t cur_deceleration_start_idx, size_t cur_deceleration_end_idx, size_t dec_end_limit_idx){
    /*
    Returns a deceleration for the segment if it finds one or -10000 if it does not.
    Deceleration is investigated in compliance with:
    - min_deceleration and max_deceleration
    - deceleration distance
    Two deceleration segment checkers are made - one for aggressive straignt, in which the start of the deceleration segment is investigated - and one for normal straight, in which the end of the deceleration segment is investigated.
    The convention used in this function is that deceleration is NEGATIVE SIGNED in the variable.
    */

    size_t considered_end_idx = cur_deceleration_end_idx == 0 ? largest_index - 1 : cur_deceleration_end_idx - 1;
    double considered_deceleration;
    size_t considered_distance;
    double considered_power_consumption;

    do {
      considered_end_idx = considered_end_idx + 1 % largest_index;
      considered_distance = precomputed_distances.get_value(cur_deceleration_start_idx, considered_end_idx);
      considered_deceleration = calc_acceleration(cur_corner_speed, next_corner_speed, considered_distance);
      considered_power_consumption = considered_deceleration * next_corner_speed * mass;

    } while (
      (considered_deceleration < 0 && (considered_power_consumption > power_allowance || considered_deceleration > minimum_acceleration) && considered_end_idx <= dec_end_limit_idx) || // if deceleration turns out to be positive, aka speed going up
      (considered_deceleration > 0 && considered_deceleration > maximum_deceleration && considered_end_idx <= dec_end_limit_idx) // if deceleration turns out to be negative, aka speed going down  
    );

    if (considered_end_idx > dec_end_limit_idx){
      considered_deceleration = -10000;
    }
    
    struct consideration ret = {considered_deceleration, considered_end_idx};
    return ret;
  };

  // Holding variables for current between-corner-section
  struct loop_extract{ // can refactor to auto?
    std::vector<std::pair<size_t, size_t>> speeds;
    std::vector<std::pair<double, double>> indexes;
    std::vector<bool> is_acc_segment;
  };

  auto zwischen_zwei_index = [](RacePlan raceplan, size_t loop_nummer, size_t start, size_t end, size_t largest_index){
    // Given a raceplan and a loop number, obtain all the start/end speeds and start/end indexes associated between indexes start and end.

    // For the programmer: this depends on consecutive segments being of the form [x1, x2], [x2, x3] - the segment index between consecutive segments is shared.

    assert(start != end);

    std::vector<std::pair<size_t, size_t>> extract_indexes;
    std::vector<std::pair<double, double>> extract_speeds;
    std::vector<bool> extract_is_acc_segments; 
    auto loop_indexes = raceplan.get_segments()[loop_nummer];
    auto loop_speeds = raceplan.get_speed_profile()[loop_nummer];
    auto loop_is_acc_segments = raceplan.get_acceleration_segments()[loop_nummer];
    size_t loop_size = loop_indexes.size();
    size_t cntr;

    if (start < end){
      for (cntr = 0; cntr < loop_size; cntr++){
        if (loop_indexes[cntr].first >= start && loop_indexes[cntr].second <= end){
          extract_indexes.push_back(loop_indexes[cntr]);
          extract_speeds.push_back(loop_speeds[cntr]);
          extract_is_acc_segments.push_back(loop_is_acc_segments[cntr]);
        }
      } 

    } else {
      for (cntr = 0; cntr < loop_size; cntr++){
        if ((loop_indexes[cntr].first >= start && loop_indexes[cntr].second <= largest_index) || (loop_indexes[cntr].first >= start && loop_indexes[cntr].second <= end) || (loop_indexes[cntr].first >= 0 && loop_indexes[cntr].second <= end)){
          extract_indexes.push_back(loop_indexes[cntr]);
          extract_speeds.push_back(loop_speeds[cntr]);
          extract_is_acc_segments.push_back(loop_is_acc_segments[cntr]);
        }
      }

    }

    struct loop_extract ret = {extract_indexes, extract_speeds, loop_is_acc_segments};
    return ret;
  };

  loop_extract considered_extract, considered_extract_a, considered_extract_b;
  size_t parent_choice;
  std::vector<std::pair<double, double>> considered_speeds, considered_idxs;
  std::vector<bool> considered_is_acc_segments;
  struct consideration considered;
  double considered_acceleration;
  size_t considered_start_index, considered_end_index;

  size_t cur_block_idx, cur_loop_idx, cur_abs_loop_idx;
  size_t last_corner_idx, cur_corner_idx;
  size_t loops_per_block = num_loops / num_blocks;
  size_t last_speed;

  struct cross_segment_results{
    std::vector<std::pair<size_t, size_t>> segments;
    std::vector<std::pair<double, double>> segment_speeds;
    std::vector<bool> is_acc_segments;
    std::vector<double> acc_values;

    void add_segment(std::pair<size_t, size_t> segment, std::pair<double, double> segment_speed, bool is_acc_segment, double acc_value){
      this->segments.push_back(segment);
      this->segment_speeds.push_back(segment_speed);
      this->is_acc_segments.push_back(is_acc_segment);
      this->acc_values.push_back(acc_value);
    }

    void all_clear(){
      this->segments.clear();
      this->segment_speeds.clear();
      this->is_acc_segments.clear();
      this->acc_values.clear();
    }
  };

  auto add_cross_segments_to_loop = [add_segment, loop_segment_speeds, loop_acceleration_segments, loop_acceleration_values](cross_segment_results results){
    assert(results.segments.size() == results.segment_speeds.size() && results.segment_speeds.size() == results.is_acc_segments.size() && results.is_acc_segments() == results.acc_value());
    
    std::pair<size_t, size_t> segment;
    std::pair<double, double> segment_speed;
    bool acceleration_segment;
    double acceleration_value;

    for (size_t i=0; i<results.segments.size(); i++){
      segment = results.segments[i];
      segment_speed = results.segment_speeds[i];
      acceleration_segment = results.is_acc_segments[i];
      acceleration_value = results.acc_values[i];
      add_segment();
    }
  };

  double point_cross_threshold = 0.3;
  double blend_cross_threshold = 0.6;
  assert(point_cross_threshold + blend_cross_threshold < 1);
  enum class crossType{
    point_cross,
    blend_cross,
    no_cross
  };
  auto cross_picker = [](std::mt19937 gen, double point_cross_threshold, double blend_cross_threshold){ // just use speed_gen or acc_gen as the generator
    std::uniform_real_distribution<double> choice_cross_type (0, 1);
    double sample = choice_cross_type(gen);
    
    crossType ret;
    if (sample < point_cross_threshold){
      ret = crossType::point_cross;
    } else if (sample < point_cross_threshold + blend_cross_threshold){
      ret = crossType::blend_cross;
    } else ret = crossType::no_cross;

    return ret;
  };

  bool final_corner{false};
  
  // only for point cross
  bool meets_contingency_1 = true, meets_contingency_2 = true;
  size_t loop_num_sample_a, loop_num_sample_b;

  for (cur_block_idx=0; cur_block_idx<num_blocks; cur_block_idx++){
    // obtain velocity exiting corner
    if (cur_block_idx != 0){
      last_speed = loop_segment_speeds[cur_block_idx*loops_per_block-1].second;
    } else last_speed = 0;

    if (cur_block_idx != 0){
      loop_num_sample_a = distn(cross_gen);
      loop_num_sample_b = distn(cross_gen);
    } else {
      loop_num_sample_a = 0;
      loop_num_sample_b = 0;
    }
    
    for (cur_corner_idx = 1; cur_corner_idx <= num_corners; cur_corner_idx++){

      last_corner_idx = cur_corner_idx - 1;
      if (cur_corner_idx == num_corners) final_corner = true;

      considered_extract_a = zwischen_zwei_index(parent_a, loop_num_sample_a, corner_indxs[last_corner_idx].second, corner_indxs[cur_corner_idx].second, largest_index);
      considered_extract_b = zwischen_zwei_index(parent_b, loop_num_sample_b, corner_indxs[last_corner_idx].second, corner_indxs[cur_corner_idx].second, largest_index);

      // choose cross type
      auto cross_type = cross_picker(skip_gen, point_cross_threshold, blend_cross_threshold);

      std::vector<std::pair<size_t, size_t>> zwischen_segments;
      std::vector<std::pair<double, double>> zwischen_segment_speeds;
      std::vector<bool> zwischen_is_acc_segments;
      std::vector<double> zwischen_acc_value;
      cross_segment_results results = {zwischen_segments, zwischen_segment_speeds, zwischen_is_acc_segments, zwischen_acc_value};

      consideration considered;

      bool works = false;

      if (cross_type == crossType::blend_cross){ // apply blend cross
      /*
      Algorithm and contingency management:
      [0] The following only applies if both parent A and B experience a normal straight at the same between-corner area.
          If they do not (i.e. aggressive straight), then apply the point mutation routine.
      [1] For a normal straight, define v_A and v_B as the cornering speeds for parents A and B defined for a certain corner.
          Make sure that the minimum of the below range is reachable without violating acceleration limits.
          Select a speed from the uniform distribution [min(v_A, v_B) - alpha * abs(v_A - v_B), max(v_A, v_B) - alpha * abs(v_A - v_B)] = [v_ai, v_bi].
          Evaluate an acceleration, distance pair with acc_segment_checker.
      [2] If acc_segment_checker cannot find a given acceleration, then identify the maximum speed reachable in [v_ai, v_max]
          If it happens that v_max < v_ai, then fix v_max * some_safety_factor as the cornering speed and proceed with the next corner.
      */

        results.all_clear();

        if (considered_extract_a.indexes.size() <= 2 && considered_extract_b.indexes.size() <= 2){ // both normal straights - enter [1]

          double v_a = considered_extract_a.speeds.back().second;
          double v_b = considered_extract_b.speeds.back().second;
          double v_ai = std::min(v_a,v_b) - blend_alpha*std::abs(v_a-v_b);
          double v_bi = std::max(v_a,v_b) + blend_alpha*std::abs(v_a-v_b);

          // [1] - check feasibility of v_ai
          considered = acc_segment_checker(last_speed, v_ai, considered_extract_a.indexes[0].first, considered_extract_a.indexes[0].first, corner_indxs[cur_corner_idx].first); // just brute force across all indexes between this and the next corner

          if (considered.acceleration != -10000){
            // [1] blend speeds
            std::uniform_real_distribution<double> v_blend_dist(v_ai, v_bi);
            double considered_blend_speed = v_blend_dist(skip_gen);

            // [1] - check feasibility of considered_blend_dist
            // TODO use acc_segment_checker -> and don't forget to apply the safety factor this time!
            if (considered_blend_speed > last_speed){
              considered = acc_segment_checker(last_speed, considered_blend_speed, considered_extract_a.indexes[0].first, considered_extract_a.indexes[0].first, corner_indxs[cur_corner_idx].first); // just brute force across all indexes between this and the next corner

              if (considered.acceleration != -10000){
                // add segments
                // RELIES ON each cornering segment ending exactly at the index of the corner end and no later

                // acceleration
                results.add_segment(std::make_pair(considered_extract_a.indexes[0].first, considered.idx), std::make_pair(last_speed, considered_blend_speed), true, considered.acceleration);
                //corner
                results.add_segment(std::make_pair(considered.idx, considered_extract_a.indexes.back().second), std::make_pair(considered_blend_speed, considered_blend_speed), false, 0.0);
              } else !works;

            } else if (considered_blend_speed < last_speed){
              considered = normal_dec_segment_checker(last_speed, considered_blend_speed, considered_extract_a.indexes[0].first, considered_extract_a.indexes[0].first, corner_indxs[cur_corner_idx].first);

              if (considered.acceleration != -10000){
                // add segments
                // acceleration
                results.add_segment(std::make_pair(considered_extract_a.indexes[0].first, considered.idx), std::make_pair(last_speed, considered_blend_speed), true, considered.acceleration);
                // corner
                results.add_segment(std::make_pair(considered.idx, considered_extract_a.indexes.back().second), std::make_pair(considered_blend_speed, considered_blend_speed), false, 0.0);
              } else !works;

            } else {
              // add segments
              // constant speed all the way
                results.add_segment(std::make_pair(considered_extract_a.indexes[0].first, considered_extract_a.indexes.back().second), std::make_pair(considered_blend_speed, considered_blend_speed), false, 0.0);
            }
          } else !works;
        } else !works;
        if (!works){
          cross_type = crossType::point_cross;
        }        
      }

      if (cross_type == crossType::point_cross){ // apply point cross
        results.all_clear();

        parent_choice = parent_distn(parent_gen);

        /*
        Contingency management:
        [1]- if acceleration is not achieveable, vary accelerating distance, attempting to maximise acceleration (with safety factor?)
        [2]- if above unsuccessful, switch parent.
        [3]- if above unsuccessful, move to no cross.

        Contingency [1] and [2] are being put in a while loop for concision.
        */

        // TODO remove the segment stuff in the do/while block and move it outside

        do {
          if (!meets_contingency_1){
            if (parent_choice < 0.5){ // parent A
              considered_extract = zwischen_zwei_index(parent_a, loop_num_sample_a, corner_indxs[last_corner_idx].second, corner_indxs[cur_corner_idx].second, largest_index);
              
            } else { // parent B
              considered_extract = zwischen_zwei_index(parent_b, loop_num_sample_b, corner_indxs[last_corner_idx].second, corner_indxs[cur_corner_idx].second, largest_index);
            }

          } else {
            if (parent_choice < 0.5){ // parent B
              considered_extract = zwischen_zwei_index(parent_b, loop_num_sample_b, corner_indxs[last_corner_idx].second, corner_indxs[cur_corner_idx].second, largest_index);
            
            } else { // parent A
              considered_extract = zwischen_zwei_index(parent_a, loop_num_sample_a, corner_indxs[last_corner_idx].second, corner_indxs[cur_corner_idx].second, largest_index);
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

              // acceleration segment
              results.add_segment(std::make_pair(considered_idxs[0].first, considered_end_index), std::make_pair(last_speed, considered_speeds[0].second), true, considered_acceleration);

              // aggressive constant speed segment
              results.add_segment(std::make_pair(considered_end_index, considered.idx), std::make_pair(considered_speeds[0].second, considered_speeds[0].second), false, 0.0);

              // deceleration segment
              results.add_segment(std::make_pair(considered.idx, considered_idxs[2].second), std::make_pair(considered_speeds[0].second, considered_speeds[2].second), true, considered.acceleration);

              // cornering segment
              results.add_segment(std::make_pair(considered_idxs[2].second, considered_idxs[3].second), std::make_pair(considered_speeds[2].second, considered_speeds[2].second), false, 0.0);
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
              
              if (considered.acceleration == 0){ // single constant speed segment
                results.add_segment(std::make_pair(considered_idxs[0].first, considered_idxs[1].second), std::make_pair(considered_speeds[0].first, considered_speeds[0].first), false, 0.0);

              } else {
                // acceleration/deceleration segment
                results.add_segment(std::make_pair(considered_idxs[0].first, considered.idx), std::make_pair(last_speed, considered_speeds[0].second), true, considered.acceleration);

                // constant speed segment
                results.add_segment(std::make_pair(considered.idx, considered_idxs[1].second), std::make_pair(considered_speeds[0].second, considered_speeds[0].second), false, 0.0);
              }
            }

          } else { // we pulled a single length corner straight which might need to be converted to an acceleration straight
            
            if (last_speed < considered_speeds[0].second){ // are we accelerating? if we are, then evaluate the acceleration segment
              considered = acc_segment_checker(last_speed, considered_speeds[0].second, considered_idxs[0].first, corner_indxs[cur_corner_idx].first, corner_indxs[cur_corner_idx].first);

            } else if (last_speed > considered_speeds[0].second){ // or are we decelerating? then evaluate the deceleration segment
              considered = normal_dec_segment_checker(last_speed, considered_speeds[0].second, considered_idxs[0].first, corner_indxs[cur_corner_idx].first, corner_indxs[cur_corner_idx].first);

            } else { // or our speed is not changing?
              considered.acceleration = 0;
            }

            if (considered.acceleration == -10000){
              if (meets_contingency_2) meets_contingency_1 = false;
              else meets_contingency_2 = false;
            } else {
              // meets all the requirements, put both segments into the vector we build the loops with

              if (considered.acceleration == 0){ // single constant speed segment
                results.add_segment(std::make_pair(considered_idxs[0].first, considered_idxs[0].second), std::make_pair(considered_speeds[0].first, considered_speeds[0].first), false, 0.0);

              } else { // we need to construct the segments!
                // acceleration/deceleration segment
                results.add_segment(std::make_pair(considered_idxs[0].first, corner_indxs[cur_corner_idx].first), std::make_pair(last_speed, considered_speeds[0].second), true, considered.acceleration);

                // constant speed segment
                results.add_segment(std::make_pair(corner_indxs[cur_corner_idx].first, corner_indxs[cur_corner_idx].second), std::make_pair(considered_speeds[0].second, considered_speeds[0].second), false, 0.0);
              }
            }
          } 
        } while (!meets_contingency_1);

        if (!meets_contingency_2) !works;

        if (!works){
          cross_type = crossType::no_cross;
        }
      }

      if (cross_type == crossType::no_cross){ // randomly generate segment
        results.all_clear();

        // perform calculation order
        // calculation order is to force a normal segment and generate randomly
        size_t prev_corner_end_loc;
        size_t cur_corner_start_loc, cur_corner_end_loc;
        double prev_corner_max_speed, cur_corner_max_speed;
        double cur_corner_proposed_speed;
        
        const double probability = 0.5;
        bool maintain_speed{false};

        int lower_bound_speed, upper_bound_speed;
        double speed_mean, speed_dev, cur_considered_speed;
        std::normal_distribution<double> speed_dist;
        std::mt19937 speed_gen(Config::get_instance()->get_speed_seed());

        std::uniform_real_distribution<double> acceleration_dist;
        std::mt19937 acc_gen(Config::get_instance()->get_speed_seed());

        double distance_to_next_corner;

        bool valid{false};

        prev_corner_end_loc = corner_indxs[last_corner_idx].second;
        prev_corner_max_speed = speed_corner_limits[last_corner_idx];

        // [1] Find a speed.
        // obtain upper and lower bound, and setup the gaussian
        if (!final_corner){
          cur_corner_start_loc = corner_indxs[0].first;
          cur_corner_end_loc = corner_indxs[0].second;
          cur_corner_max_speed = speed_corner_limits[0];

        } else {
          cur_corner_start_loc = corner_indxs[cur_corner_idx].first;
          cur_corner_end_loc = corner_indxs[cur_corner_idx].second;
          cur_corner_max_speed = speed_corner_limits[cur_corner_idx];
        }

        lower_bound_speed = std::max<int>(1, static_cast<int>(speed_corner_limits[cur_corner_idx] * corner_speed_min_ratio));
        upper_bound_speed = static_cast<int>(speed_corner_limits[cur_corner_idx] * corner_speed_max_ratio); 

        if (tgt_average_speed > upper_bound_speed){
          speed_mean = upper_bound_speed;
          speed_dev = speed_mean/6;
        } else {
          speed_mean = tgt_average_speed;
          speed_dev = speed_mean/6;
        }

        speed_dist = std::normal_distribution<double>(speed_mean, speed_dev);
        
        // keep sampling till we get a good solution
        while (!valid){
          if (last_speed >= tgt_average_speed && last_speed < cur_corner_max_speed){
            const double sample = skip_dist(skip_gen);
            if (sample < probability){
              cur_corner_proposed_speed = last_speed;
              maintain_speed = true;
            }
          }

          // obtain lower, upper bound speed estimates
          if (!maintain_speed){
            if (!final_corner){
              // sample the gaussian
              do {
                cur_corner_proposed_speed = speed_dist(speed_gen);
              } while (cur_corner_proposed_speed < lower_bound_speed || cur_corner_proposed_speed > upper_bound_speed);

            } else {
              // we must use the speed at the beginning of the loop
              cur_corner_proposed_speed = loop_segment_speeds[0].first;
            }
          }
          
          distance_to_next_corner = route->get_precomputed_distances().get_value(prev_corner_end_loc, cur_corner_start_loc);
          const std::pair<double, double> next_corner_speed_range =  {cur_corner_max_speed * corner_speed_min_ratio, cur_corner_max_speed*corner_speed_max_ratio};
          
          if (cur_corner_proposed_speed > last_speed){ // need to accelerate
            double acceleration_distance;
            double proposed_acceleration;
            size_t acceleration_end_idx;
            double considered_power;
            double lower_bound_acceleration = std::max(minimum_acceleration, calc_acceleration(last_speed, cur_corner_proposed_speed, distance_to_next_corner)), upper_bound_acceleration = lower_bound_acceleration;

            if (lower_bound_acceleration * mass * cur_corner_proposed_speed > power_allowance) continue;

            do {
              upper_bound_acceleration += 0.1;
              considered_power = upper_bound_acceleration * mass * cur_corner_proposed_speed;
            } while (considered_power < power_allowance);

            acceleration_dist = std::uniform_real_distribution<double>(lower_bound_acceleration, upper_bound_acceleration);
            proposed_acceleration = acceleration_dist(acc_gen);

            acceleration_distance = calc_distance_a(last_speed, cur_corner_proposed_speed, proposed_acceleration);

            acceleration_end_idx = prev_corner_end_loc;
            while (acceleration_distance > route->get_precomputed_distances().get_value(prev_corner_end_loc, acceleration_end_idx)){
              acceleration_end_idx = acceleration_end_idx == largest_index - 1 ? 0 : acceleration_end_idx + 1;
            }

            if (!can_reach_speeds(cur_corner_proposed_speed, power_allowance, max_acceleration, max_deceleration, next_corner_speed_range, distance_to_next_corner, mass)){
              continue;
            }

            // if we get here then we have our parameters, add to our loop variables
            results.add_segment(std::make_pair(prev_corner_end_loc, acceleration_end_idx), std::make_pair(last_speed, cur_corner_proposed_speed), true, proposed_acceleration);

            results.add_segment(std::make_pair(acceleration_end_idx, cur_corner_end_loc), std::make_pair(cur_corner_proposed_speed, cur_corner_proposed_speed), false, 0.0);

          } else if (cur_corner_proposed_speed < last_speed){ // need to decelerate

            // deceleration is not ideal, so sample chance to start over.
            if (last_speed < cur_corner_max_speed){
              const double sample = skip_dist(skip_gen);
              if (sample < 0.8) continue;
            }

            double deceleration_distance;
            double proposed_deceleration;
            size_t deceleration_end_idx;
            double considered_power;
            double lower_bound_deceleration = calc_acceleration(last_speed, cur_corner_proposed_speed, distance_to_next_corner);

            if (lower_bound_deceleration < max_deceleration) continue;

            if (!can_reach_speeds(cur_corner_proposed_speed, power_allowance, max_acceleration, max_deceleration, next_corner_speed_range, distance_to_next_corner, mass)){
              continue;
            }

            acceleration_dist = std::uniform_real_distribution<double>(lower_bound_deceleration, max_deceleration);
            proposed_deceleration = acceleration_dist(acc_gen);
            deceleration_distance = calc_distance_a(last_speed, cur_corner_proposed_speed, proposed_deceleration);

            deceleration_end_idx = prev_corner_end_loc;
            while (deceleration_distance > route->get_precomputed_distances().get_value(deceleration_end_idx, cur_corner_start_loc)){
              deceleration_end_idx = deceleration_end_idx == largest_index - 1 ? 0 : deceleration_end_idx + 1;
            }

            // if we get here then we have our parameters, add to our loop variables
            results.add_segment(std::make_pair(prev_corner_end_loc, deceleration_end_idx), std::make_pair(last_speed, cur_corner_proposed_speed), true, proposed_deceleration);

            results.add_segment(std::make_pair(deceleration_end_idx, cur_corner_end_loc), std::make_pair(cur_corner_proposed_speed, cur_corner_proposed_speed), false, 0.0);

          } else { // keep speed the same

            // add to our loop variables immediately
            results.add_segment(std::make_pair(prev_corner_end_loc, cur_corner_end_loc), std::make_pair(last_speed, last_speed), false, 0.0);
          }
        }
      }
      add_cross_segments_to_loop(results);
    }
  }

  // add loop to raceplan and clear loop
  for (size_t i=0; i<loops_per_block; i++){
    add_loop();
  } clear_loop();

  RacePlan ret(all_segments, all_segment_speeds, all_acceleration_segments, all_acceleration_values);
  return ret;
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
    if (!population[i].is_viable()) {
      population[i].set_score(-1.0);
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
      mutation_strategy(Config::get_instance()->get_mutation_strategy()),
      max_motor_power(kw2watts(Config::get_instance()->get_max_motor_power())),
      acceleration_power_budget(Config::get_instance()->get_acceleration_power_budget()),
      max_acceleration(Config::get_instance()->get_max_acceleration()),
      max_deceleration(Config::get_instance()->get_max_deceleration()) {}
