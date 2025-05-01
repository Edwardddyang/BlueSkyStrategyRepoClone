#include <random>
#include <stack>
#include <unordered_map>
#include <memory>
#include <unordered_set>
#include <string>
#include <vector>
#include <algorithm>
#include <utility>

#include "opt/GeneticUtilities.hpp"
#include "utils/Defines.hpp"
#include "config/Config.hpp"
#include "utils/Units.hpp"
#include "utils/Logger.hpp"

namespace Genetic {
RacePlan mutate_plan(RacePlan plan, const std::shared_ptr<Route> route) {
  RUNTIME_EXCEPTION(!plan.is_empty(), "RacePlan object to mutate cannot be empty");
  const std::vector<std::vector<std::pair<size_t, size_t>>> raw_segments = plan.get_orig_loop_segments();
  const std::vector<std::vector<std::pair<double, double>>> raw_speeds = plan.get_orig_loop_speeds();
  const std::vector<std::vector<bool>> raw_acceleration = plan.get_orig_loop_accelerations();
  const std::vector<std::vector<double>> raw_acceleration_value = plan.get_orig_loop_acceleration_values();
  const std::vector<std::vector<double>> raw_segment_distances = plan.get_orig_loop_segment_distances();
  std::unordered_map<size_t, size_t> corner_end_map = route->get_corner_end_map();
  std::unordered_map<size_t, size_t> corner_start_map = route->get_corner_start_map();

  if (Config::get_instance()->get_mutation_strategy() == "PreferConstantSpeed") {
    const size_t num_blocks = static_cast<int>(std::ceil(plan.get_num_loops() /
                                Config::get_instance()->get_num_repetitions()));
    RUNTIME_EXCEPTION(num_blocks == raw_segments.size(), "raw_segments vector shoud "
                      "have number of loops equal to the number of blocks {}", num_blocks);

    for (size_t block_idx=0; block_idx < num_blocks; block_idx++) {
      const size_t num_segments = raw_segments[block_idx].size();
      for (size_t seg_idx=0; seg_idx < num_segments; seg_idx++) {
        // Do nothing about segments that don't decelerate
        if (raw_acceleration_value[block_idx][seg_idx] >= 0.0) {
          continue;
        }
        const size_t seg_start = raw_segments[block_idx][seg_idx].first;
        size_t seg_end = raw_segments[block_idx][seg_idx].second;

        // We only bias towards constant speeds for "normal straights".
        // This means that the segment directly after this deceleration segment should include
        // the corner that this deceleration is for
        RUNTIME_EXCEPTION(seg_idx != num_segments - 1, "Deceleration cannot be the last segment");
        const size_t theoretical_corner_end = raw_segments[block_idx][seg_idx + 1].second;
        if (corner_end_map.find(theoretical_corner_end) == corner_end_map.end()) {
          // This is likely an aggressive straight. Don't do anything about these
          continue;
        }
        const size_t corner_idx = corner_end_map[theoretical_corner_end];
        const size_t corner_max_speed = route->get_cornering_speed_bounds()[corner_idx];
        // Sample a probability to do this at all?

        // Check if the starting speed of the deceleration is less than the maximum corner speed.
        // If it is, then we can remove the deceleration and maintain the constant speed
        if (corner_max_speed > raw_speeds[block_idx][seg_idx].first) {
          // Delete the deceleration segment and merge with the constant speed segment
        }
      }
    }
  }

  return RacePlan();
}
};  // namespace Genetic

template <typename T>
// Note that gen needs to be a reference since the internal state needs to be advanced after calling
static T bounded_gaussian(std::normal_distribution<> dist, std::mt19937& gen,  // NOLINT
                          double lower_bound, double upper_bound, const FileLogger& logger) {  // NOLINT
  double value;
  do {
    value = dist(gen);
  } while (value < lower_bound || value > upper_bound);

  return static_cast<T>(value);
}

bool RacePlanCreator::can_reach_speeds(double initial_speed, double acceleration_power, double max_acceleration,
                                       double max_deceleration, std::pair<double, double> speed_range,
                                       double max_distance, double car_mass) {
  RUNTIME_EXCEPTION(speed_range.first <= speed_range.second, "Speed range must be ordered as {smaller, bigger}");
  RUNTIME_EXCEPTION(initial_speed >= 0.0, "Initial speed must be >= 0 m/s");
  RUNTIME_EXCEPTION(max_distance >= 0.0, "Distance must be >= 0.0");
  RUNTIME_EXCEPTION(max_acceleration > 0.0 && max_deceleration < 0.0, "Maximum acceleration must be positive, "
                    "and maximum deceleration must be negative");

  if (initial_speed >= speed_range.first && initial_speed <= speed_range.second) {
    return true;
  }
  double distance = 0.0;
  double final_speed;
  double acceleration;
  double instataneous_motor_power = 0.0;
  if (initial_speed <= speed_range.first) {
    acceleration = 0.1;
    do {
      distance = calc_distance_a(initial_speed, speed_range.first, acceleration);
      instataneous_motor_power = speed_range.first * acceleration * car_mass;
      acceleration += 0.1;
      if (distance < max_distance && instataneous_motor_power < acceleration_power) return true;
    } while (instataneous_motor_power < acceleration_power && acceleration < max_acceleration);

    return false;
  }

  if (initial_speed > speed_range.second) {
    acceleration = -0.1;
    do {
      distance = calc_distance_a(initial_speed, speed_range.second, acceleration);
      acceleration -= 0.1;
      if (distance < max_distance) return true;
    } while (acceleration > max_deceleration);

    return false;
  }

  return false;
}

RacePlanCreator::RacePlanCreator(std::shared_ptr<Route> route,
                                unsigned speed_seed,
                                unsigned loop_seed,
                                unsigned aggressive_seed,
                                unsigned idx_seed,
                                unsigned acceleration_seed,
                                unsigned skip_seed) : route(route),
                                max_num_loops(Config::get_instance()->get_max_num_loops()),
                                fix_loops(Config::get_instance()->get_fix_num_loops()),
                                car_mass(Config::get_instance()->get_car_mass()),
                                max_speed(kph2mps(Config::get_instance()->get_max_route_speed())),
                                max_motor_power(kw2watts(Config::get_instance()->get_max_motor_power())),
                                max_acceleration(Config::get_instance()->get_max_acceleration()),
                                max_deceleration(Config::get_instance()->get_max_deceleration()),
                                min_acceleration(Config::get_instance()->get_min_acceleration()),
                                target_average_speed(Config::get_instance()->get_average_speed()),
                                start_time(Config::get_instance()->get_current_date_time()),
                                speed_seed(speed_seed), loop_seed(loop_seed), aggressive_seed(aggressive_seed),
                                idx_seed(idx_seed), acceleration_seed(acceleration_seed), skip_seed(skip_seed),
                                corner_speed_min_ratio(Config::get_instance()->get_corner_speed_min()),
                                corner_speed_max_ratio(Config::get_instance()->get_corner_speed_max()),
                                aggressive_straight_threshold(
                                  Config::get_instance()->get_aggressive_straight_threshold()),
                                num_repetitions(Config::get_instance()->get_num_repetitions()),
                                acceleration_power_budget(Config::get_instance()->get_acceleration_power_budget()),
                                max_iters(1000), log(Config::get_instance()->get_log_segmenting()) {
  const Time day_one_start_time = Config::get_instance()->get_day_one_start_time();
  const Time day_one_end_time = Config::get_instance()->get_day_one_end_time();
  const Time day_end_time = Config::get_instance()->get_day_end_time();  // Second and third day
  const bool is_first_day = start_time >= day_one_start_time && start_time < day_one_end_time;
  const Time race_plan_end_time = is_first_day ? day_one_end_time : day_end_time;

  end_time = race_plan_end_time;

  RUNTIME_EXCEPTION(route != nullptr, "Route object is null");

  // Extract information about the route
  route_points = route->get_route_points();
  cornering_segment_bounds = route->get_cornering_segment_bounds();
  cornering_speed_bounds = route->get_cornering_speed_bounds();
  route_distances = route->get_precomputed_distances();
  max_route_speed = route->get_max_route_speed();
  num_points = route->get_num_points();

  RUNTIME_EXCEPTION(route_points.size() > 0, "Route points not yet loaded");
  RUNTIME_EXCEPTION(cornering_segment_bounds.size() > 1, "There must exist at least 2 corners");
  RUNTIME_EXCEPTION(cornering_segment_bounds.size() == cornering_speed_bounds.size(),
                    "Number of corners must equal the length of the cornering speeds");
  RUNTIME_EXCEPTION(max_speed > 0.0 && max_acceleration > 0.0,
                    "Max. speed, and max. acceleration must be > 0");
  RUNTIME_EXCEPTION(max_deceleration < 0.0, "Max. deceleration must be negative");
  RUNTIME_EXCEPTION(max_motor_power > 0.0, "Maximum motor power must be > 0");
  RUNTIME_EXCEPTION(!route_distances.is_empty(), "Route distances must be pre-computed for acceleration segmenting");
  RUNTIME_EXCEPTION(corner_speed_min_ratio >= 0.0 && corner_speed_min_ratio <= 1.0,
                    "Corner speed minimum parameter must be in range [0.0, 1.0]");
  RUNTIME_EXCEPTION(corner_speed_max_ratio >= 0.0 && corner_speed_max_ratio <= 1.0,
                    "Corner speed maximum parameter must be in range [0.0, 1.0]");
  RUNTIME_EXCEPTION(corner_speed_min_ratio < corner_speed_max_ratio,
                    "Corner speed minimum must be strictly less than corner speed maximum");
  RUNTIME_EXCEPTION(aggressive_straight_threshold >= 0.0, "Aggressive straight threshold must be >= 0.0");
  RUNTIME_EXCEPTION(num_repetitions >= 1, "Num repetitions per loop block must >= 1");
  RUNTIME_EXCEPTION(start_time < end_time, "Start time must be before the end time");
  RUNTIME_EXCEPTION(min_acceleration > 0.0, "Minimum acceleration must be greater than 0");
}

RacePlan RacePlanCreator::create_plan() {
  // Create random number generators
  std::mt19937 speed_rng(speed_seed);
  std::mt19937 loop_rng(loop_seed);
  std::mt19937 aggressive_rng(aggressive_seed);
  std::mt19937 idx_rng(idx_seed);
  std::mt19937 acceleration_rng(acceleration_seed);
  std::mt19937 skip_rng(skip_seed);

  // Route attributes
  const size_t num_corners = cornering_segment_bounds.size();

  // Number of corners for which we need to create segments for
  const size_t num_corners_to_create = num_repetitions > 1 ? num_corners + 1 : num_corners;

  // Randomly select the number of loops to complete
  std::uniform_int_distribution<unsigned int> dis(1, max_num_loops);
  const size_t num_loops = fix_loops ? max_num_loops : static_cast<size_t>(dis(loop_rng));

  // Number of blocks to create
  size_t num_blocks;
  if (num_loops % num_repetitions == 0) {
    num_blocks = num_loops / num_repetitions;
  } else if (num_loops < num_repetitions) {
    num_blocks = 1;
  } else {
    num_blocks = num_loops / num_repetitions + 1;
  }

  const double acceleration_power_allowance = max_motor_power * acceleration_power_budget;

  FileLogger logger;
  logger = FileLogger("segment_route_corners.log", log);
  logger("Starting route segmentation with the following parameters:");
  logger("Loop seed: " + std::to_string(loop_seed));
  logger("Speed seed: " + std::to_string(speed_seed));
  logger("Aggressive straight sampling seed: " + std::to_string(aggressive_seed));
  logger("Index selection seed: " + std::to_string(idx_seed));
  logger("Acceleration seed: " + std::to_string(acceleration_seed));
  logger("Skip seed: " + std::to_string(skip_seed));
  logger("Maximum number of loops: " + std::to_string(max_num_loops));
  logger("Maximum car speed: " + std::to_string(max_speed) + " m/s");
  logger("Maximum acceleration: " + std::to_string(max_acceleration) + " m/s^2");
  logger("Maximum deceleration: " + std::to_string(max_deceleration) + " m/s^2");
  logger("Maximum motor power: " + std::to_string(max_motor_power) + " W");
  logger("Minimum acceleration to consider: " + std::to_string(min_acceleration) + "m/s^2");
  logger("Corner speed minimum clamp: " + std::to_string(corner_speed_min_ratio));
  logger("Corner speed maximum clamp: " + std::to_string(corner_speed_max_ratio));
  logger("Aggressive straight threshold: " + std::to_string(aggressive_straight_threshold) + " m");
  logger("Number of loop repetitions per chunk: " + std::to_string(num_repetitions));
  logger("Maximum iteration count for sampling: " + std::to_string(max_iters));
  logger("Fractional acceleration power budget: " + std::to_string(acceleration_power_budget));
  logger("Randomly selected number of loops to complete: " + std::to_string(num_loops));
  logger("Acceleration power allowance is " + std::to_string(acceleration_power_allowance) + " W\n");
  logger("Race Plan start time is " + start_time.get_local_readable_time());
  logger("Race Plan end time is " + end_time.get_local_readable_time());

  ///////////////////////////////////////////////////////////////////////////////////
  ////////////////////// NOTE ABOUT TERMINOLOGY USED ///////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////
  // "route index": Denotes an index in the route_points_array
  // "corner index": Denotes an index in the cornering_segment_bouds array
  // "real corner": A corner where the maximum cornering speed is less than the route's speed limit
  // "segment index": An index inside loop_segments, loop_segment_speeds ...
  // "wrap-around corner": The first corner of the route which the car wraps to after taking the entire route

  ///////////////////////////////////////////////////////////////////////////////////
  ///////////////////////// State variables for each corner /////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////
  // First corner of the first loop
  bool is_first_segment = true;

  // Whether a straight is taken aggressively
  bool aggressive_straight = false;

  // Whether a straight is taking normally
  bool normal_straight = false;

  // Attributes of the next and last corner
  double prev_corner_max_speed = 0.0;  // Maximum speed of the previous corner
  double next_corner_max_speed = 0.0;  // Maximum speed of the next speed
  size_t next_corner_start = 0;        // Starting route index of the next corner
  size_t prev_corner_end = 0;          // Ending route index of the previous corner

  // Stack of real corner speeds where the top of the stack is the most recent one
  std::stack<double> real_corner_speeds;
  real_corner_speeds.push(0.0);

  // Stack of real corner indices where the top of the stack is the most recent one
  std::stack<size_t> real_corner_indices;
  real_corner_indices.push(0);

  // Stack of # segments added for each real corner
  std::stack<uint64_t> num_segments_added;
  num_segments_added.push(0);
  uint64_t segment_counter = 0;

  // Stack of first corner speeds - pushed after constructing each new loop
  std::stack<uint64_t> first_corner_speeds;
  first_corner_speeds.push(0);

  // Stack holding the number of segments added to the last loop of the previous loop block
  // as a result of carrying over the wrap-around segments.
  std::stack<uint64_t> num_added_wrap_around_segments;
  num_added_wrap_around_segments.push(0);

  // The route index of the end of the last corner whose maximum speed is less than
  // the route speed
  size_t last_real_corner_end = 0;

  // The route index where a non acceleration zone starts on a long straight
  size_t non_acceleration_zone_start = 0;

  // The maximum distance the car can accelerate on a straight
  double max_acceleration_distance = 0.0;

  // The proposed corner speed
  int proposed_corner_speed = 0.0;

  // Distance to the next corner
  double distance_to_next_corner = 0.0;

  // General boolean variable for determining validity of a randomly chosen speed, acceleration, index etc.
  bool valid = false;

  // Counter for tracking the number of sampling iterations
  int count = 0;

  // Counter for the number of blocks
  size_t block_idx = 0;

  // Estimated instataneous motor power in W
  double instataneous_motor_power = 0.0;

  // Upper and lower bounds for speed
  int lower_bound_speed = 0;
  int upper_bound_speed = 0;

  // Mean and standard deviation to parameterize the normal distribution
  double speed_mean = target_average_speed;
  double speed_dev = 5.0;

  // Keep track of speeds that have been tested when sampling
  std::unordered_set<int> sampled_speeds;

  /** @brief Lambda to check if all speeds in the range of [lower_bound_speed, upper_bound_speed] have been tested */
  auto all_speeds_sampled = [&]() -> bool {
    int speed = lower_bound_speed;
    while (speed <= upper_bound_speed) {
      if (sampled_speeds.find(speed) == sampled_speeds.end()) {
        return false;
      }
      speed += 1;
    }
    return true;
  };

  ///////////////////////////////////////////////////////////////////////////////////
  IntermediateSegmentData seg_data;
  IntermediateLoopData loop_data;
  PlanAttributes att;

  auto get_loop_readable = [&]() -> std::string {
    return RacePlan::get_loop_string(loop_data.loop_segments, loop_data.loop_segment_speeds,
                                     loop_data.loop_acceleration_values, loop_data.loop_segment_distances);
  };

  // Probability distributions used to randomly select locations, speeds
  // and accelerations. Note that they are inclusive on both sides
  // TODO(Ethan): These should probably not be uniform distributions but at the very least,
  // gaussian
  std::uniform_int_distribution<size_t> idx_dist;
  std::normal_distribution<double> speed_dist;
  std::uniform_real_distribution<double> acceleration_dist(0.1, max_acceleration);
  std::uniform_real_distribution<double> aggressive_dist(0.0, 1.0);
  std::uniform_real_distribution<double> skip_dist(0.0, 1.0);

  size_t loop_idx = 0;
  while (block_idx < num_blocks) {
    /** @brief Lambda to add a new segment to the loop
    */
    auto add_segment = [&](IntermediateLoopData& loop_data,
                           const IntermediateSegmentData& seg_data) {
      loop_data.loop_segments.emplace_back(seg_data.segment);
      loop_data.loop_segment_speeds.emplace_back(seg_data.segment_speed);
      loop_data.loop_acceleration_segments.emplace_back(seg_data.acceleration);
      loop_data.loop_acceleration_values.emplace_back(seg_data.acceleration_value);
      loop_data.loop_segment_distances.emplace_back(seg_data.segment_distance);
      segment_counter = segment_counter + 1;
    };

    /** @brief Lambda to add the latest loop to the overall race plan 
     * Note: Will be added as vec[start_idx:end_idx] for each loop property
    */
    auto add_loop = [&](const size_t start_idx, const size_t end_idx) {
      RUNTIME_EXCEPTION(start_idx <= end_idx, "End index must be greater than Start index");
      RUNTIME_EXCEPTION(end_idx <= loop_data.loop_segments.size(), "Ending index must be less than or equal to the "
                                                                   "loop segments vector");
      // Store results for this loop
      std::vector<std::pair<size_t, size_t>> segments_slice(loop_data.loop_segments.begin() + start_idx,
                                                            loop_data.loop_segments.begin() + end_idx);
      att.all_segments.push_back(segments_slice);

      std::vector<std::pair<double, double>> segment_speeds_slice(loop_data.loop_segment_speeds.begin() + start_idx,
                                                                  loop_data.loop_segment_speeds.begin() + end_idx);
      att.all_segment_speeds.push_back(segment_speeds_slice);

      std::vector<bool> loop_acceleration_slice(loop_data.loop_acceleration_segments.begin() + start_idx,
                                                loop_data.loop_acceleration_segments.begin() + end_idx);
      att.all_acceleration_segments.push_back(loop_acceleration_slice);

      std::vector<double> loop_acceleration_v_slice(loop_data.loop_acceleration_values.begin() + start_idx,
                                                    loop_data.loop_acceleration_values.begin() + end_idx);
      att.all_acceleration_values.push_back(loop_acceleration_v_slice);

      std::vector<double> loop_segment_distances_slice(loop_data.loop_segment_distances.begin() + start_idx,
                                                       loop_data.loop_segment_distances.begin() + end_idx);
      att.all_segment_distances.push_back(loop_segment_distances_slice);
    };

    /** @brief Create loop block with the newly created loop */
    auto create_loop_block = [&]() -> size_t {
      const size_t num_loops_in_block = std::min(static_cast<size_t>(num_repetitions), num_loops - loop_idx);

      logger("Creating loop block " + std::to_string(block_idx) + " with " +
              std::to_string(num_loops_in_block) + " loops");

      logger("\n////CREATED LOOP////");
      logger(get_loop_readable());

      // --------------- Create the first loop of the block ---------------
      logger("Creating loop 1");
      // For loops in a block greater than the first one, they will begin with segments that travel from
      // the last corner of the route to the first corner. Add these segments to the last loop of the previous block.
      // For example, the beginning of a typical loop past the first block looks something like this:
      // +-----------+---------------+-----------------------+----------------
      // ; Segments  ; Speeds (m/s)  ; Acceleration (m/s^2)  ; Distances (m) ;
      // +-----------+---------------+-----------------------+----------------
      // | [573,585] |     [  6, 12] |                  0.500|          117.4|
      // | [585,591] |     [ 12, 12] |                      0|          56.05|
      // | [591, 19] |     [ 12,  7] |                  -0.10|          469.7|
      // | [ 19, 54] |     [  7,  7] |                      0|          261.1|
      // | [ 54, 58] |     [  7,  4] |                     -1|          25.69|
      // FIGURE A

      // Recall that for each corner, we create the segments coming from the last corner to the current
      // corner. In this case, we want to place [573,585],[585,591] at the end of the last loop in the
      // previous block. For crossover segments e.g. [591, 19], we break them up into two sub-segments.
      // The first will travel from the segment beginning to index 0 and the second will travel from index 0
      // to the end of the original segment

      // For the very first block, these segments will not exist, since we're starting at index 0
      num_added_wrap_around_segments.push(0);
      while (block_idx != 0 && loop_data.loop_segments[0].second != 0 &&
            loop_data.loop_segments[0].first < loop_data.loop_segments[0].second) {
        att.all_segments.back().push_back(loop_data.loop_segments[0]);
        att.all_segment_speeds.back().push_back(loop_data.loop_segment_speeds[0]);
        att.all_acceleration_segments.back().push_back(loop_data.loop_acceleration_segments[0]);
        att.all_acceleration_values.back().push_back(loop_data.loop_acceleration_values[0]);
        att.all_segment_distances.back().push_back(loop_data.loop_segment_distances[0]);
        num_added_wrap_around_segments.top() = num_added_wrap_around_segments.top() + 1;

        logger("Moving segment [" + std::to_string(loop_data.loop_segments[0].first) + "," +
              std::to_string(loop_data.loop_segments[0].second) + "] to the previous loop");

        // Remove the first element of loop vectors - this is why we don't have to keep track of an index variable
        loop_data.loop_segments.erase(loop_data.loop_segments.begin());
        loop_data.loop_segment_speeds.erase(loop_data.loop_segment_speeds.begin());
        loop_data.loop_acceleration_segments.erase(loop_data.loop_acceleration_segments.begin());
        loop_data.loop_acceleration_values.erase(loop_data.loop_acceleration_values.begin());
        loop_data.loop_segment_distances.erase(loop_data.loop_segment_distances.begin());
      }

      // Crossover segment detected. Break into two sub-segments
      if (loop_data.loop_segments[0].first > loop_data.loop_segments[0].second) {
        std::pair<size_t, size_t> first_segment = {loop_data.loop_segments[0].first, 0};
        const double first_segment_distance = route_distances.get_value(first_segment.first, 0);
        const double first_segment_ending_speed = calc_final_speed_a(loop_data.loop_segment_speeds[0].first,
                                                                     loop_data.loop_acceleration_values[0],
                                                                     first_segment_distance);
        std::pair<double, double> first_segment_speeds = {loop_data.loop_segment_speeds[0].first,
                                                          first_segment_ending_speed};

        att.all_segments.back().push_back(first_segment);
        att.all_segment_speeds.back().push_back(first_segment_speeds);
        att.all_acceleration_segments.back().push_back(loop_data.loop_acceleration_segments[0]);
        att.all_acceleration_values.back().push_back(loop_data.loop_acceleration_values[0]);
        att.all_segment_distances.back().push_back(first_segment_distance);

        std::pair<size_t, size_t> second_segment = {0, loop_data.loop_segments[0].second};
        const double second_segment_distance = route_distances.get_value(0, second_segment.second);
        const double second_segment_ending_speed = loop_data.loop_segment_speeds[0].second;
        std::pair<double, double> second_segment_speeds = {first_segment_ending_speed,
                                                          second_segment_ending_speed};
        loop_data.loop_segments[0] = second_segment;
        loop_data.loop_segment_speeds[0] = second_segment_speeds;
        loop_data.loop_segment_distances[0] = second_segment_distance;
      }

      // For num_repetitions > 1, there will be a wrap-around corner. Therefore, the end of the loop
      // will look something like this:
      // ...
      // | [560,573] |     [  6,  6] |                      0|          112.9|
      // | [573,574] |     [  6,  7] |                  1.552|          8.895|
      // | [574,615] |     [  7,  7] |                      0|          393.5|
      // | [615, 19] |     [  7,  2] |                  -0.09|          240.7|
      // | [ 19, 54] |     [  2,  2] |                      0|          261.1|
      // FIGURE B

      // We want to cut off at index 0, meaning we remove the [19,54] segment and break the crossover segment
      // into two sub-segments as in the above
      // Record the wrap around segments which needed to be added to the beginning of the loop later
      std::vector<std::pair<size_t, size_t>> wrap_around_segments;
      std::vector<std::pair<double, double>> wrap_around_speeds;
      std::vector<bool> wrap_around_acceleration;
      std::vector<double> wrap_around_acceleration_values;
      std::vector<double> wrap_around_distances;
      while (num_repetitions > 1 && loop_data.loop_segments.back().second > loop_data.loop_segments.back().first &&
            loop_data.loop_segments.back().second != 0) {
        wrap_around_segments.insert(wrap_around_segments.begin(), loop_data.loop_segments.back());
        wrap_around_speeds.insert(wrap_around_speeds.begin(), loop_data.loop_segment_speeds.back());
        wrap_around_acceleration.insert(wrap_around_acceleration.begin(),
                                        loop_data.loop_acceleration_segments.back());
        wrap_around_acceleration_values.insert(wrap_around_acceleration_values.begin(),
                                               loop_data.loop_acceleration_values.back());
        wrap_around_distances.insert(wrap_around_distances.begin(),
                                     loop_data.loop_segment_distances.back());

        loop_data.loop_segments.pop_back();
        loop_data.loop_segment_speeds.pop_back();
        loop_data.loop_acceleration_segments.pop_back();
        loop_data.loop_acceleration_values.pop_back();
        loop_data.loop_segment_distances.pop_back();
      }

      // Crossover segment detected on last segment. We will only insert the first sub-segment into
      // the current loop, but we also need to preserve the second segment when we create the loops > 1st one
      // of the block
      if (loop_data.loop_segments.back().second < loop_data.loop_segments.back().first) {
        std::pair<size_t, size_t> first_segment = {loop_data.loop_segments.back().first, 0};
        const double first_segment_distance = route_distances.get_value(first_segment.first, 0);
        const double first_segment_ending_speed = calc_final_speed_a(loop_data.loop_segment_speeds.back().first,
                                                                     loop_data.loop_acceleration_values.back(),
                                                                     first_segment_distance);
        std::pair<double, double> first_segment_speeds = {loop_data.loop_segment_speeds.back().first,
                                                          first_segment_ending_speed};
        const std::pair<size_t, size_t> second_segment = {0, loop_data.loop_segments.back().second};
        const double second_segment_distance = route_distances.get_value(0, second_segment.second);
        const double second_segment_ending_speed = loop_data.loop_segment_speeds.back().second;
        const std::pair<double, double> second_segment_speeds = {first_segment_ending_speed,
                                                                  second_segment_ending_speed};

        wrap_around_segments.insert(wrap_around_segments.begin(), second_segment);
        wrap_around_speeds.insert(wrap_around_speeds.begin(), second_segment_speeds);
        wrap_around_acceleration.insert(wrap_around_acceleration.begin(), loop_data.loop_acceleration_segments.back());
        wrap_around_acceleration_values.insert(wrap_around_acceleration_values.begin(),
                                               loop_data.loop_acceleration_values.back());
        wrap_around_distances.insert(wrap_around_distances.begin(), second_segment_distance);

        loop_data.loop_segments.back() = first_segment;
        loop_data.loop_segment_speeds.back() = first_segment_speeds;
        loop_data.loop_segment_distances.back() = first_segment_distance;
      }
      // Add the first loop
      if (num_loops_in_block > 1) {
        logger("Loop 1 Modified:");
        logger(get_loop_readable());
        add_loop(0, loop_data.loop_segments.size());
      }
      // --------------------------------------------------------------------

      // --------------- Create the loops after the first one ---------------
      // This is only applicable if the number of loops in a block is greater than 1
      if (num_loops_in_block > 1) {
        logger("Modifying loop construct for the second... loop in the block");
        // Refer to figure A, we first want to delete the segments that extend up to and including
        // the first corner as they will be replaced by the wrap-around segment
        while (loop_data.loop_segments[0].second <= cornering_segment_bounds[0].second) {
          loop_data.loop_segments.erase(loop_data.loop_segments.begin());
          loop_data.loop_segment_speeds.erase(loop_data.loop_segment_speeds.begin());
          loop_data.loop_acceleration_segments.erase(loop_data.loop_acceleration_segments.begin());
          loop_data.loop_acceleration_values.erase(loop_data.loop_acceleration_values.begin());
          loop_data.loop_segment_distances.erase(loop_data.loop_segment_distances.begin());
        }

        // Move the recorded wrap around segments to the front
        loop_data.loop_segments.insert(loop_data.loop_segments.begin(), wrap_around_segments.begin(),
                                       wrap_around_segments.end());
        loop_data.loop_segment_speeds.insert(loop_data.loop_segment_speeds.begin(), wrap_around_speeds.begin(),
                                             wrap_around_speeds.end());
        loop_data.loop_acceleration_segments.insert(loop_data.loop_acceleration_segments.begin(),
                                                    wrap_around_acceleration.begin(),
                                                    wrap_around_acceleration.end());
        loop_data.loop_acceleration_values.insert(loop_data.loop_acceleration_values.begin(),
                                                  wrap_around_acceleration_values.begin(),
                                                  wrap_around_acceleration_values.end());
        loop_data.loop_segment_distances.insert(loop_data.loop_segment_distances.begin(),
                                                wrap_around_distances.begin(), wrap_around_distances.end());

        for (size_t i=1; i < num_loops_in_block - 1; i++) {
          add_loop(0, loop_data.loop_segments.size());
        }
        logger("LOOP X MODIFIED");
        logger(get_loop_readable());
      }

      // --------------------------------------------------------------------

      // --------------- Add the last loop of the block ---------------
      size_t last_corner_ending_idx = 0;
      for (; last_corner_ending_idx < loop_data.loop_segments.size(); last_corner_ending_idx++) {
        if (loop_data.loop_segments[last_corner_ending_idx].second == cornering_segment_bounds.back().second) {
          break;
        }
      }
      if (block_idx == num_blocks - 1) {
        // Remove all the segments past the last corner and create a constant segment until index 0
        logger("Last corner ending idx " + std::to_string(last_corner_ending_idx));
        logger("Loop segments size: " + std::to_string(loop_data.loop_segments.size()));
        const size_t num_segments_to_remove = loop_data.loop_segments.size() - last_corner_ending_idx;
        logger("Num segments to remove: " + std::to_string(num_segments_to_remove));

        loop_data.loop_segments.erase(loop_data.loop_segments.end() - num_segments_to_remove,
                                      loop_data.loop_segments.end());
        loop_data.loop_segment_speeds.erase(loop_data.loop_segment_speeds.end() - num_segments_to_remove,
                                            loop_data.loop_segment_speeds.end());
        loop_data.loop_acceleration_segments.erase(loop_data.loop_acceleration_segments.end() - num_segments_to_remove,
                                                   loop_data.loop_acceleration_segments.end());
        loop_data.loop_acceleration_values.erase(loop_data.loop_acceleration_values.end() - num_segments_to_remove,
                                                 loop_data.loop_acceleration_values.end());
        loop_data.loop_segment_distances.erase(loop_data.loop_segment_distances.end() - num_segments_to_remove,
                                               loop_data.loop_segment_distances.end());

        const size_t corner_end_idx = loop_data.loop_segments.back().second;
        const int corner_speed = loop_data.loop_segment_speeds.back().second;

        seg_data.segment = {corner_end_idx, 0};
        seg_data.segment_speed = {corner_speed, corner_speed};
        seg_data.acceleration_value = 0.0;
        seg_data.acceleration = false;
        logger(std::to_string(seg_data.segment.first) + ", " + std::to_string(seg_data.segment.second));
        seg_data.segment_distance = route_distances.get_value(seg_data.segment.first, seg_data.segment.second);

        add_segment(loop_data, seg_data);
        add_loop(0, loop_data.loop_segments.size());
      } else {
        logger("Last corner ending index: " + std::to_string(last_corner_ending_idx));
        logger("Block counter: " + std::to_string(block_idx));
        // If this is the last loop of the block, then we stop at the end of the last corner
        // since we want the next loop to decide how to travel from the last corner to the first
        add_loop(0, last_corner_ending_idx + 1);
      }
      return num_loops_in_block;
    };

    // Clear all temp data holders
    loop_data.clear();

    // Labmda to remove the latest corner segments added to the loop
    auto remove_last_corner_segments = [&]() {
      const size_t curr_loop_num_segments = loop_data.loop_segments.size();
      const size_t num_segments_of_last_corner = static_cast<size_t>(num_segments_added.top());
      logger("Number of segments in current loop: " + std::to_string(curr_loop_num_segments));
      const size_t num_remove_from_curr_loop = std::min(curr_loop_num_segments, num_segments_of_last_corner);
      if (num_segments_of_last_corner == 0) {
        return;
      }
      logger("Removing " + std::to_string(num_segments_of_last_corner) + " segments");

      for (size_t idx = 0; idx < num_remove_from_curr_loop; idx++) {
        loop_data.loop_segments.pop_back();
        loop_data.loop_segment_speeds.pop_back();
        loop_data.loop_acceleration_segments.pop_back();
        loop_data.loop_acceleration_values.pop_back();
        loop_data.loop_segment_distances.pop_back();
      }

      const size_t num_remove_from_last_loop = num_segments_of_last_corner > curr_loop_num_segments ?
                                               num_segments_of_last_corner - curr_loop_num_segments : 0;
      logger("Removed " + std::to_string(num_remove_from_curr_loop) + " segments from current loop and " +
             std::to_string(num_remove_from_last_loop) + " from last loop");
      if (att.all_segments.size() > 0 && num_remove_from_last_loop > 0) {
        RUNTIME_EXCEPTION(num_remove_from_last_loop <= att.raw_loop_segments.back().size(),
                          "Last loop has too few segments");
      }
      for (size_t idx = 0; idx < num_remove_from_last_loop; idx++) {
        att.raw_loop_segments.back().pop_back();
        att.raw_loop_speeds.back().pop_back();
        att.raw_acceleration_segments.back().pop_back();
        att.raw_acceleration_values.back().pop_back();
        att.raw_loop_distances.back().pop_back();
      }
    };

    // Lambda to convert segment information into a string (debugging + logging purposes)
    auto get_segment_string = [&]() {
      std::stringstream ss;
      ss << "Segment: [" << seg_data.segment.first << "," << seg_data.segment.second << "]\n";
      ss << "Segment Distance: " << seg_data.segment_distance << "m\n";
      ss << "Segment Speeds: [" << seg_data.segment_speed.first << "," << seg_data.segment_speed.second << "]\n";
      ss << "Acceleration: " << (seg_data.acceleration ? "True\n" : "False\n");
      ss << "Acceleration Value: " << seg_data.acceleration_value << "\n";
      return ss.str();
    };

    // We construct the segments between the previous corner to the current corner.
    // We consider a complete loop as one that wraps back around to the first corner
    // of the route.
    size_t corner_idx = 0;
    while (corner_idx < num_corners_to_create) {
      segment_counter = 0;
      auto create_segments = [&]() -> bool {
        // Wrap-around corner is index 0
        logger("--------------CREATING SEGMENTS FOR CORNER " + std::to_string(corner_idx) +
              " OF LOOP BLOCK " + std::to_string(block_idx), false);
        if (corner_idx == num_corners) {
          logger(" [WRAP-AROUND CORNER]--------------");
        } else {
          logger("--------------");
        }
        // Get attributes for current corner
        const size_t corner_start = cornering_segment_bounds[corner_idx % num_corners].first;
        const size_t corner_end = cornering_segment_bounds[corner_idx % num_corners].second;
        const double max_corner_speed = cornering_speed_bounds[corner_idx % num_corners];
        logger("Corner maximum speed is " + std::to_string(max_corner_speed) + "m/s");
        logger("Corner start is index " + std::to_string(corner_start));
        logger("Corner end is index " + std::to_string(corner_end));

        // If the maximum cornering speed is greater than the route speed, then we can treat this as a
        // no acceleration zone and move onto the next corner
        if (max_corner_speed > max_route_speed) {
          logger("This corner has a speed greater than the route speed. "
                "Skip segment generation and treat as no-acceleration zone\n");
          return true;
        }

        // Get attributes for previous corner
        if (corner_idx > 0) {  // Note this includes the wrap-around corner e.g. corner_idx = num_corners
          prev_corner_end = cornering_segment_bounds[corner_idx-1].second;
          prev_corner_max_speed = cornering_speed_bounds[corner_idx-1];
        } else {  // Last corner
          prev_corner_end = cornering_segment_bounds[num_corners-1].second;
          prev_corner_max_speed = cornering_speed_bounds[num_corners-1];
        }

        // Get attributes for next corner
        if (corner_idx < num_corners - 1) {
          next_corner_start = cornering_segment_bounds[corner_idx+1].first;
          next_corner_max_speed = cornering_speed_bounds[corner_idx+1];
        } else if (corner_idx == num_corners) {  // First corner
          next_corner_start = cornering_segment_bounds[1].first;
          next_corner_max_speed = cornering_speed_bounds[1];
        } else {
          next_corner_start = cornering_segment_bounds[0].first;
          next_corner_max_speed = cornering_speed_bounds[0];
        }

        // Get the length of the straight between the last real corner and the current corner
        const size_t last_real_corner_idx = real_corner_indices.top();
        const double last_real_corner_speed = real_corner_speeds.top();
        last_real_corner_end = is_first_segment ? 0 : cornering_segment_bounds[last_real_corner_idx].second;
        logger("Last real corner index was " + std::to_string(last_real_corner_idx));
        logger("Last real corner speed was " + std::to_string(last_real_corner_speed));

        const double straight_distance = route_distances.get_value(last_real_corner_end, corner_start);
        logger("Straight distance between index " + std::to_string(last_real_corner_end) + " and " +
              std::to_string(corner_start) + " is " + std::to_string(straight_distance) + "m");

        // In the case where the straight is composed of a non-acceleration zone (a corner with high speed limit),
        // then there is a constraint to complete the acceleration prior to the beginning of the non-acceleration
        // zone
        bool straight_includes_corners = !is_first_segment && last_real_corner_idx != corner_idx - 1 &&
                                        !(last_real_corner_idx == num_corners - 1 && corner_idx == 0);
        if (straight_includes_corners) {
          non_acceleration_zone_start = cornering_segment_bounds[last_real_corner_idx+1].first;
          max_acceleration_distance = route_distances.get_value(last_real_corner_end, non_acceleration_zone_start);
          logger("The straight includes corners with maximum speeds less than the route speed");
          logger("Maximum acceleration distance is: " + std::to_string(max_acceleration_distance) +
                " from index " + std::to_string(last_real_corner_end) + " to index " +
                std::to_string(non_acceleration_zone_start));
        } else {
          non_acceleration_zone_start = corner_start;
          // Spending a quarter the straight accelerating seems wasteful. Maybe change later
          max_acceleration_distance = straight_distance * 0.4;
          logger("Maximum acceleration distance is: " + std::to_string(max_acceleration_distance) +
                " which is 4/10 the distance from index " + std::to_string(last_real_corner_end) + " to index " +
                std::to_string(corner_start));
        }

        // If the straight is long, sample a chance to take the straight aggressively
        double probability = 0.0;
        if (straight_distance > aggressive_straight_threshold) {
          // If the straight distance >= 2 * aggressive straight threshold, then we are guaranteed to take
          // the straight aggressively
          probability = std::min(1.0, (straight_distance - aggressive_straight_threshold)
                                      / aggressive_straight_threshold);
        }
        const double sample = aggressive_dist(aggressive_rng);
        if (sample < probability) {
          normal_straight = false;
          // Parameters to search for
          double proposed_deceleration;
          double proposed_acceleration;
          double acceleration_end_speed;
          double acceleration_distance;

          // Route index of the ending point for acceleration. This is selected
          // by sampling an ending speed for the acceleration
          size_t acceleration_end_idx;

          // Route index of the starting point for deceleration. This must be in the range
          // of [last_real_corner_end, corner_start-1]
          size_t deceleration_start_idx;

          // Aggressive straight
          if (is_first_segment) {
            logger("Taking straight between starting line and " +
                    std::to_string(corner_idx) + " aggressively. Straight distance is " +
                    std::to_string(straight_distance) + "m");
          } else {
            logger("Taking straight between corner " + std::to_string(last_real_corner_idx) + " and " +
                    std::to_string(corner_idx) + " aggressively. Straight distance is " +
                    std::to_string(straight_distance) + "m");
          }
          logger("Last real corner speed: " + std::to_string(last_real_corner_speed) + "m/s");

          acceleration_dist = std::uniform_real_distribution<double>(0.0, max_acceleration);

          // Loop until we get valid parameters or we reach the maximum number of iterations
          valid = false;
          count = 0;
          while (!valid) {
            count++;
            if (count == max_iters) {
              return false;
            }

            if (min_acceleration * car_mass * last_real_corner_speed > acceleration_power_allowance) {
              logger("Last real corner speed is too high for the motor power to even sustain 0.1m/s^2. "
                     "Using normal straight");
              normal_straight = true;
              break;
            }

            double upper_bound_speed = last_real_corner_speed;
            double lower_bound_acceleration = 0.0;
            do {
              upper_bound_speed += 0.1;
              lower_bound_acceleration = calc_acceleration(last_real_corner_speed, upper_bound_speed,
                                                           max_acceleration_distance);
              instataneous_motor_power = lower_bound_acceleration * car_mass * upper_bound_speed;
            } while (instataneous_motor_power < acceleration_power_allowance);
            upper_bound_speed = upper_bound_speed - 0.1;

            if (static_cast<int>(upper_bound_speed) <= last_real_corner_speed + 1) {
              // If the upper bound speed is the same as the corner speed, then that means
              // the maximum acceleration distance is too low or the last corner speed was too high
              // for the motor
              normal_straight = true;
              logger("Acceleration Distance: " + std::to_string(acceleration_distance));
              logger("Motor Power: " + std::to_string(min_acceleration * car_mass * upper_bound_speed));
              logger("Upper bound speed is too close to the last real corner speed. Using normal straight");
              break;
            }

            if (target_average_speed > upper_bound_speed) {
              speed_mean = upper_bound_speed;
              speed_dev = speed_mean / 8.0;
            } else {
              speed_mean = target_average_speed;
              speed_dev = target_average_speed / 6.0;
            }

            speed_dist = std::normal_distribution<double>(speed_mean, speed_dev);
            logger("Created acceleration ending speed gaussian distribution with lower bound " +
                   std::to_string(last_real_corner_speed + 1) + ", upper bound " +
                   std::to_string(static_cast<int>(upper_bound_speed)) + ", mean of " +
                   std::to_string(speed_mean) + ", deviation of " + std::to_string(speed_dev));
            acceleration_end_speed = bounded_gaussian<int>(speed_dist, speed_rng, last_real_corner_speed + 1,
                                                           upper_bound_speed, logger);
            logger("Trying acceleration ending speed of " + std::to_string(acceleration_end_speed) + "m/s");

            // Find lower upper bound acceleration that can support this ending speed
            lower_bound_acceleration = calc_acceleration(last_real_corner_speed, upper_bound_speed,
                                                          max_acceleration_distance);
            double upper_bound_acceleration = lower_bound_acceleration;

            do {
              upper_bound_acceleration = upper_bound_acceleration+ 0.1;
              instataneous_motor_power = upper_bound_acceleration * car_mass * acceleration_end_speed;
            } while (instataneous_motor_power < acceleration_power_allowance);
            upper_bound_acceleration = upper_bound_acceleration - 0.1;
            acceleration_dist = std::uniform_real_distribution<double>(lower_bound_acceleration,
                                                                       upper_bound_acceleration);
            if (lower_bound_acceleration < upper_bound_acceleration) {
              logger("Created acceleration distribution with lower bound " + std::to_string(lower_bound_acceleration) +
                     "m/s^2 and upper bound " + std::to_string(upper_bound_acceleration) + "m/s^2");
              proposed_acceleration = acceleration_dist(acceleration_rng);
            } else if (lower_bound_acceleration == upper_bound_acceleration) {
              proposed_acceleration = lower_bound_acceleration;
            } else {
              logger("Somehow, the lower bound acceleration > upper bound");
              normal_straight = true;
              break;
            }
            acceleration_distance = calc_distance_a(last_real_corner_speed, acceleration_end_speed,
                                                    proposed_acceleration);
            logger("Trying acceleration " + std::to_string(proposed_acceleration) + " with distance " +
                    std::to_string(acceleration_distance));
            RUNTIME_EXCEPTION(acceleration_distance < max_acceleration_distance, "acceleration distance "
                              "should be lower than max acceleration distance");

            // Ensure that the acceleration power budget is not exceeded
            instataneous_motor_power = proposed_acceleration * car_mass * last_real_corner_speed;
            if (instataneous_motor_power > acceleration_power_allowance) {
              logger(std::to_string(instataneous_motor_power) + " W exceeds maximum acceleration power budget. "
                    "Trying again");
              continue;
            }

            // Locate the smallest route index such that [last_real_corner_end, idx] is greater
            // than the acceleration distance
            acceleration_end_idx = last_real_corner_end;
            while (route_distances.get_value(last_real_corner_end, acceleration_end_idx) < acceleration_distance) {
              acceleration_end_idx = acceleration_end_idx == num_points - 1 ? 0 : acceleration_end_idx + 1;
            }

            // Select corner speed - if wrap-around, then the corner speed is fixed further below
            // We create a distribution that is biased by the desired average speed for the route
            lower_bound_speed = std::max<int>(1, static_cast<int>(max_corner_speed * corner_speed_min_ratio));
            upper_bound_speed = static_cast<int>(max_corner_speed * corner_speed_max_ratio);

            if (target_average_speed > upper_bound_speed) {
              speed_mean = (upper_bound_speed + lower_bound_speed) / 2.0;
              speed_dev = speed_mean / 6.0;
            } else {
              speed_mean = target_average_speed;
              speed_dev = target_average_speed / 6.0;
            }
            speed_dist = std::normal_distribution<double>(speed_mean, speed_dev);
            logger("Created corner speed gaussian distribution with lower bound " +
                   std::to_string(lower_bound_speed) + "m/s, upper bound " +
                   std::to_string(upper_bound_speed) + "m/s, mean of " +
                   std::to_string(speed_mean) + ", and deviation of " +
                   std::to_string(speed_dev));

            // Loop until we find a valid corner speed
            int count2 = 0;
            bool valid_corner_speed = false;
            while (!valid_corner_speed) {
              count2++;
              if (count2 == max_iters) {
                return false;
              }

              if (corner_idx == num_corners) {
                proposed_corner_speed = first_corner_speeds.top();
              } else {
                proposed_corner_speed = bounded_gaussian<int>(speed_dist, speed_rng, lower_bound_speed,
                                                              upper_bound_speed, logger);
              }
              logger("Trying corner speed " + std::to_string(proposed_corner_speed) + "m/s");

              // Select a location to start decelerating
              if (corner_start - 1 < acceleration_end_idx) {
                // Crossover from one loop to the next
                idx_dist = std::uniform_int_distribution<size_t>(acceleration_end_idx + 1,
                                                                num_points + corner_start - 1);
                logger("Created deceleration index distribution with lower bound " +
                      std::to_string(acceleration_end_idx + 1) +
                      " and upper bound " + std::to_string(num_points + corner_start - 1) +
                      ". This is crossover from one loop to the next");
              } else {
                idx_dist = std::uniform_int_distribution<size_t>(acceleration_end_idx + 1, corner_start - 1);
                logger("Created deceleration index distribution with lower bound " +
                      std::to_string(acceleration_end_idx + 1) +
                      " and upper bound " + std::to_string(corner_start-1));
              }
              deceleration_start_idx = idx_dist(idx_rng);
              if (deceleration_start_idx >= num_points) {
                deceleration_start_idx -= num_points;
              }

              logger("Trying deceleration starting index of " + std::to_string(deceleration_start_idx));
              const double distance_to_corner = route_distances.get_value(deceleration_start_idx, corner_start);
              logger("Deceleration distance is " + std::to_string(distance_to_corner) + "m");
              proposed_deceleration = calc_acceleration(acceleration_end_speed,
                                                        proposed_corner_speed,
                                                        distance_to_corner);
              logger("Proposed deceleration is " + std::to_string(proposed_deceleration) + "m/s^2");
              // Deceleration could be positive if the selected corner speed is greater than
              // the acceleration ending speed.
              if (proposed_deceleration > 0.0 && proposed_deceleration < max_acceleration &&
                  proposed_deceleration * car_mass * proposed_corner_speed <= acceleration_power_allowance) {
                valid_corner_speed = true;
              } else if (proposed_deceleration < 0.0 && proposed_deceleration > max_deceleration) {
                valid_corner_speed = true;
              } else {
                logger("Necessary deceleration to reach next corner exceeds maximum acceleration/deceleration "
                       "or exceeds maximum acceleration power allowance");
              }

              // Check that the selected corner speed allows the car to reach the next corner's range of speeds in half
              // the straight distance
              distance_to_next_corner = route_distances.get_value(corner_end, next_corner_start);
              logger("Distance to next corner is " + std::to_string(distance_to_next_corner));
              const std::pair<double, double> next_corner_range = {next_corner_max_speed * corner_speed_min_ratio,
                                                                  next_corner_max_speed * corner_speed_max_ratio};
              if (!can_reach_speeds(proposed_corner_speed, acceleration_power_allowance, max_acceleration,
                                    max_deceleration, next_corner_range, distance_to_next_corner, car_mass)) {
                logger("Corner speed is too high to reach the next corner");
                valid_corner_speed = false;
                continue;
              }
            }

            if (!normal_straight) {
              logger("Aggressive straight with valid parameters were found");

              // Add acceleration segment
              seg_data.segment = {last_real_corner_end, acceleration_end_idx};
              seg_data.segment_distance = route_distances.get_value(last_real_corner_end, acceleration_end_idx);
              seg_data.acceleration_value = proposed_acceleration;
              seg_data.segment_speed = {last_real_corner_speed, acceleration_end_speed};
              seg_data.acceleration = true;
              add_segment(loop_data, seg_data);

              logger("\nACCELERATION SEGMENT");
              logger(get_segment_string());

              // Add constant speed segment
              seg_data.segment = {acceleration_end_idx, deceleration_start_idx};
              seg_data.segment_distance = route_distances.get_value(acceleration_end_idx, deceleration_start_idx);
              seg_data.acceleration_value = 0.0;
              seg_data.acceleration = false;
              seg_data.segment_speed = {acceleration_end_speed, acceleration_end_speed};
              add_segment(loop_data, seg_data);

              logger("\nCONSTANT SPEED SEGMENT");
              logger(get_segment_string());

              // Add deceleration speed segment
              seg_data.segment = {deceleration_start_idx, corner_start};
              seg_data.segment_distance = route_distances.get_value(deceleration_start_idx, corner_start);
              seg_data.acceleration_value = proposed_deceleration;
              seg_data.acceleration = seg_data.acceleration_value != 0.0;
              seg_data.segment_speed = {acceleration_end_speed, proposed_corner_speed};
              add_segment(loop_data, seg_data);

              logger("\nDECELERATION SPEED SEGMENT");
              logger(get_segment_string());

              // Add the cornering segment
              seg_data.segment = {corner_start, corner_end};
              seg_data.segment_distance = route_distances.get_value(corner_start, corner_end);
              seg_data.acceleration_value = 0.0;
              seg_data.acceleration = seg_data.acceleration_value != 0.0;
              seg_data.segment_speed = {proposed_corner_speed, proposed_corner_speed};
              add_segment(loop_data, seg_data);

              logger("\nCORNER SPEED SEGMENT");
              logger(get_segment_string());
            }
            valid = true;
          }
        } else {
          normal_straight = true;
        }

        // Create a normal straight: Pick a corner speed, accelerate/decelerate to the corner speed
        // in one shot and maintain it
        if (normal_straight) {
          if (is_first_segment) {
            logger("Taking straight between starting line and " +
            std::to_string(corner_idx) + " normally. Straight distance is " + std::to_string(straight_distance)
            + "m");
          } else {
            logger("Taking straight between corner " + std::to_string(last_real_corner_idx) + " and " +
            std::to_string(corner_idx) + " normally. Straight distance is " + std::to_string(straight_distance)
            + "m");
          }

          // Center the gaussian distribution at the desired average speed
          lower_bound_speed = std::max<int>(1, static_cast<int>(max_corner_speed * corner_speed_min_ratio));
          upper_bound_speed = static_cast<int>(max_corner_speed * corner_speed_max_ratio);
          if (target_average_speed > upper_bound_speed) {
            speed_mean = upper_bound_speed;
            speed_dev = speed_mean / 6.0;
          } else {
            speed_mean = target_average_speed;
            speed_dev = speed_mean / 6.0;
          }
          speed_dist = std::normal_distribution<double>(speed_mean, speed_dev);
          logger("Created corner speed gaussian distribution with lower bound " + std::to_string(lower_bound_speed) +
                "m/s, upper bound " + std::to_string(upper_bound_speed) + "m/s, mean of " +
                std::to_string(speed_mean) + " and deviation of " + std::to_string(speed_dev));

          valid = false;
          count = 0;
          sampled_speeds.clear();
          while (!valid) {
            count = count + 1;
            if (count == max_iters) {
              return false;
            }
            const double probability = 0.5;
            // Pick a corner speed. If the current speed is already >= the target average speed of the course,
            // then bias towards maintaining this constant speed rather than trying to accelerate
            bool maintain_speed = false;
            if (last_real_corner_speed >= target_average_speed && last_real_corner_speed < max_corner_speed) {
              logger("Last corner speed is greater or equal to the target average speed. Sample chance to maintain"
                     " same corner speed.");
              const double sample = skip_dist(skip_rng);
              if (sample < probability) {
                proposed_corner_speed = last_real_corner_speed;
                maintain_speed = true;
                logger("Maintaining last corner speed of " + std::to_string(last_real_corner_speed) + "m/s");
              }
            }

            if (corner_idx == num_corners && !maintain_speed) {
              // Wrap-around corner
              proposed_corner_speed = first_corner_speeds.top();
              // If this isn't the first time that we tried reaching the corner speed, unsupport
              // and rollback
              if (sampled_speeds.find(proposed_corner_speed) != sampled_speeds.end()) {
                return false;
              }
              sampled_speeds.insert(proposed_corner_speed);
            } else if (!maintain_speed) {
              proposed_corner_speed = bounded_gaussian<int>(speed_dist, speed_rng, lower_bound_speed,
                                                            upper_bound_speed, logger);
              // If all speeds have been attempted, return false and rollback to the last corner
              if (all_speeds_sampled()) {
                return false;
              } else if (sampled_speeds.find(proposed_corner_speed) != sampled_speeds.end()) {
                // If the sampled speed was already tried, sample another speed
                continue;
              }
            }

            distance_to_next_corner = route_distances.get_value(corner_end, next_corner_start);
            const std::pair<double, double> next_corner_speed_range = {next_corner_max_speed * corner_speed_min_ratio,
                                                                      next_corner_max_speed * corner_speed_max_ratio};
            if (proposed_corner_speed > last_real_corner_speed) {
              sampled_speeds.insert(proposed_corner_speed);
              // Need to accelerate to the current corner
              // Parameters to search for
              double acceleration_distance;
              double acceleration_ending_speed = proposed_corner_speed;
              double proposed_acceleration;
              size_t acceleration_end_idx;

              logger("Selected corner speed is " + std::to_string(proposed_corner_speed) +
                    " m/s which is greater than the last corner speed of " +
                    std::to_string(last_real_corner_speed) + "m/s");

              if (min_acceleration * car_mass * proposed_corner_speed > acceleration_power_allowance) {
                logger("Selected corner speed requires an acceleration that is too high for the motor");
                continue;
              }

              // Find the range of accelerations that can support the corner speed i.e. does not exceed
              // maximum acceleration distance and does not violate motor power constraints
              double lower_bound_acceleration = std::max(0.1, calc_acceleration(last_real_corner_speed,
                                                                                proposed_corner_speed,
                                                                                max_acceleration_distance));
              double upper_bound_acceleration = lower_bound_acceleration - 0.1;
              if (lower_bound_acceleration * car_mass * proposed_corner_speed >= acceleration_power_allowance) {
                logger("Corner speed is too high for the power of the motor");
                continue;
              }

              // Find the maximum acceleration that allows the car to reach the corner speed without
              // violating motor constraints
              do {
                // First iteration is guaranteed to be successful
                upper_bound_acceleration = upper_bound_acceleration + 0.1;
                instataneous_motor_power = proposed_corner_speed * upper_bound_acceleration * car_mass;
              } while (instataneous_motor_power < acceleration_power_allowance);

              upper_bound_acceleration = upper_bound_acceleration - 0.1;
              acceleration_dist = std::uniform_real_distribution<double>(lower_bound_acceleration,
                                                                         upper_bound_acceleration);

              proposed_acceleration = acceleration_dist(acceleration_rng);
              acceleration_distance = calc_distance_a(last_real_corner_speed,
                                                      proposed_corner_speed,
                                                      proposed_acceleration);

              // See if the car can reach the speed range of the next corner
              if (!can_reach_speeds(proposed_corner_speed, acceleration_power_allowance, max_acceleration,
                                    max_deceleration, next_corner_speed_range, distance_to_next_corner,
                                    car_mass)) {
                logger("This corner speed does not allow the car to reach the next corner's range of speeds");
                continue;
              }

              // Find the route index such that [last_real_corner_end, route index] is greater than the acceleration
              // distance
              acceleration_end_idx = last_real_corner_end;
              while (route_distances.get_value(last_real_corner_end, acceleration_end_idx) < acceleration_distance) {
                acceleration_end_idx = acceleration_end_idx == num_points - 1 ? 0 : acceleration_end_idx + 1;
              }

              logger("Proposed corner speed is valid. Required acceleration is " +
                     std::to_string(proposed_acceleration) + "m/s^2, the acceleration distance is " +
                     std::to_string(acceleration_distance) + "m corresponding to an ending index of " +
                     std::to_string(acceleration_end_idx));

              // Add the acceleration segment
              seg_data.segment = {last_real_corner_end, acceleration_end_idx};
              seg_data.segment_speed = {last_real_corner_speed, proposed_corner_speed};
              seg_data.segment_distance = route_distances.get_value(last_real_corner_end, acceleration_end_idx);
              seg_data.acceleration = true;
              seg_data.acceleration_value = proposed_acceleration;
              add_segment(loop_data, seg_data);

              logger("\nACCELERATION SEGMENT");
              logger(get_segment_string());

              // Add the corner segment
              seg_data.segment_distance = route_distances.get_value(acceleration_end_idx, corner_end);
              seg_data.segment = {acceleration_end_idx, corner_end};
              seg_data.segment_speed = {proposed_corner_speed, proposed_corner_speed};
              seg_data.acceleration_value = 0.0;
              seg_data.acceleration = false;
              add_segment(loop_data, seg_data);

              logger("\nCORNER SEGMENT");
              logger(get_segment_string());

              valid = true;
            } else if (proposed_corner_speed < last_real_corner_speed) {
              // Decelerate to corner speed
              // If the current corner can support the previous corner without decelerating, sample a chance
              // to try again - hope to maintain previous corner speed or accelerate
              if (last_real_corner_speed < max_corner_speed) {
                const double sample = skip_dist(skip_rng);
                if (sample < 0.8) {
                  logger("Last corner speed is less than the maximum corner speed of the current corner. "
                         "Sampled chance to try this sampling again");
                  continue;
                }
              }
              // Parameters to search for
              size_t deceleration_end_idx;
              double deceleration_distance;
              double proposed_deceleration;

              logger("Selected corner speed is " + std::to_string(proposed_corner_speed) + "m/s which is less than "
                    "the last corner speed of " + std::to_string(last_real_corner_speed) + "m/s");
              // Chosen corner speed is less than the previous corner speed, need to decelerate
              // First check if this corner speed is possible to reach with the preferred deceleration
              double min_deceleration = calc_acceleration(last_real_corner_speed, proposed_corner_speed,
                                                          max_acceleration_distance);
              if (min_deceleration < max_deceleration) {
                logger("Deceleration required to achieve the corner speed is too high. Trying again");
                continue;
              }

              // See if the car can reach the speed range of the next corner given the proposed
              // corner speed
              if (!can_reach_speeds(proposed_corner_speed, acceleration_power_allowance,
                                    max_acceleration, max_deceleration,
                                    next_corner_speed_range, max_acceleration_distance,
                                    car_mass)) {
                logger("The corner speed is too low/high to reach the next corner under the preferred "
                       "acceleration or preferred deceleration");
                continue;
              }

              deceleration_end_idx = last_real_corner_end;
              acceleration_dist = std::uniform_real_distribution<double>(max_deceleration, min_deceleration);
              logger("Created deceleration distribution with lower bound " + std::to_string(max_deceleration) +
                     " and upper bound " + std::to_string(min_deceleration));
              proposed_deceleration = acceleration_dist(acceleration_rng);
              deceleration_distance = calc_distance_a(last_real_corner_speed,
                                                      proposed_corner_speed,
                                                      proposed_deceleration);
              while (route_distances.get_value(last_real_corner_end, deceleration_end_idx) < deceleration_distance) {
                deceleration_end_idx++;
              }
              logger("Deceleration distance is " + std::to_string(deceleration_distance) + "m");

              logger("Proposed corner speed is valid");
              // Add the deceleration
              seg_data.segment = {last_real_corner_end, deceleration_end_idx};
              seg_data.segment_speed = {last_real_corner_speed, proposed_corner_speed};
              seg_data.acceleration_value = proposed_deceleration;
              seg_data.acceleration = true;
              seg_data.segment_distance = route_distances.get_value(last_real_corner_end, deceleration_end_idx);
              add_segment(loop_data, seg_data);

              logger("\nDECELERATION SEGMENT");
              logger(get_segment_string());

              // Add the corner segment
              seg_data.segment = {deceleration_end_idx, corner_end};
              seg_data.segment_speed = {proposed_corner_speed, proposed_corner_speed};
              seg_data.acceleration_value = 0.0;
              seg_data.acceleration = false;
              seg_data.segment_distance = route_distances.get_value(deceleration_end_idx, corner_end);
              add_segment(loop_data, seg_data);

              logger("\nCORNER SEGMENT");
              logger(get_segment_string());

              valid = true;
            } else {
              // Maintain same corner speed
              logger("Proposed corner speed is the same as the current speed of the car. Adding one segment");

              seg_data.acceleration = false;
              seg_data.acceleration_value = 0.0;
              seg_data.segment_speed = {proposed_corner_speed, proposed_corner_speed};
              seg_data.segment = {last_real_corner_end,  corner_end};
              seg_data.segment_distance = route_distances.get_value(last_real_corner_end, corner_end);
              add_segment(loop_data, seg_data);

              logger("\nCORNER SEGMENT");
              logger(get_segment_string());
              valid = true;
            }
          }
        }
        return true;
      };
      if (!create_segments()) {
        logger("Segments could NOT be created. Rolling back to the last segment");
        size_t last_real_corner_idx = real_corner_indices.top();
        logger("Re-creating segments for corner " + std::to_string(last_real_corner_idx) +
               "of loop block " + std::to_string(block_idx));

        remove_last_corner_segments();
        // Remove the last corner indices and speeds since they will be re-found
        real_corner_indices.pop();
        real_corner_speeds.pop();
        num_segments_added.pop();  // This must be called after remove_last_corner_segments()

        // If we failed on corner index 0, we need to rollback the entire last loop block
        if (corner_idx == 0) {
          RUNTIME_EXCEPTION(block_idx > 0, "Failed on the first segment");
          logger("Rolling back to the last corner of the previous loop block");
          loop_idx = loop_idx - num_repetitions;
          block_idx = block_idx - 1;

          // Restore the loop vectors
          loop_data.loop_segments = att.raw_loop_segments.back();
          loop_data.loop_segment_speeds = att.raw_loop_speeds.back();
          loop_data.loop_acceleration_segments = att.raw_acceleration_segments.back();
          loop_data.loop_acceleration_values = att.raw_acceleration_values.back();
          loop_data.loop_segment_distances = att.raw_loop_distances.back();

          att.raw_loop_segments.pop_back();
          att.raw_loop_speeds.pop_back();
          att.raw_loop_distances.pop_back();
          att.raw_acceleration_segments.pop_back();
          att.raw_acceleration_values.pop_back();

          RUNTIME_EXCEPTION(att.all_segments.size() > 0, "Segment plan has no vectors");

          // Remove the last loop block
          att.all_segments.erase(att.all_segments.end() - num_repetitions, att.all_segments.end());
          att.all_segment_speeds.erase(att.all_segment_speeds.end() - num_repetitions, att.all_segment_speeds.end());
          att.all_segment_distances.erase(att.all_segment_distances.end() - num_repetitions,
                                          att.all_segment_distances.end());
          att.all_acceleration_segments.erase(att.all_acceleration_segments.end() - num_repetitions,
                                              att.all_acceleration_segments.end());
          att.all_acceleration_values.erase(att.all_acceleration_values.end() - num_repetitions,
                                            att.all_acceleration_values.end());

          // Remove the number of wrap-around segments added to the block prior
          const size_t num_added_segments = num_added_wrap_around_segments.top();
          att.all_segments.back().erase(att.all_segments.back().end() - num_added_segments - 1,
                                        att.all_segments.back().end());
          att.all_segment_speeds.back().erase(att.all_segment_speeds.back().end() - num_added_segments - 1,
                                              att.all_segment_speeds.back().end());
          att.all_segment_distances.back().erase(att.all_segment_distances.back().end() - num_added_segments - 1,
                                                att.all_segment_distances.back().end());
          att.all_acceleration_segments.back().erase(att.all_acceleration_segments.back().end() - num_added_segments
                                                     - 1, att.all_acceleration_segments.back().end());
          att.all_acceleration_values.back().erase(att.all_acceleration_values.back().end() - num_added_segments - 1,
                                                   att.all_acceleration_values.back().end());
          num_added_wrap_around_segments.pop();
        }
        corner_idx = last_real_corner_idx;

        // If we're rolling back to the first corner of the first loop, set is_first_segment signal
        if (corner_idx == 0 && loop_idx == 0) {
          is_first_segment = true;
        }
      } else {
        // Segment creation was successful
        is_first_segment = false;
        // Update stack structures

        // We don't add no. of wrap-around corners for rolling back to specific corners since it doesn't make sense
        // to ever rollback to the wrap-around corner
        if (segment_counter > 0 && corner_idx != num_corners) {
          num_segments_added.push(segment_counter);
          real_corner_indices.push(corner_idx);
          real_corner_speeds.push(proposed_corner_speed);
          // If we completed the first corner, push the speeds and the ending segment index
          if (corner_idx == 0) {
            first_corner_speeds.push(static_cast<uint64_t>(proposed_corner_speed));
          }
        }

        // In the case of the wrap around corner, we need to add the number of segments so that if we ever
        // rollback to the last corner of the route, we remove the wrap-around as well
        if (segment_counter > 0 && corner_idx == num_corners) {
          num_segments_added.top() += segment_counter;
        }
        corner_idx = corner_idx + 1;
      }
    }

    // Add the created loop num_repetitions times to form a single block
    att.raw_loop_segments.push_back(loop_data.loop_segments);
    att.raw_loop_speeds.push_back(loop_data.loop_segment_speeds);
    att.raw_acceleration_segments.push_back(loop_data.loop_acceleration_segments);
    att.raw_acceleration_values.push_back(loop_data.loop_acceleration_values);
    att.raw_loop_distances.push_back(loop_data.loop_segment_distances);

    const size_t num_loops_created = create_loop_block();
    loop_idx = loop_idx + num_loops_created;
    block_idx = block_idx + 1;
  }

  return RacePlan(att.all_segments, att.all_segment_speeds, att.all_acceleration_segments,
                  att.all_acceleration_values, att.all_segment_distances, num_repetitions,
                  att.raw_loop_segments, att.raw_loop_speeds, att.raw_acceleration_segments,
                  att.raw_acceleration_values, att.raw_loop_distances);
}
