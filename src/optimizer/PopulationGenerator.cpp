#include <random>
#include <stack>
#include <unordered_map>
#include <memory>
#include <unordered_set>
#include <string>
#include <vector>
#include <algorithm>
#include <utility>

#include "opt/PopulationGenerator.hpp"
#include "utils/Defines.hpp"
#include "config/ConfigParser.hpp"
#include "utils/Units.hpp"
#include "utils/Logger.hpp"

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

RacePlanCreator::RacePlanCreator(std::shared_ptr<FSGPRoute> route,
                                unsigned speed_seed,
                                unsigned loop_seed,
                                unsigned aggressive_seed,
                                unsigned idx_seed,
                                unsigned acceleration_seed,
                                unsigned skip_seed) : route(route),
                                max_num_loops(Config::get_instance().get_max_num_loops()),
                                fix_loops(Config::get_instance().get_fix_num_loops()),
                                car_mass(Config::get_instance().get_car_mass()),
                                max_speed(kph2mps(Config::get_instance().get_max_route_speed())),
                                max_motor_power(kw2watts(Config::get_instance().get_max_motor_power())),
                                max_acceleration(Config::get_instance().get_max_acceleration()),
                                max_deceleration(Config::get_instance().get_max_deceleration()),
                                min_acceleration(Config::get_instance().get_min_acceleration()),
                                target_average_speed(Config::get_instance().get_average_speed()),
                                start_time(Config::get_instance().get_current_date_time()),
                                speed_seed(speed_seed), loop_seed(loop_seed), aggressive_seed(aggressive_seed),
                                idx_seed(idx_seed), acceleration_seed(acceleration_seed), skip_seed(skip_seed),
                                corner_speed_min_ratio(Config::get_instance().get_corner_speed_min()),
                                corner_speed_max_ratio(Config::get_instance().get_corner_speed_max()),
                                aggressive_straight_threshold(
                                  Config::get_instance().get_aggressive_straight_threshold()),
                                num_repetitions(Config::get_instance().get_num_repetitions()),
                                acceleration_power_budget(Config::get_instance().get_acceleration_power_budget()),
                                max_iters(1000), log(Config::get_instance().get_log_optimization()) {
  const Time day_one_start_time = Config::get_instance().get_day_one_start_time();
  const Time day_one_end_time = Config::get_instance().get_day_one_end_time();
  const Time day_end_time = Config::get_instance().get_day_end_time();  // Second and third day
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

  acceleration_power_allowance = acceleration_power_budget * max_motor_power;
  num_corners = cornering_segment_bounds.size();

  // Create random number generators
  rng_collection = Gen(speed_seed, loop_seed, aggressive_seed, idx_seed, acceleration_seed, skip_seed);
}

RacePlan RacePlanCreator::create_plan() {
  // Number of corners for which we need to create segments for
  const size_t num_corners_to_create = num_repetitions > 1 ? num_corners + 1 : num_corners;

  // Randomly select the number of loops to complete
  std::uniform_int_distribution<unsigned int> dis(1, max_num_loops);
  const size_t num_loops = fix_loops ? max_num_loops : static_cast<size_t>(dis(rng_collection.loop_rng));

  // Number of blocks to create
  size_t num_blocks;
  if (num_loops % num_repetitions == 0) {
    num_blocks = num_loops / num_repetitions;
  } else if (num_loops < num_repetitions) {
    num_blocks = 1;
  } else {
    num_blocks = num_loops / num_repetitions + 1;
  }

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

  // First corner of the first loop
  bool is_first_segment = true;

  // Counter for the number of blocks
  size_t block_idx = 0;

  // Create intermediate data storage
  RacePlan::LoopData loop_data;
  PlanAttributes att;
  PlanHistory history;

  size_t loop_idx = 0;
  while (block_idx < num_blocks) {
    // We construct the segments between the previous corner to the current corner.
    // We consider a complete loop as one that wraps back around to the first corner
    // of the route.
    size_t corner_idx = 0;
    while (corner_idx < num_corners_to_create) {
      logger("--------------CREATING SEGMENTS FOR CORNER " + std::to_string(corner_idx) +
             " OF LOOP BLOCK " + std::to_string(block_idx), false);
      if (!create_segments(corner_idx, &loop_data,
                           is_first_segment, &rng_collection, &history, logger)) {
        RUNTIME_EXCEPTION(corner_idx > 0 || loop_idx > 0, "Failed on the first segment of the first loop lol");
        const size_t last_real_corner_idx = rollback_to_last_real_corner(corner_idx, &history,
                                                                         &loop_data, &att, logger);
        if (corner_idx == 0) {
          loop_idx = loop_idx - num_repetitions;
          block_idx = block_idx - 1;
        }
        corner_idx = last_real_corner_idx;
        // If we're rolling back to the first corner of the first loop, set is_first_segment signal
        if (corner_idx == 0 && loop_idx == 0) {
          is_first_segment = true;
        }
      } else {
        // Segment creation was successful
        is_first_segment = false;
        corner_idx = corner_idx + 1;
      }
    }

    // Add the created loop num_repetitions times to form a single block
    att.raw_segments.push_back(loop_data);

    const size_t num_loops_in_block = std::min(static_cast<size_t>(num_repetitions), num_loops - loop_idx);
    logger("Creating loop block " + std::to_string(block_idx) + " with " +
           std::to_string(num_loops_in_block) + " loops");
    create_loop_block(&loop_data, num_loops_in_block, block_idx == num_blocks - 1,
                      block_idx == 0,  &att, &logger);
    loop_data.clear();
    loop_idx = loop_idx + num_loops_in_block;
    block_idx = block_idx + 1;
  }

  return RacePlan(att.all_segments, att.raw_segments, num_repetitions);
}

size_t RacePlanCreator::rollback_to_last_real_corner(size_t corner_idx, PlanHistory* history,
                                                     RacePlan::LoopData* loop_data,
                                                     PlanAttributes* att, FileLogger& logger) {  // NOLINT
  RUNTIME_EXCEPTION(history != nullptr && loop_data != nullptr && att != nullptr,
                    "Loop data, attributes and history storage cannot be null");

  logger("Loop being rolled back: " + RacePlan::get_loop_string(*loop_data));
  RUNTIME_EXCEPTION(history->real_corner_indices.size() >= 2, "Cannot rollback to last real corner - history is inaccurate");
  const size_t return_idx = history->real_corner_indices.top();
  history->real_corner_indices.pop();
  const size_t last_real_corner_idx = history->real_corner_indices.top();
  const size_t last_real_corner_ending_idx = (this->route->get_cornering_segment_bounds())[last_real_corner_idx].second;
  logger("Segments could NOT be created. Rolling back to the last real corner idx "
          + std::to_string(last_real_corner_idx) + " with ending route index of "
          + std::to_string(last_real_corner_ending_idx));
  logger("Re-creating segments for corner " + std::to_string(last_real_corner_idx));
  logger("Ending route index for this corner is " + std::to_string(last_real_corner_ending_idx));

  // The ending route index of the last corner of the route
  const size_t last_corner_ending_idx = this->route->get_cornering_segment_bounds().back().second;

  // Rollback from current loop
  size_t curr_loop_counter = 0;
  size_t prev_loop_counter = 0;
  bool rollback_complete = false;
  while (!loop_data->empty() && !rollback_complete) {
    if (loop_data->back().end_idx != last_real_corner_ending_idx) {
      loop_data->pop_back();
      curr_loop_counter++;
    } else {
      rollback_complete = true;
    }
  }

  logger("Removed " + std::to_string(curr_loop_counter) + " segments from current loop");

  RUNTIME_EXCEPTION(rollback_complete || att->raw_segments.size() > 0, "No loops left but rollback not complete");

  while (!rollback_complete) {
    RUNTIME_EXCEPTION(!att->raw_segments.back().empty(), "No segments left to remove");
    if (att->raw_segments.back().back().end_idx == last_real_corner_ending_idx) {
      rollback_complete = true;
    } else {
      att->raw_segments.back().pop_back();
      prev_loop_counter++;
    }
  }
  logger("Removed " + std::to_string(prev_loop_counter) + " segments from the previous loop");
  // These parameters will be re-found
  history->real_corner_speeds.pop();

  // If we failed on corner index 0, we need to rollback to the last loop block
  if (corner_idx == 0) {
    logger("Rolling back to the last corner of the previous loop block");

    // Restore the loop vectors
    *loop_data = att->raw_segments.back();

    att->raw_segments.pop_back();

    RUNTIME_EXCEPTION(att->all_segments.size() > 0, "Segment plan has no vectors");

    // Remove the last loop block
    att->all_segments.erase(att->all_segments.end() - num_repetitions, att->all_segments.end());

    // From the last loop, remove until the end of the last corner of the route
    bool successful_removal = false;
    while (!att->all_segments.back().empty()) {
      if (att->all_segments.back().back().end_idx == last_corner_ending_idx) {
        successful_removal = true;
      } else {
        att->all_segments.back().pop_back();
      }
    }
    RUNTIME_EXCEPTION(successful_removal, "Could not find the last corner index of the previous loop");
  }
  return return_idx;
}

void RacePlanCreator::PlanAttributes::add_loop(const RacePlan::LoopData& loop_data,
                                               size_t start_idx, size_t end_idx) {
  RUNTIME_EXCEPTION(start_idx <= end_idx, "End index must be greater than Start index");
  RUNTIME_EXCEPTION(end_idx <= loop_data.size(), "Ending index must be less than or equal to the "
                                                 "loop segments vector");
  // Store results for this loop
  RacePlan::LoopData segments_slice(loop_data.begin() + start_idx,
                                    loop_data.begin() + end_idx);
  all_segments.push_back(segments_slice);
}

void RacePlanCreator::create_loop_block(RacePlan::LoopData* loop_data,
                                        int num_loops_in_block,
                                        bool is_last_block,
                                        bool is_first_block,
                                        PlanAttributes* att,
                                        FileLogger* logger) {
  RUNTIME_EXCEPTION(loop_data != nullptr, "Intermediate loop data is null");
  RUNTIME_EXCEPTION(att != nullptr, "Race Plan attributes struct is null");
  RUNTIME_EXCEPTION(route != nullptr, "Route distances LUT must be computed");
  const BasicLut route_distances = route->get_precomputed_distances();

  if (logger != nullptr) {
    (*logger)("\n////CREATED LOOP////");
    (*logger)(RacePlan::get_loop_string(*loop_data));
  }

  // --------------- Create the first loop of the block ---------------
  if (logger != nullptr) {
    (*logger)("Creating loop 1");
  }
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
  while (!is_first_block && (*loop_data)[0].start_idx < (*loop_data)[0].end_idx) {
    att->all_segments.back().push_back((*loop_data)[0]);

    if (logger) {
      (*logger)("Moving segment [" + std::to_string((*loop_data)[0].start_idx) + "," +
            std::to_string((*loop_data)[0].end_idx) + "] to the previous loop");
    }

    // Remove the first element of loop vectors - this is why we don't have to keep track of an index variable
    loop_data->erase(loop_data->begin());
  }
  if ((*loop_data)[0].end_idx == 0) {
    att->all_segments.back().push_back((*loop_data)[0]);

    if (logger) {
      (*logger)("Moving segment [" + std::to_string((*loop_data)[0].start_idx) + "," +
            std::to_string((*loop_data)[0].end_idx) + "] to the previous loop");
    }

    loop_data->erase(loop_data->begin());
  }

  // Crossover segment detected. Break into two sub-segments
  if ((*loop_data)[0].start_idx > (*loop_data)[0].end_idx) {
    const double first_segment_distance = route_distances.get_value((*loop_data)[0].start_idx, 0);
    const double first_segment_ending_speed = calc_final_speed_a((*loop_data)[0].start_speed,
                                                                 (*loop_data)[0].acceleration_value,
                                                                 first_segment_distance);
    // To the end of the loop [x,0]
    RacePlan::SegmentData first_segment(
      (*loop_data)[0].start_idx, 0, (*loop_data)[0].start_speed,
      first_segment_ending_speed,
      (*loop_data)[0].acceleration_value,
      first_segment_distance
    );
    
    att->all_segments.back().push_back(first_segment);

    const double second_segment_distance = route_distances.get_value(0, (*loop_data)[0].end_idx);
    (*loop_data)[0].start_idx = 0;
    (*loop_data)[0].start_speed = first_segment_ending_speed;
    (*loop_data)[0].distance = second_segment_distance;
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
  RacePlan::LoopData wrap_around_segments;
  while (num_repetitions > 1 && loop_data->back().end_idx > loop_data->back().start_idx &&
        loop_data->back().end_idx != 0) {
    wrap_around_segments.insert(wrap_around_segments.begin(), loop_data->back());
    loop_data->pop_back();
  }

  // Crossover segment detected on last segment. We will only insert the first sub-segment into
  // the current loop, but we also need to preserve the second segment when we create the loops > 1st one
  // of the block
  if (loop_data->back().end_idx < loop_data->back().start_idx && loop_data->back().end_idx != 0) {
    std::pair<size_t, size_t> first_segment = {loop_data->back().start_idx, 0};
    const double first_segment_distance = route_distances.get_value(first_segment.first, 0);
    const double first_segment_ending_speed = calc_final_speed_a(loop_data->back().start_speed,
                                                                 loop_data->back().acceleration_value,
                                                                 first_segment_distance);
    std::pair<double, double> first_segment_speeds = {loop_data->back().start_speed,
                                                      first_segment_ending_speed};
    const std::pair<size_t, size_t> second_segment = {0, loop_data->back().end_idx};
    const double second_segment_distance = route_distances.get_value(0, second_segment.second);
    const double second_segment_ending_speed = loop_data->back().end_speed;
    const std::pair<double, double> second_segment_speeds = {first_segment_ending_speed,
                                                             second_segment_ending_speed};

    RacePlan::SegmentData new_first_segment(
      first_segment.first, first_segment.second, first_segment_speeds.first,
      first_segment_speeds.second, loop_data->back().acceleration_value,
      first_segment_distance,
      route->get_overlapping_corners(first_segment)
    );
    wrap_around_segments.insert(wrap_around_segments.begin(), RacePlan::SegmentData(
      second_segment.first, second_segment.second, second_segment_speeds.first,
      second_segment_speeds.second, loop_data->back().acceleration_value,
      second_segment_distance,
      route->get_overlapping_corners(second_segment)
    ));

    loop_data->back() = new_first_segment;
  }
  // Add the first loop
  if (num_loops_in_block > 1) {
    if (logger != nullptr) {
      (*logger)("Loop 1 Modified:");
      (*logger)(RacePlan::get_loop_string(*loop_data));
    }
    att->add_loop(*loop_data, 0, loop_data->size());
  }
  // --------------------------------------------------------------------

  // --------------- Create the loops after the first one ---------------
  // This is only applicable if the number of loops in a block is greater than 1
  if (num_loops_in_block > 1) {
    if (logger) {
      (*logger)("Modifying loop construct for the second... loop in the block");
    }
    // Refer to figure A, we first want to delete the segments that extend up to and including
    // the first corner as they will be replaced by the wrap-around segment
    while ((*loop_data)[0].end_idx <= cornering_segment_bounds[0].second) {
      loop_data->erase(loop_data->begin());
    }

    // Move the recorded wrap around segments to the front
    loop_data->insert(loop_data->begin(),
                      wrap_around_segments.begin(),
                      wrap_around_segments.end());

    for (size_t i=1; i < num_loops_in_block - 1; i++) {
      att->add_loop(*loop_data, 0, loop_data->size());
    }
    if (logger) {
      (*logger)("LOOP X MODIFIED");
      (*logger)(RacePlan::get_loop_string(*loop_data));
    }
  }

  // --------------------------------------------------------------------

  // --------------- Add the last loop of the block ---------------
  int last_real_corner_ending_idx = loop_data->size() - 1;  // A segment index
  const auto corner_end = this->route->get_corner_end_indices();
  bool crossed_over = false;
  for (; last_real_corner_ending_idx >= 0; last_real_corner_ending_idx--) {
    const size_t end_idx = (*loop_data)[last_real_corner_ending_idx].end_idx;
    const size_t start_idx = (*loop_data)[last_real_corner_ending_idx].start_idx;
    if (end_idx < start_idx) {
      crossed_over = true;
    }
    if (crossed_over && corner_end.find(end_idx) != corner_end.end()) {
      break;
    }
  }
  RUNTIME_EXCEPTION(last_real_corner_ending_idx >= 0, "Failed search for last real corner index");
  if (is_last_block) {
    // Remove all the segments past the last corner and create a constant segment until index 0
    const size_t num_segments_to_remove = loop_data->size() - last_real_corner_ending_idx;
    if (logger) {
      (*logger)("Last corner ending idx " + std::to_string(last_real_corner_ending_idx));
      (*logger)("Loop segments size: " + std::to_string(loop_data->size()));
      (*logger)("Num segments to remove: " + std::to_string(num_segments_to_remove));
    }
    loop_data->erase(loop_data->end() - num_segments_to_remove, loop_data->end());

    const size_t corner_end_idx = loop_data->back().end_idx;
    const int corner_speed = loop_data->back().end_speed;

    RacePlan::SegmentData seg_data(corner_end_idx, 0, corner_speed, corner_speed, 0.0,
                                   route_distances.get_value(corner_end_idx, 0),
                                   route->get_overlapping_corners({corner_end_idx, 0}));

    loop_data->push_back(seg_data);
    att->add_loop(*loop_data, 0, loop_data->size());
  } else {
    if (logger) {
      (*logger)("Last corner ending index: " + std::to_string(last_real_corner_ending_idx));
    }
    // If this is the last loop of the block, then we stop at the end of the last corner
    // since we want the next loop to decide how to travel from the last corner to the first
    att->add_loop(*loop_data, 0, last_real_corner_ending_idx + 1);
  }
  if (logger) {
    (*logger)("LOOP LAST MODIFIED");
    (*logger)(RacePlan::get_loop_string(att->all_segments.back()));
  }
  return;
}

bool RacePlanCreator::create_segments(size_t corner_idx,
                                      RacePlan::LoopData* loop_data,
                                      bool is_first_segment,
                                      Gen* rng,
                                      PlanHistory* history,
                                      FileLogger& logger) {  // NOLINT
  ///////////////////////////////////////////////////////////////////////////////////
  ///////////////////////// State variables for each corner /////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////
  RacePlan::SegmentData seg_data;

  // Whether a straight is taken aggressively
  bool aggressive_straight = false;

  // Whether a straight is taking normally
  bool normal_straight = false;

  // Attributes of the next and last corner
  double prev_corner_max_speed = 0.0;  // Maximum speed of the previous corner
  double next_corner_max_speed = 0.0;  // Maximum speed of the next speed
  size_t next_corner_start = 0;        // Starting route index of the next corner
  size_t prev_corner_end = 0;          // Ending route index of the previous corner

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

  // Probability distributions used to randomly select locations, speeds
  // and accelerations. Note that they are inclusive on both sides
  std::uniform_int_distribution<size_t> idx_dist;
  std::normal_distribution<double> speed_dist;
  std::uniform_real_distribution<double> acceleration_dist(0.1, max_acceleration);
  std::uniform_real_distribution<double> aggressive_dist(0.0, 1.0);
  std::uniform_real_distribution<double> skip_dist(0.0, 1.0);

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

  // Wrap-around corner is index 0
  if (corner_idx == num_corners) {
    logger(" [WRAP-AROUND CORNER]--------------");
  } else {
    logger("--------------");
  }
  size_t true_corner_idx = corner_idx == num_corners ? 0 : corner_idx;
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
  const size_t last_real_corner_idx = history->real_corner_indices.top();
  const double last_real_corner_speed = history->real_corner_speeds.top();
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
  // Collect all the corner indices
  std::vector<size_t> corner_indices;
  for (size_t i=last_real_corner_idx+1; i <= corner_idx; i++) {
    corner_indices.emplace_back(i);
  }

  if (straight_includes_corners) {
    non_acceleration_zone_start = cornering_segment_bounds[last_real_corner_idx+1].first;
    max_acceleration_distance = route_distances.get_value(last_real_corner_end, non_acceleration_zone_start);
    logger("The straight includes corners with maximum speeds greater than the route speed");
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
  const double sample = aggressive_dist(rng->aggressive_rng);
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
      acceleration_end_speed = bounded_gaussian<int>(speed_dist, rng->speed_rng, last_real_corner_speed + 1,
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
        proposed_acceleration = acceleration_dist(rng->acceleration_rng);
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
          proposed_corner_speed = history->first_corner_speeds.top();
        } else {
          proposed_corner_speed = bounded_gaussian<int>(speed_dist, rng->speed_rng, lower_bound_speed,
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
        deceleration_start_idx = idx_dist(rng->idx_rng);
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
        if (proposed_deceleration > 0.0 && straight_includes_corners &&
            route->get_overlapping_corners({deceleration_start_idx, corner_start}).size() > 0) {
          logger("Proposed deceleration is positive and overlaps with a corner. Safety constraint violated");
          valid_corner_speed = false;
          continue;
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
        seg_data = RacePlan::SegmentData(
          last_real_corner_end, acceleration_end_idx, last_real_corner_speed, acceleration_end_speed,
          proposed_acceleration, route_distances.get_value(last_real_corner_end, acceleration_end_idx),
          route->get_overlapping_corners({last_real_corner_end, acceleration_end_idx}), corner_idx == num_corners - 1
        );
        loop_data->push_back(seg_data);

        logger("\nACCELERATION SEGMENT");
        logger(RacePlan::get_segment_string(seg_data));

        // Add constant speed segment
        seg_data = RacePlan::SegmentData(
          acceleration_end_idx, deceleration_start_idx, acceleration_end_speed, acceleration_end_speed,
          0.0, route_distances.get_value(acceleration_end_idx, deceleration_start_idx),
          route->get_overlapping_corners({acceleration_end_idx, deceleration_start_idx})
        );
        loop_data->push_back(seg_data);

        logger("\nCONSTANT SPEED SEGMENT");
        logger(RacePlan::get_segment_string(seg_data));

        // Add deceleration speed segment
        seg_data = RacePlan::SegmentData(
          deceleration_start_idx, corner_start, acceleration_end_speed, proposed_corner_speed,
          proposed_deceleration, route_distances.get_value(deceleration_start_idx, corner_start),
          route->get_overlapping_corners({deceleration_start_idx, corner_start})
        );
        loop_data->push_back(seg_data);

        logger("\nDECELERATION SPEED SEGMENT");
        logger(RacePlan::get_segment_string(seg_data));

        // Add the cornering segment
        seg_data = RacePlan::SegmentData(
          corner_start, corner_end, proposed_corner_speed, proposed_corner_speed,
          0.0, route_distances.get_value(corner_start, corner_end),
          route->get_overlapping_corners({corner_start, corner_end})
        );
        loop_data->push_back(seg_data);

        logger("\nCORNER SPEED SEGMENT");
        logger(RacePlan::get_segment_string(seg_data));
      }
      valid = true;
    }
  } else {
    normal_straight = true;
  }

  // Create a normal straight: Pick a corner speed, accelerate/decelerate the entire straight distance
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

    if (!straight_includes_corners) {
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
    } else {
      lower_bound_speed = 1;
      upper_bound_speed = last_real_corner_speed;
      speed_mean = static_cast<int>((lower_bound_speed + upper_bound_speed) / 2.0);
      speed_dev = speed_mean / 6.0;
      logger("Straight includes corners, upper bound of speed distribution will be the current speed of the car");
    }
    logger("Created corner speed gaussian distribution with lower bound " + std::to_string(lower_bound_speed) +
          "m/s, upper bound " + std::to_string(upper_bound_speed) + "m/s, mean of " +
          std::to_string(speed_mean) + " and deviation of " + std::to_string(speed_dev));
    speed_dist = std::normal_distribution<double>(speed_mean, speed_dev);

    valid = false;
    count = 0;
    sampled_speeds.clear();
    while (!valid) {
      count = count + 1;
      if (count == max_iters) {
        return false;
      }

      if (corner_idx == num_corners) {
        // Wrap-around corner
        proposed_corner_speed = history->first_corner_speeds.top();
        // If this isn't the first time that we tried reaching the corner speed, unsupport
        // and rollback
        if (sampled_speeds.find(proposed_corner_speed) != sampled_speeds.end()) {
          return false;
        }
        sampled_speeds.insert(proposed_corner_speed);
      } else {
        proposed_corner_speed = bounded_gaussian<int>(speed_dist, rng->speed_rng, lower_bound_speed,
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
        double acceleration_distance = straight_distance;
        double acceleration_ending_speed = proposed_corner_speed;
        double proposed_acceleration = calc_acceleration(last_real_corner_speed, proposed_corner_speed, straight_distance);
        size_t acceleration_end_idx = corner_start;

        logger("Selected corner speed is " + std::to_string(proposed_corner_speed) +
              " m/s which is greater than the last corner speed of " +
              std::to_string(last_real_corner_speed) + "m/s");
        logger("The required acceleration is " + std::to_string(proposed_acceleration) + "m/s^2");

        if (proposed_acceleration * car_mass * proposed_corner_speed > acceleration_power_allowance) {
          logger("Selected corner speed requires an acceleration that is too high for the motor");
          continue;
        }

        if (proposed_acceleration > max_acceleration) {
          logger("Selected corner speed requires an acceleration that exceeds the maximum acceleration");
          continue;
        }

        // See if the car can reach the speed range of the next corner
        if (!can_reach_speeds(proposed_corner_speed, acceleration_power_allowance, max_acceleration,
                              max_deceleration, next_corner_speed_range, distance_to_next_corner,
                              car_mass)) {
          logger("This corner speed does not allow the car to reach the next corner's range of speeds");
          continue;
        }

        logger("Proposed corner speed is valid. Required acceleration is " +
                std::to_string(proposed_acceleration) + "m/s^2, the acceleration distance is " +
                std::to_string(acceleration_distance) + "m corresponding to an ending index of " +
                std::to_string(acceleration_end_idx));

        // Add the acceleration segment
        seg_data = RacePlan::SegmentData(
          last_real_corner_end, acceleration_end_idx, last_real_corner_speed, proposed_corner_speed,
          proposed_acceleration, route_distances.get_value(last_real_corner_end, acceleration_end_idx),
          route->get_overlapping_corners({last_real_corner_end, acceleration_end_idx})
        );
        loop_data->push_back(seg_data);

        logger("\nACCELERATION SEGMENT");
        logger(RacePlan::get_segment_string(seg_data));

        // Add the corner segment
        seg_data = RacePlan::SegmentData(
          acceleration_end_idx, corner_end, proposed_corner_speed, proposed_corner_speed,
          0.0, route_distances.get_value(acceleration_end_idx, corner_end),
          route->get_overlapping_corners({acceleration_end_idx, corner_end})
        );
        loop_data->push_back(seg_data);

        logger("\nCORNER SEGMENT");
        logger(RacePlan::get_segment_string(seg_data));

        valid = true;
      } else if (proposed_corner_speed < last_real_corner_speed) {
        // Decelerate to corner speed
        // If the current corner can support the previous corner without decelerating, sample a chance
        // to try again - hope to maintain previous corner speed or accelerate
        if (last_real_corner_speed < max_corner_speed) {
          const double sample = skip_dist(rng->skip_rng);
          if (sample < 0.8) {
            logger("Last corner speed is less than the maximum corner speed of the current corner. "
                    "Sampled chance to try this sampling again");
            continue;
          }
        }
        sampled_speeds.insert(proposed_corner_speed);

        // Parameters to search for
        size_t deceleration_end_idx = corner_start;
        double deceleration_distance = straight_distance;
        double proposed_deceleration = calc_acceleration(last_real_corner_speed, proposed_corner_speed, straight_distance);

        logger("Selected corner speed is " + std::to_string(proposed_corner_speed) + "m/s which is less than "
              "the last corner speed of " + std::to_string(last_real_corner_speed) + "m/s");
        logger("Required deceleration is " + std::to_string(proposed_deceleration) + "m/s^2");

        if (proposed_deceleration < max_deceleration) {
          logger("Selected corner speed requires deceleration that is greater than the maximum deceleration");
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

        logger("Proposed corner speed is valid");
        // Add the deceleration
        seg_data = RacePlan::SegmentData(
          last_real_corner_end, deceleration_end_idx, last_real_corner_speed, proposed_corner_speed,
          proposed_deceleration, route_distances.get_value(last_real_corner_end, deceleration_end_idx),
          route->get_overlapping_corners({last_real_corner_end, deceleration_end_idx})
        );
        loop_data->push_back(seg_data);

        logger("\nDECELERATION SEGMENT");
        logger(RacePlan::get_segment_string(seg_data));

        // Add the corner segment
        seg_data = RacePlan::SegmentData(
          deceleration_end_idx, corner_end, proposed_corner_speed, proposed_corner_speed,
          0.0, route_distances.get_value(deceleration_end_idx, corner_end),
          route->get_overlapping_corners({deceleration_end_idx, corner_end})
        );
        loop_data->push_back(seg_data);

        logger("\nCORNER SEGMENT");
        logger(RacePlan::get_segment_string(seg_data));

        valid = true;
      } else {
        // Maintain same corner speed
        logger("Proposed corner speed is the same as the current speed of the car. Adding one segment");
        seg_data = RacePlan::SegmentData(
          last_real_corner_end, corner_start, proposed_corner_speed, proposed_corner_speed, 0.0,
          route_distances.get_value(last_real_corner_end, corner_start),
          route->get_overlapping_corners({last_real_corner_end, corner_start})
        );
        loop_data->push_back(seg_data);
        seg_data = RacePlan::SegmentData(
          corner_start, corner_end, proposed_corner_speed, proposed_corner_speed,
          0.0, route_distances.get_value(corner_start, corner_end),
          route->get_overlapping_corners({corner_start, corner_end})
        );
        loop_data->push_back(seg_data);

        logger("\nCORNER SEGMENT");
        logger(RacePlan::get_segment_string(seg_data));
        valid = true;
      }
    }
  }

  // We don't add no. of wrap-around corners for rolling back to specific corners since it doesn't make sense
  // to ever rollback to the wrap-around corner
  if (corner_idx != num_corners) {
    history->real_corner_indices.push(corner_idx);
    history->real_corner_speeds.push(proposed_corner_speed);
    // If we completed the first corner, push the speeds and the ending segment index
    if (corner_idx == 0) {
      history->first_corner_speeds.push(static_cast<uint64_t>(proposed_corner_speed));
    }
  }

  return true;
}
