/** This file provides functions for crossover, mutation and initial population creation */

#pragma once

#include <memory>
#include <utility>
#include <stack>
#include <vector>
#include <random>

#include "route/Route.hpp"
#include "utils/CustomTime.hpp"
#include "utils/Logger.hpp"
#include "utils/Luts.hpp"

namespace Genetic {
/** @brief Mutate a race plan according to a strategy chosen from configuration */
RacePlan mutate_plan(RacePlan plan, const std::shared_ptr<Route> route);

};  // namespace Genetic

// Class for creating RacePlan objects in the initial population. RacePlan creation as done according to the process
// described in. This class also provides functions for creating race plans on existing loops used for mutation
// in the genetic optimizer
// https://www.notion.so/blueskysolar/Race-Strategy-and-Testing-Process-1da8d1f46c3680a79056edf3c9fecd1b?pvs=4
class RacePlanCreator {
 public:
  ////////////////////////////////////////////////////////////////////////////////////////////////
  // The following structs are used in create_plan() to hold intermediate data as the race plan //
  // is being created                                                                           //
  ////////////////////////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////////////////////
  ////////////////////// NOTE ABOUT TERMINOLOGY USED ///////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////
  // "route index": Denotes an index in the route_points_array
  // "corner index": Denotes an index in the cornering_segment_bounds array of the Route object
  // "real corner": A corner where the maximum cornering speed is less than the route's speed limit
  // "segment index": An index inside loop_segments, loop_segment_speeds ...
  // "wrap-around corner": The first corner of the route which the car wraps to after taking the entire route

  // Keeps track of the history of different data elements added to the race plan
  // e.g. number of segments added for each corner, speed of each corner etc.
  // This is required when rolling back to a previous corner after segment generation failed
  struct PlanHistory {
    // The chosen speed for each real corner
    std::stack<double> real_corner_speeds;
    // Real corner indices for which segments were created
    std::stack<size_t> real_corner_indices;
    // Number of segments added for each real corner
    std::stack<uint64_t> num_segments_added;
    // Speed of the first corner of a generated loop
    std::stack<uint64_t> first_corner_speeds;
    // Number of segments added to the last loop of the previous loop block
    // as a result of carrying over the wrap-around segments.
    std::stack<uint64_t> num_added_wrap_around_segments;
    // Counter to track the number of segments added for each real corner
    uint64_t segment_counter;

    void reset_segment_counter() {
      segment_counter = 0;
    }

    PlanHistory() {
      real_corner_speeds.push(0.0);
      real_corner_indices.push(0);
      num_segments_added.push(0);
      first_corner_speeds.push(0);
      num_added_wrap_around_segments.push(0);
    }
  };

  // Hold information for a single segment
  struct SegmentData {
    std::pair<size_t, size_t> segment;
    std::pair<double, double> segment_speed;
    bool acceleration;
    double acceleration_value;
    double segment_distance;

    SegmentData(
      std::pair<size_t, size_t> segment = {0.0, 0.0},
      std::pair<double, double> segment_speed = {0.0, 0.0},
      bool acceleration = false,
      double acceleration_value = 0.0,
      double segment_distance = 0.0) : segment(segment),
        segment_speed(segment_speed),
        acceleration(acceleration),
        acceleration_value(acceleration_value),
        segment_distance(segment_distance) {}
  };

  // Holds the intermediate loop data when creating a race plan
  struct LoopData {
    std::vector<std::pair<size_t, size_t>> loop_segments;
    std::vector<std::pair<double, double>> loop_segment_speeds;
    std::vector<bool> loop_acceleration_segments;
    std::vector<double> loop_acceleration_values;
    std::vector<double> loop_segment_distances;
    uint64_t segment_counter;

    void clear() {
      loop_segments.clear();
      loop_segment_speeds.clear();
      loop_acceleration_segments.clear();
      loop_acceleration_values.clear();
      loop_segment_distances.clear();
      segment_counter = 0;
    }

    void reset_segment_counter() {
      segment_counter = 0;
    }

    void add_segment(SegmentData* seg_data, PlanHistory* history);

    LoopData(
      std::vector<std::pair<size_t, size_t>> loop_segments = {},
      std::vector<std::pair<double, double>> loop_segment_speeds = {},
      std::vector<bool> loop_acceleration_segments = {},
      std::vector<double> loop_acceleration_values = {},
      std::vector<double> loop_segment_distances = {},
      uint64_t segment_counter = 0) : loop_segments(loop_segments),
        loop_segment_speeds(loop_segment_speeds),
        loop_acceleration_segments(loop_acceleration_segments),
        loop_acceleration_values(loop_acceleration_values),
        loop_segment_distances(loop_segment_distances),
        segment_counter(segment_counter) {}
  };

  // Holds the RacePlan attributes that will be passed into the return object
  struct PlanAttributes {
    // Data holders for each loop before processing when constructing a loop block. For each loop
    // block, we create one loop and modify the beginning or ending segments in order to glue the
    // loops of the block together. These vectors hold the uniquely created loop for each block
    std::vector<std::vector<std::pair<size_t, size_t>>> raw_loop_segments;
    std::vector<std::vector<std::pair<double, double>>> raw_loop_speeds;
    std::vector<std::vector<bool>> raw_acceleration_segments;
    std::vector<std::vector<double>> raw_acceleration_values;
    std::vector<std::vector<double>> raw_loop_distances;

    // RacePlan attributes
    std::vector<std::vector<std::pair<size_t, size_t>>> all_segments;
    std::vector<std::vector<std::pair<double, double>>> all_segment_speeds;
    std::vector<std::vector<bool>> all_acceleration_segments;
    std::vector<std::vector<double>> all_acceleration_values;
    std::vector<std::vector<double>> all_segment_distances;

    void add_loop(const LoopData& loop_data, size_t start_idx, size_t end_idx);

    PlanAttributes(
      std::vector<std::vector<std::pair<size_t, size_t>>> raw_loop_segments = {},
      std::vector<std::vector<std::pair<double, double>>> raw_loop_speeds = {},
      std::vector<std::vector<bool>> raw_acceleration_segments = {},
      std::vector<std::vector<double>> raw_acceleration_values = {},
      std::vector<std::vector<double>> raw_loop_distances = {},
      std::vector<std::vector<std::pair<size_t, size_t>>> all_segments = {},
      std::vector<std::vector<std::pair<double, double>>> all_segment_speeds = {},
      std::vector<std::vector<bool>> all_acceleration_segments = {},
      std::vector<std::vector<double>> all_acceleration_values = {},
      std::vector<std::vector<double>> all_segment_distances = {}) : raw_loop_segments(raw_loop_segments),
        raw_loop_speeds(raw_loop_speeds),
        raw_acceleration_segments(raw_acceleration_segments),
        raw_acceleration_values(raw_acceleration_values),
        raw_loop_distances(raw_loop_distances),
        all_segments(all_segments),
        all_segment_speeds(all_segment_speeds),
        all_acceleration_segments(all_acceleration_segments),
        all_acceleration_values(all_acceleration_values),
        all_segment_distances(all_segment_distances) {}
  };

  /** Initialize all parameters */
  RacePlanCreator(std::shared_ptr<Route> route,
                  unsigned speed_seed = 1,
                  unsigned loop_seed = 1,
                  unsigned aggressive_seed = 1,
                  unsigned idx_seed = 1,
                  unsigned acceleration_seed = 1,
                  unsigned skip_seed = 1);

  /** @brief Create a single race plan */
  RacePlan create_plan();

  /** @brief Helper to create_plan used to create a single loop block
   * @param seg_data Intermediate segment data for the latest created segment
   * @param loop_data Intermediate data for the last created loop
   * @param attributes All data for the entire race plan to the current point
   * @param route_distances Lookup table for pre-computed distances of the route
   * @param logger Logger for the race plan creation
   * @param num_loops_in_block Number of loops in the block
   * @param is_last_block Whether the block to be created is the last block of the entire plan
   * @param is_first_block Whether the block to be created is the first block of the entire plan
  */
  void create_loop_block(LoopData* loop_data,
                         PlanAttributes* att,
                         BasicLut* route_distances,
                         PlanHistory* history,
                         FileLogger& logger,  // NOLINT
                         int num_loops_in_block = 1,
                         bool is_last_block = false,
                         bool is_first_block = false);

  // Create only one rng at the beginning of create_plan and pass it to
  // create_segments each corner so that the state is constantly changed
  struct Gen {
    std::mt19937 speed_rng;
    std::mt19937 loop_rng;
    std::mt19937 aggressive_rng;
    std::mt19937 idx_rng;
    std::mt19937 acceleration_rng;
    std::mt19937 skip_rng;

    Gen(unsigned int speed_seed = 1, unsigned int loop_seed = 1, unsigned int aggressive_seed = 1,
        unsigned int idx_seed = 1, unsigned int acceleration_seed = 1, unsigned int skip_seed = 1) {
      speed_rng.seed(speed_seed);
      loop_rng.seed(loop_seed);
      aggressive_rng.seed(aggressive_seed);
      idx_rng.seed(idx_seed);
      acceleration_rng.seed(acceleration_seed);
      skip_rng.seed(skip_seed);
    }
  };

  /** @brief Helper to create_plan used to create segments for a single corner */
  bool create_segments(size_t corner_idx, size_t block_idx,
                       LoopData* loop_data,
                       bool is_first_segment,
                       Gen* rng,
                       PlanHistory* history,
                       FileLogger& logger);  // NOLINT

  /** @brief Helper to create_plan used for rolling back a corner after failed segment generation 
   * @return Index of last real corner to which we are rolling back
  */
  size_t rollback_to_last_real_corner(size_t corner_idx, PlanHistory* history, LoopData* loop_data,
                                      PlanAttributes* att, FileLogger& logger);  // NOLINT

 private:
  /** @brief Helper function to segment_route_corners that determines if a range of
    * speeds is reachable within some maximum distance without violating constraints
    *
    * @param initial_speed: Initial speed in m/s
    * @param acceleration_power: Acceleration power
    * @param max_acceleration: Maximum accleration
    * @param max_deceleration: Maximum deceleration
    * @param preferred_deceleration: Preferred deceleration
    * @param speed_range: Range of speeds to target
    * @param max_distance: Distance to cover
    * @param car_mass: Car mass in kg
    */
  bool can_reach_speeds(double initial_speed, double acceleration_power, double max_acceleration,
                        double max_deceleration, std::pair<double, double> speed_range,
                        double distance, double car_mass);

  // Route for which to create a Race Plan
  const std::shared_ptr<Route> route;
  // Attributes about the route
  std::vector<Coord> route_points;
  std::vector<std::pair<size_t, size_t>> cornering_segment_bounds;
  std::vector<double> cornering_speed_bounds;
  BasicLut route_distances;
  double max_route_speed;
  size_t num_points;

  // Maximum number of loops to create
  int max_num_loops;

  // If true, then max_num_loops will be created for any plan. Otherwise, a number will be sampled
  bool fix_loops;

  // Mass of the car in kg
  double car_mass;

  // Maximum speed of the car in m/s
  double max_speed;

  // Maximum instataneous motor power draw in W
  double max_motor_power;

  // Maximum magnitude for acceleration in m/s^2. This should be positive
  double max_acceleration;

  // Maximum deceleration in m/s^2. This should be negative
  double max_deceleration;

  // Minimum acceleration to consider when sampling in m/s^2
  double min_acceleration;

  // The target average speed of the car in m/s. Speed distributions
  // will be centered around this number
  double target_average_speed;

  // Start time of the race plan (Unused)
  Time start_time;

  // End time of the race plan (Unused)
  Time end_time;

  // Sampling seeds for sampling distributions
  unsigned speed_seed;
  unsigned loop_seed;
  unsigned aggressive_seed;
  unsigned idx_seed;
  unsigned acceleration_seed;
  unsigned skip_seed;

  // Minimum ratio of the maximum cornering speed to consider
  double corner_speed_min_ratio;

  // Maximum ratio of the maximum cornering speed to choose from
  double corner_speed_max_ratio;

  // Threshold value in meters for taking a straight aggressively. If a straight is longer than this
  // this distance, then the car will accelerate, hold a constant speed before decelerating
  double aggressive_straight_threshold;

  // Number of loops to repeat in a single loop block
  int num_repetitions;

  // Fraction of total motor power that can be allocated towards acceleration
  double acceleration_power_budget;

  // Amount of motor power in W that can be allocated towards acceleraiton
  double acceleration_power_allowance;

  // Number of iterations to try when sampling speeds, distances, indices, accelerations etc.
  double max_iters;

  // Whether to log the segmentation process to a file. Used for debugging
  bool log;

  // Number of corners in the route
  int num_corners;
};
