/** This file provides functions for crossover, mutation and initial population creation */

#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "route/Route.hpp"
#include "utils/CustomTime.hpp"

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
  // Holds the intermediate segment data when creating a race plan
  struct IntermediateSegmentData {
    std::pair<size_t, size_t> segment;
    std::pair<double, double> segment_speed;
    bool acceleration;
    double acceleration_value;
    double segment_distance;

    IntermediateSegmentData(
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
  struct IntermediateLoopData {
    std::vector<std::pair<size_t, size_t>> loop_segments;
    std::vector<std::pair<double, double>> loop_segment_speeds;
    std::vector<bool> loop_acceleration_segments;
    std::vector<double> loop_acceleration_values;
    std::vector<double> loop_segment_distances;

    void clear() {
      loop_segments.clear();
      loop_segment_speeds.clear();
      loop_acceleration_segments.clear();
      loop_acceleration_values.clear();
      loop_segment_distances.clear();
    }

    IntermediateLoopData(
      std::vector<std::pair<size_t, size_t>> loop_segments = {},
      std::vector<std::pair<double, double>> loop_segment_speeds = {},
      std::vector<bool> loop_acceleration_segments = {},
      std::vector<double> loop_acceleration_values = {},
      std::vector<double> loop_segment_distances = {}) : loop_segments(loop_segments),
        loop_segment_speeds(loop_segment_speeds),
        loop_acceleration_segments(loop_acceleration_segments),
        loop_acceleration_values(loop_acceleration_values),
        loop_segment_distances(loop_segment_distances) {}
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

  // Number of iterations to try when sampling speeds, distances, indices, accelerations etc.
  double max_iters;

  // Whether to log the segmentation process to a file. Used for debugging
  bool log;
};
