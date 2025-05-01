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
// described in
// https://www.notion.so/blueskysolar/Race-Strategy-and-Testing-Process-1da8d1f46c3680a79056edf3c9fecd1b?pvs=4
class RacePlanCreator {
 public:
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
