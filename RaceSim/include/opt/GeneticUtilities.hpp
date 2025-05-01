/** This file provides functions for crossover, mutation and initial population creation */

#pragma once

#include <memory>

#include "route/Route.hpp"
#include "utils/CustomTime.hpp"

namespace Genetic {

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

/** @brief Mutate a race plan according to a strategy chosen from configuration */
RacePlan mutate_plan(RacePlan plan, const std::shared_ptr<Route> route);

/** @brief Create a Race plan by segmenting a route as described in
  * https://www.notion.so/blueskysolar/Race-Strategy-and-Testing-Process-1da8d1f46c3680a79056edf3c9fecd1b?pvs=4
  *
  * @param route Route for which to create a Race Plan
  * @param max_num_loops Maximum number of loops to create
  * @param fix_loops If true, then max_num_loops will be created. Otherwise, a number will be sampled
  * @param speed_seed Random seed for selecting speeds
  * @param loop_seed Random seed for selecting number of loops to complete
  * @param aggressive_seed Random seed for taking a straight aggressively
  * @param idx_seed Random seed for selecting route indices
  * @param acceleration_seed Random seed for selecting acceleration
  * @param skip_seed Random seed for maintaining constant speeds on existing corners
  * @param max_num_loops Maximum number of loops
  * @param car_mass Mass of the car in kg
  * @param max_speed Maximum speed of the car in m/s
  * @param max_motor_power Maximum instataneous motor power draw from the motor in W
  * @param max_acceleration Maximum magnitude for acceleration in m/s^2
  * @param max_deceleration Maximum magnitude for deceleration in m/s^2
  * @param start_time Start time of the race plan
  * @param end_time End time of the race plan
  * @param preferred_acceleration Magnitude of preferred acceleration value
  * @param preferred_deceleration Magnitude of preferred deceleration value
  * @param min_acceleration Minimum acceleration value to consider
  * @param average_speed Average speed to target
  * @param corner_speed_min The fractional value to clamp the lower bound of speed ranges
  * @param corner_speed_max The fractional value to clamp the upper bound of speed ranges
  * @param aggressive_straight_threshold Threshold value in meters to take a straight aggressively
  * This means accelerating out of a corner and decelerating into the next corner
  * @param num_repetitions Number of loops to repeat in a single chunk
  * @param acceleration_power_budget Fraction of total motor power that can be allocated to
  * acceleration
  * @param max_iters Number of iterations to try selecting a corner speed until we give up and return
  * empty
  * @param log Whether to log the segmentation process to a file
  *
  * Note: This function can only create a plan for a single day at a time
  */
RacePlan segment_route_corners(const std::shared_ptr<Route> route,
                                const int max_num_loops,
                                bool fix_loops,
                                const double car_mass,
                                const double max_speed,
                                const double max_motor_power,
                                const double max_acceleration,
                                const double max_deceleration,
                                const double min_acceleration,
                                const double average_speed,
                                const Time* start_time,
                                const Time* end_time,
                                const double preferred_acceleration = 1.0,
                                const double preferred_deceleration = -1.0,
                                const unsigned speed_seed = 1,
                                const unsigned loop_seed = 1,
                                const unsigned aggressive_seed = 1,
                                const unsigned idx_seed = 1,
                                const unsigned acceleration_seed = 1,
                                const unsigned skip_seed = 1,
                                const double corner_speed_min = 0.2,
                                const double corner_speed_max = 0.9,
                                const double aggressive_straight_threshold = 100.0,
                                const int num_repetitions = 5,
                                const double acceleration_power_budget = 0.7,
                                const int max_iters = 1000,
                                bool log = false);
}; // namespace Genetic
