/**
 * Classes used to represent a race route and proposed race plans
 */

#pragma once

#include <stdlib.h>

#include <string>
#include <filesystem>
#include <vector>
#include <unordered_set>
#include <limits>
#include <utility>

#include "utils/Units.hpp"
#include "utils/Luts.hpp"

/** A class to represent a proposed race plan
 * 
 * Each plan splits the route into segments described by a series of {start index, end index} pairs.
 * Each segment is assigned either a constant speed that the car maintains or it is designated as an
 * acceleration segment. This means that the the car will accelerate from the initial speed to the
 * final speed.
 */
class RacePlan {
 private:
  /* Viability of race plan */
  bool viable = false;

  /* Time taken to complete the race in seconds using this plan */
  time_t time_taken;

  /* 2D matrix where each row is a single loop of the track split into segments of
  {start index, end index} pairs. Both indices are inclusive */
  std::vector<std::vector<std::pair<size_t, size_t>>> segments;

  /* 2D matrix where each row consists of the {speed at start index in m/s, speed at end index in m/s}
  for each segment of the specific loop of the track. Order is the same as segments. Note that if
  segment_speeds[loop][segment].second = segment_speeds[loop][segment].first, then the car
  travels at a constant speed throughout the segment */
  std::vector<std::vector<std::pair<double, double>>> segment_speeds;

  /* 2D matrix where each row is a single loop of the track. If acceleration_segments[loop][segment]
  is true, then the car is accelerating starting from segment_speeds[loop][segment].first to
  segment_speeds[loop][segment].second at acceleration[loop][segment] */
  std::vector<std::vector<bool>> acceleration_segments;

  /* 2D matrix where each row is a single loop of the track. If acceleration_segments[loop][segment]
  is true, then the car accelerates at a rate of acceleration[loop][segment] m/s */
  std::vector<std::vector<double>> acceleration;

  // Note that acceleration.size() = acceleration_segments.size() = segments.size() = segment_speeds.size()
  // AND acceleration[i].size() = acceleration_segments[i].size() = segments[i].size() = segment_speeds[i].size()
  // for all i in the range of [0, acceleration.size()-1]

  // Number of loops completed by the car i.e. segments.size()
  int num_loops;

  // If the plan is not viable, this string holds the reason
  std::string reason_for_inviability = "";

  // Represents an empty race plan
  bool empty;

 public:
  RacePlan() : empty(true) {}
  RacePlan(std::vector<std::vector<std::pair<size_t, size_t>>> segments,
           std::vector<std::vector<std::pair<double, double>>> segment_speeds,
           std::vector<std::vector<bool>> acceleration_segments = {},
           std::vector<std::vector<double>> acceleration = {});
  explicit RacePlan(std::string inviability_reason) : empty(true) {
    reason_for_inviability = inviability_reason; viable = false;
  }

  inline std::vector<std::vector<std::pair<size_t, size_t>>> get_segments() const {return segments;}
  inline std::vector<std::vector<std::pair<double, double>>> get_speed_profile() const {return segment_speeds;}
  inline std::vector<std::vector<bool>> get_acceleration_segments() const {return acceleration_segments;}
  inline std::vector<std::vector<double>> get_acceleration_values() const {return acceleration;}
  inline std::string get_inviability_reason() const {return reason_for_inviability;}
  inline int get_num_loops() const {return num_loops;}
  inline bool is_viable() const {return viable;}
  inline bool is_empty() const {return empty;}

  inline void set_segments(std::vector<std::vector<std::pair<size_t, size_t>>> new_segments) {segments = new_segments;}
  inline void set_speed_profile(std::vector<std::vector<std::pair<double, double>>> new_speed_profile) {
    segment_speeds = new_speed_profile;
  }
  inline void set_acceleration_segments(std::vector<std::vector<bool>> new_acceleration_segments) {
    acceleration_segments = new_acceleration_segments;
  }
  inline void set_time_taken(time_t new_time_taken) {time_taken = new_time_taken;}
  inline void set_viability(bool viability) {viable = viability;}
  inline void set_num_loops(int loops) {num_loops = loops;}
  inline void set_inviability_reason(std::string message) {reason_for_inviability = message;}

  /** @brief Validate members of a race plan. Should be called before run_sim()
   *
   * @param route_points Coordinate points of the base route
   * @return True if all members are valid
   */
  bool validate_members(const std::vector<Coord>& route_points) const;

  /** @brief Print the route plan to stdout */
  void print_plan() const;
};

/** A class to hold all points and control stop locations in a race route */
class Route {
 private:
  /* Indices of all control stops */
  std::unordered_set<size_t> control_stops;

  /* Points of the route */
  std::vector<Coord> route_points;

  /* Number of points along the route */
  size_t num_points;

  /* Total length of the route */
  double route_length;

  /* Cornering segments */
  std::vector<std::pair<size_t, size_t>> cornering_segment_bounds;

  /* Maximum speed of cornering segments */
  std::vector<double> cornering_speed_bounds;

  /* Lookup table for all distances between any two points. Should be num_points x num_points*/
  BasicLut route_distances;

  /* Longest straight path */
  double longest_straight;

  /* Maximum speed of the route (mps) - comes from regulations */
  double max_route_speed;

  /** @brief Helper function to segment_route_acceleration that determines the set of speeds
   * valid for a particular acceleration to not exceed the maximum motor power
   *
   * @param acceleration acceleration in m/s^2
   * @param mass Mass of the car in kg
   * @param max_car_speed Maximum speed of the car in m/s
   * @param max_motor_power Maximum motor power in W
   * @return {min_speed, max_speed} inclusive on both sides for the range of viable speeds
   * If no speeds can be found, return {-1, -1}
   */
  std::pair<double, double> find_valid_speeds(const double& acceleration, const double& mass,
    const double& max_car_speed, const double& max_motor_power);

  /** @brief Helper function to segment_route_acceleration to determine if an acceleration a < max_bound exists
  * such that given some initial speed, distance to cover and acceptable ending speed range, the acceleration will
  * take the initial speed to some final speed within the acceptable range
  *
  * @param init_speed Initial speed in m/s
  * @param distance Distance to cover in m
  * @param speed_range Range of [min_speed, max_speed] inclusive on both sides. Units in m/s
  * @param deceleration_bound Maximum deceleration magnitude in m/s^2
  * @param acceleration_bound Maximum acceleration magnitude in m/s^2
  */
  bool does_acceleration_exist(double init_speed, double distance,
                                std::pair<double, double> speed_range,
                                double deceleration_bound,
                                double acceleration_bound);

 public:
  /** @brief Load information about the race route
   * 
   * @param route_path: Absolute path to the base route csv
   * @param init_control_stops: Initialize control stop indices from config file
   * @param cornering_bounds_path: Absolute path to the cornering bounds. If empty, don't load
   * @param precomputed_distances_path: Absolute path to the precomputed distances csv. If empty or precompute_distances
   * is true, don't load
   * @param precompute_distances: Whether to pre-compute distances. If precomputed_distances_path is not empty, it
   * will save the csv to that path
   */
  Route(const std::filesystem::path route_path, const bool init_control_stops = false,
        const std::filesystem::path cornering_bounds_path = std::filesystem::path(),
        const std::filesystem::path precomputed_distances_path = std::filesystem::path(),
        const bool precompute_distances = false);

  /** @brief initial the base route csv */
  void init_base_route(const std::filesystem::path route_path);

  /** @brief Initialize control stops */
  void init_control_stops();

  /** @brief Read a csv of corner index bounds with columns |starting index|ending index|max speed(mps)|
   * @param cornering_bounds_path: Path to the csv file
   * @param max_car_speed: Maximum car speed to limit cornering speeds. Defaults to infinity
  */
  void init_cornering_bounds(const std::filesystem::path cornering_bounds_path,
                             double max_car_speed = std::numeric_limits<double>::infinity());

  /** @brief Read a num_points x num_points csv of pre-computed distances */
  void init_precomputed_distances(const std::filesystem::path precomputed_distances_path);

  /** @brief Find the longest straight in the route */
  void init_longest_straight();

  /** @brief Pre-compute distances between every possible pair of indices inside the route
   * 
   * @param csv_path: Path for the save location of the pre-computed distance csv
   */
  void precompute_distances(const std::filesystem::path csv_path = std::filesystem::path());

  Route() {}

  /** @brief Segment the route into uniform lengths
   * 
   * @param length The length of each segment. If the total length of the base route cannot be divided
   * into equal lengths, the last segment will be shorter than the rest
   */
  std::vector<std::pair<size_t, size_t>> segment_route_uniform(double length);

  /** @brief Segment the route into a certain number of loops
   * 
   * @param segment_idx_seed Random seed for selecting segment indices
   * @param speed_seed Random seed for selecting segment speeds
   * @param acceleration_seed Random seed for selecting accelerations
   * @param skip_seed Random seed for skipping acceleration segments
   * @param loop_seed Random seed for selecting the number of loops to complete
   * @param max_num_loops Maximum possible number of loops around the track
   * @param car_mass Car mass in kg
   * @param max_speed Maximum speed of the car in m/s
   * @param max_motor_power Maximum instataneous motor power draw from the motor
   * @param max_acceleration Maximum magnitude for acceleration in m/s^2
   * @param max_deceleration Maximum magnitude for deceleration ini m/s^2
   * @param max_iterations Maximum number of iterations when searching for speeds/accelerations/indices
   * This is to prevent hanging from infinite loops
   * 
   * @return Empty race plan if segment was unsuccessful. Valid race plan otherwise
   */
  RacePlan segment_route_acceleration(const unsigned segment_idx_seed,
                                      const unsigned speed_seed,
                                      const unsigned acceleration_seed,
                                      const unsigned skip_seed,
                                      const unsigned loop_seed,
                                      const int max_num_loops,
                                      const double car_mass,
                                      const double max_speed,
                                      const double max_motor_power,
                                      const double max_acceleration,
                                      const double max_deceleration,
                                      const int max_iters = 1000);

  /** @brief Segment a rourte by assigning corner speeds
   *
   * @param speed_seed Random seed for selecting speeds
   * @param loop_seed Random seed for selecting number of loops to complete
   * @param aggressive_seed Random seed for taking a straight aggressively
   * @param idx_seed Random seed for selecting route indices
   * @param max_num_loops Maximum number of loops
   * @param car_mass Mass of the car in kg
   * @param max_speed Maximum speed of the car in m/s
   * @param max_motor_power Maximum instataneous motor power draw from the motor in W
   * @param max_acceleration Maximum magnitude for acceleration in m/s^2
   * @param max_deceleration Maximum magnitude for deceleration in m/s^2
   * @param preferred_acceleration Magnitude of preferred acceleration value
   * @param preferred_deceleration Magnitude of preferred deceleration value
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
   */
  RacePlan segment_route_corners(const int max_num_loops,
                                 const double car_mass,
                                 const double max_speed,
                                 const double max_motor_power,
                                 const double max_acceleration,
                                 const double max_deceleration,
                                 const double preferred_acceleration = 1.0,
                                 const double preferred_deceleration = 1.0,
                                 const unsigned speed_seed = 1,
                                 const unsigned loop_seed = 1,
                                 const unsigned aggressive_seed = 1,
                                 const unsigned idx_seed = 1,
                                 const unsigned acceleration_seed = 1,
                                 const double corner_speed_min = 0.05,
                                 const double corner_speed_max = 0.9,
                                 const double aggressive_straight_threshold = 200.0,
                                 const int num_repetitions = 5,
                                 const double acceleration_power_budget = 0.5,
                                 const int max_iters = 1000,
                                 bool log = true);

  /** @brief Find segment length
   *
   * @param start_idx Start index of the segment
   * @param end_idx End index of the segment
   */
  double get_segment_length(const size_t start_idx, const size_t end_idx) const;

  /* Getters */
  inline std::unordered_set<size_t> get_control_stops() const {return control_stops;}
  inline const std::vector<Coord>& get_route_points() {return route_points;}
  inline std::vector<Coord> get_route_points() const {return route_points;}
  inline size_t get_num_points() const {return num_points;}
  inline double get_route_length() const {return route_length;}
  inline const BasicLut& get_precomputed_distances() {return route_distances;}
};
