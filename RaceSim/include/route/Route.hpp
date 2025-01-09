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

#include "utils/Units.hpp"

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

 public:
  RacePlan() {}
  RacePlan(std::vector<std::vector<std::pair<size_t, size_t>>> segments,
           std::vector<std::vector<std::pair<double, double>>> segment_speeds,
           std::vector<std::vector<bool>> acceleration_segments = {},
           std::vector<std::vector<double>> acceleration = {});

  inline std::vector<std::vector<std::pair<size_t, size_t>>> get_segments() const {return segments;}
  inline std::vector<std::vector<std::pair<double, double>>> get_speed_profile() const {return segment_speeds;}
  inline std::vector<std::vector<bool>> get_acceleration_segments() const {return acceleration_segments;}
  inline std::vector<std::vector<double>> get_acceleration_values() const {return acceleration;}
  inline std::string get_inviability_reason() const {return reason_for_inviability;}
  inline int get_num_loops() const {return num_loops;}
  inline bool is_viable() const {return viable;}

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

  /* Speed bounds */
  std::vector<double> max_speed;

 public:
  /* Read a CSV with columns |latitude|longitude|altitude(m)| */
  explicit Route(const std::string route_path);
  explicit Route(const std::filesystem::path route_path);

  void init_route(const std::filesystem::path route_path);
  void init_control_stops();

  /** @brief Read a csv of corner index bounds with columns |starting index|ending index|max speed(mps)|
   * @param cornering_bounds_path: Path to the csv file
   * @param max_car_speed: Maximum car speed to limit cornering speeds. Defaults to infinity
  */
  void init_cornering_bounds(const std::filesystem::path cornering_bounds_path,
                             double max_car_speed = std::numeric_limits<double>::infinity());

  Route() {}

  /** @brief Segment the route into uniform lengths
   * 
   * @param length The length of each segment. If the total length of the base route cannot be divided
   * into equal lengths, the last segment will be shorter than the rest
   */
  std::vector<std::pair<size_t, size_t>> segment_route_uniform(double length);

  /** @brief Segment the route assuming that the route completes one loop of the track
   * 
   * @param segment_idx_seed Random seed for selecting segment indices
   * @param speed_seed Random seed for selecting segment speeds
   */
  RacePlan segment_route_acceleration(const unsigned segment_idx_seed, const unsigned speed_seed);

  /* Getters */
  inline std::unordered_set<size_t> get_control_stops() const {return control_stops;}
  inline std::vector<Coord> get_route_points() const {return route_points;}
  inline size_t get_num_points() const {return num_points;}
  inline double get_route_length() const {return route_length;}
};
