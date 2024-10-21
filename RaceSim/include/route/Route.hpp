/**
 * Classes used to represent a race route and proposed race plans
 */

#pragma once

#include <stdlib.h>

#include <string>
#include <filesystem>
#include <vector>
#include <unordered_set>

#include "utils/Units.hpp"


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

 public:
  /* Read a CSV with columns |latitude|longitude|altitude| */
  explicit Route(const std::string route_path);
  explicit Route(const std::filesystem::path route_path);

  void init_route(const std::filesystem::path route_path);
  void init_control_stops();
  Route() {}

  /* Segment the route into uniform lengths */
  std::vector<std::pair<size_t, size_t>> segment_route_uniform(double length);

  /* Getters */
  inline std::unordered_set<size_t> get_control_stops() const {return control_stops;}
  inline std::vector<Coord> get_route_points() const {return route_points;}
  inline size_t get_num_points() const {return num_points;}
  inline double get_route_length() const {return route_length;}
};

/** A class to represent a proposed race plan
 * 
 * Each plan splits the route into segments described by a series of {start index, end index} pairs.
 * Each segment is assigned either a constant speed that the car maintains or it is designated as an
 * acceleration segment. This means that the the car will accelerate from the final speed of the
 * previous segment to the initial speed of the next segment
 */
class RacePlan {
 private:
  bool viable = false;

  /* Time taken to complete the race in seconds using this plan */
  double time_taken;

  /* Segments of the route split into a series of {start index, end index} */
  std::vector<std::pair<size_t, size_t>> segments;

  /* Speed assigned to each segment */
  std::vector<double> speed_profile;

  // Whether a segment is an acceleration segment. If acceleration_segments[i] = true,
  // then speed_profile[i] is effectively ignored
  std::vector<bool> acceleration_segments;

  // Validate member variables
  void validate_members() const;

 public:
  RacePlan() {}
  RacePlan(std::vector<std::pair<size_t, size_t>> segments, std::vector<double> speed_profile,
           std::vector<bool> acceleration_segments = {});

  inline std::vector<std::pair<size_t, size_t>> get_segments() const {return segments;}
  inline std::vector<double> get_speed_profile() const {return speed_profile;}
  inline std::vector<bool> get_acceleration_segments() const {return acceleration_segments;}
  inline bool is_viable() const {return viable;}

  void set_segments(std::vector<std::pair<size_t, size_t>> new_segments);
  void set_speed_profile(std::vector<double> new_speed_profile);
  void set_acceleration_segments(std::vector<bool> new_acceleration_segments);
  inline void set_viability(bool viability) {viable = viability;}
};
