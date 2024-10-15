/*
Class for representing the route
*/

#pragma once

#include <string.h>
#include <stdlib.h>
#include <vector>
#include <unordered_set>

#include "utils/Units.hpp"

class Route {
 protected:
  /* Number of segments to split the route up into */
  size_t num_segments;

  /* Starting and Ending indices of all segments */
  std::vector<std::pair<size_t, size_t>> segments;

  /* Length of each segment */
  std::vector<double> segment_lengths;

  /* Indices of all control stops */
  std::unordered_set<size_t> control_stops;

  /* Points of the route */
  std::vector<Coord> route_points;

  /* Number of points along the route */
  size_t num_points;

  /* Total length of the route */
  double route_length;

 public:
  /* Initialize the vector of route points */
  Route();

  /* Segment the route into uniform lengths */
  void segment_route_uniform(double length);

  /* Getters */
  inline size_t get_num_segments() const {return num_segments;}
  inline std::vector<std::pair<size_t, size_t>> get_segments() const {return segments;}
  inline std::vector<double> get_segment_lengths() const {return segment_lengths;}
  inline std::unordered_set<size_t> get_control_stops() const {return control_stops;}
  inline std::vector<Coord> get_route_points() const {return route_points;}
  inline size_t get_num_points() const {return num_points;}
};
