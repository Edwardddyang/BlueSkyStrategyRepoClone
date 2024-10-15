#include <stdlib.h>

#include <iostream>
#include <sstream>
#include <cassert>
#include <fstream>
#include <limits>
#include <filesystem>
#include <string>
#include <utility>

#include "utils/Defines.hpp"
#include "route/Route.hpp"
#include "config/Config.hpp"
#include "utils/Utilities.hpp"
#include "utils/Geography.hpp"

/* Load the base route */
Route::Route() {
  Config* params = Config::get_instance();
  std::string strat_root = Config::get_strat_root();
  std::string route_path;
  if (strat_root == "") {
    route_path = params->get_base_route_path();
  } else {
    route_path = (std::filesystem::path(strat_root) / params->get_base_route_path()).string();
  }
  std::fstream base_route(route_path);

  RUNTIME_EXCEPTION(base_route.is_open(), "Base route file not found {}", route_path);

  route_length = 0.0;
  Coord last_coord;

  bool first_coord = true;

  // Read and parse the file
  while (!base_route.eof()) {
    std::string line;
    base_route >> line;
    std::stringstream linestream(line);

    while (!linestream.eof() && !linestream.str().empty()) {
      std::string cell;
      Coord coord{};

      std::getline(linestream, cell, ',');
      RUNTIME_EXCEPTION(isDouble(cell), "Value {} in route file {} is not a number", cell, route_path);
      coord.lat = std::stod(cell);

      std::getline(linestream, cell, ',');
      RUNTIME_EXCEPTION(isDouble(cell), "Value {} in route file {} is not a number", cell, route_path);
      coord.lon = std::stod(cell);

      std::getline(linestream, cell, ',');
      RUNTIME_EXCEPTION(isDouble(cell), "Value {} in route file {} is not a number", cell, route_path);
      coord.alt = std::stod(cell);

      route_points.emplace_back(coord);

      if (!first_coord) {
        route_length = route_length + get_distance(last_coord, coord);
      } else {
        first_coord = false;
      }

      last_coord = coord;
    }
  }

  /* Segment the route */
  num_points = route_points.size();
  if (Config::get_instance()->get_optimizer() == "Constant") {
    segment_route_uniform(route_length);
  }

  control_stops = Config::get_instance()->get_control_stops();
  spdlog::info("Loaded base route {} with {} coordinates", route_path, std::to_string(num_points));
}

/* Segment a route into uniform lengths */
void Route::segment_route_uniform(double length) {
  RUNTIME_EXCEPTION(route_points.size() > 0, "Route not yet loaded");

  // Clear segments data
  segment_lengths.clear();
  segments.clear();

  // Create segments
  double current_segment_distance = 0.0;
  double difference = 0.0;  // Difference between target segment length and current_segment_distance
  double last_difference = std::numeric_limits<double>::max();
  std::pair<size_t, size_t> segment_indices = {0, 0};

  for (size_t idx=0; idx < num_points-1; idx++) {
    Coord coord_one = route_points[idx];
    Coord coord_two = route_points[idx+1];

    segment_indices.second = idx + 1;
    double distance = get_distance(coord_one, coord_two);
    current_segment_distance += distance;

    difference = std::abs(length - current_segment_distance);

    if (difference > last_difference) {  // Is this condition ever met?
      segment_indices.second = idx;
      segments.push_back(segment_indices);
      segment_lengths.push_back(current_segment_distance);

      segment_indices = {idx, idx+1};
      current_segment_distance = distance;
      difference = std::abs(length - current_segment_distance);
    } else if (current_segment_distance > length) {
      segments.push_back(segment_indices);
      segment_lengths.push_back(current_segment_distance);

      segment_indices = {idx, idx};
      current_segment_distance = 0;
      difference = std::abs(length - current_segment_distance);
    }

    last_difference = difference;
  }

  if (segment_indices.first != segment_indices.second) {
    segments.push_back(segment_indices);
    segment_lengths.push_back(current_segment_distance);
  }

  num_segments = segments.size();
  return;
}
