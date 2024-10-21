#include <stdlib.h>

#include <iostream>
#include <sstream>
#include <cassert>
#include <fstream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "utils/Defines.hpp"
#include "route/Route.hpp"
#include "config/Config.hpp"
#include "utils/Utilities.hpp"
#include "utils/Geography.hpp"

Route::Route(const std::string lut_path) {
  init_route(std::filesystem::path(lut_path));
}

Route::Route(const std::filesystem::path lut_path) {
  init_route(lut_path);
}

void Route::init_route(const std::filesystem::path route_path) {
  std::fstream base_route(route_path);
  RUNTIME_EXCEPTION(base_route.is_open(), "Base route file not found {}", route_path.string());

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
      RUNTIME_EXCEPTION(isDouble(cell), "Value {} in route file {} is not a number", cell, route_path.string());
      coord.lat = std::stod(cell);

      std::getline(linestream, cell, ',');
      RUNTIME_EXCEPTION(isDouble(cell), "Value {} in route file {} is not a number", cell, route_path.string());
      coord.lon = std::stod(cell);

      std::getline(linestream, cell, ',');
      RUNTIME_EXCEPTION(isDouble(cell), "Value {} in route file {} is not a number", cell, route_path.string());
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
  num_points = route_points.size();
  spdlog::info("Loaded base route {} with {} coordinates", route_path.string(), std::to_string(num_points));
}

void Route::init_control_stops() {
  control_stops = Config::get_instance()->get_control_stops();
}

/* Segment a route into uniform lengths */
std::vector<std::pair<size_t, size_t>> Route::segment_route_uniform(double length) {
  RUNTIME_EXCEPTION(route_points.size() > 0, "Route not yet loaded");

  std::vector<std::pair<size_t, size_t>> segments;

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

      segment_indices = {idx, idx+1};
      current_segment_distance = distance;
      difference = std::abs(length - current_segment_distance);
    } else if (current_segment_distance > length) {
      segments.push_back(segment_indices);

      segment_indices = {idx, idx};
      current_segment_distance = 0;
      difference = std::abs(length - current_segment_distance);
    }

    last_difference = difference;
  }

  if (segment_indices.first != segment_indices.second) {
    segments.push_back(segment_indices);
  }

  return segments;
}

void RacePlan::validate_members() const {
  RUNTIME_EXCEPTION(segments.size() == speed_profile.size() && segments.size() == acceleration_segments.size(),
                    "RacePlan not properly created. Speed profile, route segments and acceleration segments"
                    "have unequal lengths");
  const size_t num_segments = segments.size();
  for (size_t i=0; i < num_segments; i++) {
    RUNTIME_EXCEPTION(speed_profile[i] > 0.0 || acceleration_segments[i], "Speed profile must have >0 speeds");
  }

  RUNTIME_EXCEPTION(segments[0].first == 0, "Segments must start at index 0");
  for (size_t i=0; i < num_segments - 1; i++) {
    RUNTIME_EXCEPTION(segments[i].first < segments[i].second &&
                      segments[i].second == segments[i+1].first-1,
                      "Segments must cover all indices of the route and segment.first < segment.second");
  }
}

RacePlan::RacePlan(std::vector<std::pair<size_t, size_t>> segments, std::vector<double> speed_profile,
                   std::vector<bool> acceleration_segments)
                  : segments(segments), speed_profile(speed_profile) {
  if (acceleration_segments.size() == 0) {
    this->acceleration_segments = std::vector<bool>(segments.size(), false);
  }
  validate_members();
}

void RacePlan::set_segments(std::vector<std::pair<size_t, size_t>> new_segments) {
  segments = new_segments;
  validate_members();
}

void RacePlan::set_speed_profile(std::vector<double> new_speed_profile) {
  speed_profile = new_speed_profile;
  validate_members();
}

void RacePlan::set_acceleration_segments(std::vector<bool> new_acceleration_segments) {
  acceleration_segments = new_acceleration_segments;
  validate_members();
}
