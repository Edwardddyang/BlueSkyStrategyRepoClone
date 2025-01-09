#include <stdlib.h>

#include <iostream>
#include <sstream>
#include <cassert>
#include <fstream>
#include <limits>
#include <string>
#include <random>
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

RacePlan Route::segment_route_acceleration(const unsigned segment_idx_seed, const unsigned speed_seed,
                                           const int max_num_loops, const double max_speed) {
  RUNTIME_EXCEPTION(route_points.size() > 0, "Route points not yet loaded");
  RUNTIME_EXCEPTION(cornering_segment_bounds.size() > 0, "Cornering bounds CSV not yet read");
  RUNTIME_EXCEPTION(cornering_segment_bounds.size() == cornering_speed_bounds.size(),
                    "Number of corners must equal the length of the cornering speeds");
  std::mt19937 idx_rng(segment_idx_seed);
  std::mt19937 speed_rng(speed_seed);

  // RacePlan attributes
  std::vector<std::pair<size_t, size_t>> segments;
  std::vector<std::pair<double, double>> segment_speeds;
  std::vector<bool> acceleration_segments;
  std::vector<double> acceleration_values;

  // Randomly select the number of loops to complete
  std::mt19937 gen(1);
  std::uniform_int_distribution<unsigned int> dis(1, max_num_loops);
  const size_t num_loops = static_cast<size_t>(dis(gen));
  std::cout << "Num loops: " << num_loops << std::endl;
  // Iterate through the cornering intervals. For each one, we must create three/four segments
  // 1. Constant speed segment
  // 2. Deceleration/Acceleration segment going into the corner
  // 3. Constant speed segment for taking the corner
  // 4. Acceleration/Deceleration segment leaving the corner
  // Afterwards, we fill in the remaining segments with constant speed
  std::pair<size_t, size_t> last_segment;
  std::pair<double, double> last_segment_speed;
  const size_t num_corners = cornering_segment_bounds.size();
  std::uniform_int_distribution<size_t> idx_dist;
  std::uniform_real_distribution<double> speed_dist;
  for (size_t corner_idx=0; corner_idx < num_corners; corner_idx++) {
    const size_t corner_start = cornering_segment_bounds[corner_idx].first;
    const size_t corner_end = cornering_segment_bounds[corner_idx].second;

    // Create deceleration/acceleration segment coming into the corner
    // Note that std::uniform_int_distribution is inclusive on both sides

    // Choose some random starting point between the last segment's ending index
    // and the start of the next corner
    if (corner_idx == 0) {
      idx_dist = std::uniform_int_distribution<size_t>(0, corner_start - 1);
    } else {
      idx_dist = std::uniform_int_distribution<size_t>(last_segment.second, corner_start - 1);
    }
    const size_t chosen_start_idx = idx_dist(idx_rng);
    const double deceleration_starting_speed = last_segment_speed.second;

    // If this isn't the first corner, then we need to create a segment going from the last acceleration
    // segment to the current deceleration segment
    if (corner_idx > 0) {
      segments.push_back({last_segment.second, chosen_start_idx});
      segment_speeds.push_back({deceleration_starting_speed, deceleration_starting_speed});
      acceleration_segments.push_back(false);
      acceleration_values.push_back(0.0);
    }

    segments.push_back({chosen_start_idx, corner_start});

    const double max_corner_speed = cornering_speed_bounds[corner_idx];
    speed_dist = std::uniform_real_distribution<double>(10.0, max_corner_speed);
    const double corner_speed = speed_dist(speed_rng);
    segment_speeds.push_back({deceleration_starting_speed, corner_speed});

    acceleration_segments.push_back(!(corner_speed == deceleration_starting_speed));
    double accumulated_distance = 0.0;
    size_t num_segment_points = corner_start - chosen_start_idx;
    for (size_t i=0; i < num_segment_points; i++) {
      accumulated_distance += get_distance(route_points[chosen_start_idx + i],
                                           route_points[chosen_start_idx + i + 1]);
    }
    acceleration_values.push_back((corner_speed * corner_speed -
                                   deceleration_starting_speed * deceleration_starting_speed) /
                                   (2.0 * accumulated_distance));

    // Create the constant speed segment for taking the corner
    segments.push_back({corner_start, corner_end});
    segment_speeds.push_back({corner_speed, corner_speed});
    acceleration_segments.push_back(false);
    acceleration_values.push_back(0.0);

    // Create acceleration/deceleration segment coming out of the corner
    // TODO(Somebody): A copy of this function should be made such that there is a chance
    // that acceleration does not happen, and the car simply continues travelling at the
    // speed at which it took the corner. This will be common for the latter half of the track
    // where the entire thing is pretty much a bend
    if (corner_idx == num_corners - 1) {
      idx_dist = std::uniform_int_distribution<size_t>(corner_end + 1, num_points - 1);
      if (corner_end + 1 >= num_points - 1) {
        break;
      }
    } else {
      const size_t next_corner_start = cornering_segment_bounds[corner_idx + 1].first;
      idx_dist = std::uniform_int_distribution<size_t>(corner_end + 1, next_corner_start - 1);
    }
    const size_t chosen_end_idx = idx_dist(idx_rng);
    segments.push_back({corner_end, chosen_end_idx});

    // Accelerate up to any speed from 1 to the maximum speed of the car
    speed_dist = std::uniform_real_distribution<double>(1.0, max_speed);
    const double segment_ending_speed = speed_dist(speed_rng);
    segment_speeds.push_back({corner_speed, segment_ending_speed});
    acceleration_segments.push_back(!(corner_speed == segment_ending_speed));
    accumulated_distance = 0.0;
    num_segment_points = chosen_end_idx - corner_end;
    for (size_t i=0; i < num_segment_points; i++) {
      accumulated_distance += get_distance(route_points[corner_end + i], route_points[corner_end + i + 1]);
    }
    acceleration_values.push_back((segment_ending_speed * segment_ending_speed -
                                   corner_speed * corner_speed) /
                                   (2.0 * accumulated_distance));

    last_segment = {corner_end, chosen_end_idx};
    last_segment_speed = {corner_speed, segment_ending_speed};
  }
  return RacePlan({segments}, {segment_speeds}, {acceleration_segments}, {acceleration_values});
}

void Route::init_cornering_bounds(const std::filesystem::path cornering_bounds_path,
                                  double max_car_speed) {
  cornering_segment_bounds.resize(0);
  std::fstream csv(cornering_bounds_path);
  RUNTIME_EXCEPTION(csv.is_open(), "Base route file not found {}", cornering_bounds_path.string());

  bool first_coord = true;
  std::string starting_bound;
  std::string ending_bound;
  std::string max_speed;
  std::pair<size_t, size_t> last_bound;
  // Read and parse the file
  bool is_first_segment = true;
  while (!csv.eof()) {
    std::string line;
    csv >> line;
    std::stringstream linestream(line);

    std::getline(linestream, starting_bound, ',');
    std::getline(linestream, ending_bound, ',');
    std::getline(linestream, max_speed, ',');
    std::pair<size_t, size_t> bound;
    RUNTIME_EXCEPTION(isSizeT(starting_bound, &bound.first), "Value {} in cornering bounds file {} is not a number",
                      starting_bound, cornering_bounds_path.string());
    RUNTIME_EXCEPTION(isSizeT(ending_bound, &bound.second), "Value {} in cornering bounds file {} is not a number",
                      ending_bound, cornering_bounds_path.string());
    RUNTIME_EXCEPTION((is_first_segment && bound.first < bound.second) || (!is_first_segment &&
                      bound.first > last_bound.second), "Cornering bounds are degenerate");
    RUNTIME_EXCEPTION(isDouble(max_speed), "Value {} in cornering bounds file {} is not a number",
                      max_speed, cornering_bounds_path.string());
    double max_corner_speed = std::stod(max_speed) < max_car_speed ? std::stod(max_speed) : max_car_speed;
    cornering_speed_bounds.push_back(max_corner_speed);
    cornering_segment_bounds.push_back(bound);
    last_bound = bound;
    is_first_segment = false;
  }
}

void RacePlan::print_plan() const {
  std::cout << "---------Segments---------" << std::endl;
  for (size_t i=0; i < segments.size(); i++) {
    std::cout << "Loop " << i+1 << " -> ";
    for (size_t j=0; j < segments[i].size(); j++) {
      std::cout << "[" << segments[i][j].first << "," << segments[i][j].second << "]";
      if (j == segments[i].size()-1) {
        std::cout << "\n";
      } else {
        std::cout << ",";
      }
    }
  }
  std::cout << "------Segment Speeds------" << std::endl;
  for (size_t i=0; i < segment_speeds.size(); i++) {
    std::cout << "Loop " << i+1 << " -> ";
    for (size_t j=0; j < segment_speeds[i].size(); j++) {
      std::cout << "[" << segment_speeds[i][j].first << "," << segment_speeds[i][j].second << "]";
      if (j == segment_speeds[i].size()-1) {
        std::cout << "\n";
      } else {
        std::cout << ",";
      }
    }
  }
  std::cout << "-------Acceleration-------" << std::endl;
  for (size_t i=0; i < acceleration.size(); i++) {
    std::cout << "Loop " << i+1 << " -> ";
    for (size_t j=0; j < acceleration[i].size(); j++) {
      std::cout << "[" << acceleration[i][j] << "]";
      if (j == acceleration[i].size()-1) {
        std::cout << "\n";
      } else {
        std::cout << ",";
      }
    }
  }
}

bool RacePlan::validate_members(const std::vector<Coord>& route_points) const {
  RUNTIME_EXCEPTION(segments.size() > 0, "Validate_members() for RacePlan called before segments were set.");
  RUNTIME_EXCEPTION(segments.size() == segment_speeds.size() && segments.size() == acceleration_segments.size() &&
                    acceleration_segments.size() == acceleration.size(),
                    "RacePlan not properly created. Speed profile, route segments and acceleration segments"
                    "have unequal number of loops");
  const size_t num_loops = segments.size();
  for (size_t loop_idx=0; loop_idx < num_loops; loop_idx++) {
    const std::vector<std::pair<size_t, size_t>>& loop_segments = segments[loop_idx];
    const std::vector<std::pair<double, double>>& loop_segments_speeds = segment_speeds[loop_idx];
    const std::vector<bool>& loop_segments_acceleration = acceleration_segments[loop_idx];
    const std::vector<double>& loop_segments_acceleration_values = acceleration[loop_idx];

    RUNTIME_EXCEPTION(loop_segments.size() == loop_segments_speeds.size() &&
                      loop_segments.size() == loop_segments_acceleration.size() &&
                      loop_segments.size() == loop_segments_acceleration_values.size(),
                      "Segment speeds, acceleration and indices must be the same length for each loop");

    const size_t num_segments = loop_segments.size();
    // Don't have this check in case we are starting at a different start point. More relevant to WSC than
    // FSGP
    // RUNTIME_EXCEPTION(loop_segments[0].first == 0, "Segments must start at index 0");
    RUNTIME_EXCEPTION(loop_segments[num_segments-1].second == route_points.size()-1, "Last segment's ending point must "
                                                                                "be the last index of the route");
    for (size_t i=0; i < num_segments; i++) {
      RUNTIME_EXCEPTION(loop_segments_speeds[i].first >= 0.0 && loop_segments_speeds[i].second >= 0.0,
                        "Segment speed in segment {} is not positive", i);

      RUNTIME_EXCEPTION(loop_segments[i].first <= loop_segments[i].second, "Segment start must be <= segment end");
      if (i < num_segments-1) {
        RUNTIME_EXCEPTION(loop_segments[i].second == loop_segments[i+1].first,
                          "Segments must be continuous i.e. segment end = next segment start."
                          " Invalid on segment {}", i);
        RUNTIME_EXCEPTION(loop_segments_speeds[i].second == loop_segments_speeds[i+1].first,
                          "Ending speed of segment {} must equal starting speed of next segment", i);
      }

      if (!loop_segments_acceleration[i]) {
        RUNTIME_EXCEPTION(loop_segments_speeds[i].first == loop_segments_speeds[i].second,
                          "Speed profile for non-acceleration segment {} must have equal and positive starting "
                          "and ending speeds", i);
      } else {
        const double acceleration_value = loop_segments_acceleration_values[i];
        const double starting_speed = loop_segments_speeds[i].first;
        const double ending_speed = loop_segments_speeds[i].second;
        // Get distance travelled during this segment
        double accumulated_distance = 0.0;
        const size_t num_segment_points = loop_segments[i].second - loop_segments[i].first;
        for (size_t j=0; j < num_segment_points; j++) {
          const Coord src_point = route_points[loop_segments[i].first + j];
          const Coord dest_point = route_points[loop_segments[i].first + j + 1];
          accumulated_distance += get_distance(src_point, dest_point);
        }
        RUNTIME_EXCEPTION(accumulated_distance > 0.0, "Accumulated distance must be > 0");
        const double calculated_acceleration = (ending_speed * ending_speed - starting_speed * starting_speed) /
                                                (2.0 * accumulated_distance);
        RUNTIME_EXCEPTION(acceleration_value == calculated_acceleration, "Acceleration is invalid");
      }
    }
  }

  return true;
}

RacePlan::RacePlan(std::vector<std::vector<std::pair<size_t, size_t>>> segments,
                   std::vector<std::vector<std::pair<double, double>>> segment_speeds,
                   std::vector<std::vector<bool>> acceleration_segments,
                   std::vector<std::vector<double>> acceleration)
                  : segments(segments), segment_speeds(segment_speeds), acceleration_segments(acceleration_segments),
                  acceleration(acceleration) {
  if (acceleration_segments.size() == 0 || acceleration.size() == 0) {
    this->acceleration_segments.resize(segments.size());
    this->acceleration.resize(segments.size());
    for (size_t i=0; i < segments.size(); i++) {
      this->acceleration_segments[i] = std::vector<bool>(segments[i].size(), false);
      this->acceleration[i] = std::vector<double>(segments.size(), 0.0);
    }
  }
}
