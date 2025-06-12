#include <stdlib.h>

#include <algorithm>
#include <iostream>
#include <sstream>
#include <cmath>
#include <cassert>
#include <fstream>
#include <limits>
#include <string>
#include <random>
#include <utility>
#include <vector>
#include <iomanip>
#include <stack>

#include "utils/Defines.hpp"
#include "route/Route.hpp"
#include "config/Config.hpp"
#include "utils/Utilities.hpp"
#include "utils/Logger.hpp"
#include "utils/Geography.hpp"
#include "utils/CustomException.hpp"

Route::Route(const std::filesystem::path route_path,
            bool telem_flow,
            const bool init_control_stops,
            const std::filesystem::path cornering_bounds_path,
            const std::filesystem::path precomputed_distances_path,
            const bool precompute_distances) {
  init_base_route(route_path, telem_flow);

  if (init_control_stops) {
    this->init_control_stops();
  }

  if (!cornering_bounds_path.empty()) {
    this->max_route_speed = kph2mps(Config::get_instance()->get_max_route_speed());
    this->init_cornering_bounds(cornering_bounds_path, kph2mps(Config::get_instance()->get_max_car_speed()));
  }

  if (!precomputed_distances_path.empty() && !precompute_distances) {
    route_distances = BasicLut(precomputed_distances_path);
    RUNTIME_EXCEPTION(route_distances.get_num_rows() == route_distances.get_num_cols(),
                      "Precomputed distance from {} must be a square matrix", precomputed_distances_path.string());
  }

  if (precompute_distances) {
    this->precompute_distances(precomputed_distances_path);
  }
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

void Route::precompute_distances(const std::filesystem::path csv_path) {
  RUNTIME_EXCEPTION(route_points.size() > 0 && num_points > 0, "Route csv not yet loaded");

  // Pre-allocate data
  std::vector<std::vector<double>> index_distances;
  index_distances.resize(num_points);
  for (size_t i=0; i < num_points; i++) {
    index_distances[i].resize(num_points, 0.0);
  }
  // We want to calculate the distance between each pair of coordinates along the points of
  // the route e.g. for index_distances[15][200], it's the accumulated distance from route_points[15]
  // to route_points[200]. For index_distances[270][8], it's the accumulated distance from
  // route_point[200] to route_point.end() + accumulated distance from route_points[0] to route_points[8]
  index_distances[num_points-1][0] = get_distance(route_points[num_points-1], route_points[0]);
  for (size_t src=0; src < num_points; src++) {
    for (size_t dest=0; dest < num_points; dest++) {
      if (src == dest) {
        continue;
      } else if (src < dest) {
        index_distances[src][dest] = index_distances[src][dest-1] +
                                     get_distance(route_points[dest-1], route_points[dest]);
      } else {
        index_distances[src][dest] = index_distances[0][num_points-1] - index_distances[0][src]
                                     + index_distances[0][dest] + index_distances[num_points-1][0];
      }
    }
  }
  route_distances = BasicLut(index_distances);

  if (!csv_path.empty()) {
    std::ofstream output_stream(csv_path);
    RUNTIME_EXCEPTION(output_stream.is_open(), "Output csv {} could not be opened for writing", csv_path.string());
    output_stream << std::fixed << std::setprecision(8);
    for (const auto& row : index_distances) {
      for (size_t i=0; i < num_points; i++) {
        output_stream << row[i];
        if (i < row.size() - 1) {
          output_stream << ",";
        }
      }
      output_stream << "\n";
    }

    output_stream.close();
  }
}

void Route::init_base_route(const std::filesystem::path route_path, bool telem_flow) {
  std::fstream base_route(route_path);
  RUNTIME_EXCEPTION(base_route.is_open(), "Base route file not found {}", route_path.string());

  route_length = 0.0;
  Coord last_coord;

  bool first_coord = true;
  // Read and parse the file
  while (!base_route.eof()) {
    std::string line;
    base_route >> line;
    std::cout << line << std::endl;
    std::stringstream linestream(line);

    std::string cell;
    Coord coord{};

    std::getline(linestream, cell, ',');
    RUNTIME_EXCEPTION(isDouble(cell), "Value {} in route file {} is not a number", cell, route_path.string());
    coord.lat = std::stod(cell);
    std::cout << cell << std::endl;

    std::getline(linestream, cell, ',');
    RUNTIME_EXCEPTION(isDouble(cell), "Value {} in route file {} is not a number", cell, route_path.string());
    coord.lon = std::stod(cell);
    std::cout << cell << std::endl;

    std::getline(linestream, cell, ',');
    RUNTIME_EXCEPTION(isDouble(cell), "Value {} in route file {} is not a number", cell, route_path.string());
    coord.alt = std::stod(cell);
    std::cout << cell << std::endl;

    if (telem_flow) {
      if (linestream.eof()) {
        std::cout << cell << std::endl;
        std::cout << "Yo what?" << std::endl;
      }
      if (linestream.peek() == '\n' || linestream.eof()) {
        RUNTIME_EXCEPTION(false, "No new values detected for telemetry flow. Does {} have |time|speed| columns?",
                          route_path.string());
      }
      std::getline(linestream, cell, ',');
      std::cout << cell << std::endl;
      Time time_point(cell, Config::get_instance()->get_utc_adjustment());
      timestamps.emplace_back(time_point);

      std::getline(linestream, cell, ',');
      RUNTIME_EXCEPTION(isDouble(cell), "Value {} in route file {} is not a number", cell, route_path.string());
      double speed = std::stod(cell);
      std::cout << cell << std::endl;
      speeds.push_back(speed);
    }
    route_points.emplace_back(coord);

    if (!first_coord) {
      route_length = route_length + get_distance(last_coord, coord);
    } else {
      first_coord = false;
    }

    last_coord = coord;
  }
  num_points = route_points.size();
  spdlog::info("Loaded base route {} with {} coordinates", route_path.string(), std::to_string(num_points));
}

void Route::init_control_stops() {
  control_stops = Config::get_instance()->get_control_stops();
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
  size_t idx = 0;
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
    double max_corner_speed = std::stod(max_speed);
    cornering_speed_bounds.push_back(max_corner_speed);
    cornering_segment_bounds.push_back(bound);
    corner_end_to_corner_idx.insert({bound.second, idx});
    corner_start_to_corner_idx.insert({bound.first, idx});
    idx = idx + 1;
    last_bound = bound;
    is_first_segment = false;
  }
}


size_t Route::get_closest_corner_idx(size_t route_index) const {
  RUNTIME_EXCEPTION(route_index >= 0 && route_index < num_points, "Route index is out of bounds");
  RUNTIME_EXCEPTION(cornering_segment_bounds.size() > 0, "Cornering bounds is empty");
  if (cornering_segment_bounds.size() == 1) {
    return 0;
  }

  // Run a binary search
  int low = 0;
  int high = cornering_segment_bounds.size() - 1;
  while (low <= high) {
    // Implicitly "rounds" down
    int mid = low + (high - low) / 2;

    if (mid == cornering_segment_bounds.size() - 1) {
      return cornering_segment_bounds.size() - 1;
    } else if (mid == 0) {
      return 0;
    } else if (cornering_segment_bounds[mid].first <= route_index &&
               cornering_segment_bounds[mid+1].first >= route_index) {
      return mid;
    }

    if (cornering_segment_bounds[mid].first < route_index) {
      low = mid + 1;
    } else {
      high = mid - 1;
    }
  }

  RUNTIME_EXCEPTION(false, "The binary search failed on a continuous range. Check implementation. "
                           "This should be impossible");
}

double Route::get_segment_length(const size_t start_idx, const size_t end_idx) const {
  RUNTIME_EXCEPTION(route_points.size() > 0, "Route points not loaded");
  double accumulated_distance = 0.0;
  size_t num_segment_points;
  if (end_idx < start_idx) {
    num_segment_points = route_points.size() - start_idx + end_idx + 1;
  } else {
    num_segment_points = end_idx - start_idx;
  }
  for (size_t i=0; i < num_segment_points; i++) {
    const size_t coord_one_idx = start_idx + i < route_points.size() ? start_idx + i :
                                i - (route_points.size() - start_idx);
    const size_t coord_two_idx = start_idx + i + 1 < route_points.size() ? start_idx + i + 1 :
                                i - (route_points.size() - start_idx) + 1;
    const Coord& coord_one = route_points[coord_one_idx];
    const Coord& coord_two = route_points[coord_two_idx];
    accumulated_distance += get_distance(coord_one, coord_two);
  }

  return accumulated_distance;
}

void Route::init_longest_straight() {
  const size_t num_corners = cornering_segment_bounds.size();
  longest_straight = 0.0;
  for (size_t i=0; i < num_corners; i++) {
    double straight_distance;
    const std::pair<size_t, size_t> current_corner = cornering_segment_bounds[i];
    std::pair<size_t, size_t> next_corner;
    if (i < num_corners - 1) {
      next_corner = cornering_segment_bounds[i+1];
    } else {
      next_corner = cornering_segment_bounds[0];
    }
    if (route_distances.is_empty()) {
      straight_distance = get_segment_length(current_corner.second, next_corner.first);
    } else {
      straight_distance = route_distances.get_value(current_corner.second, next_corner.first);
    }

    if (straight_distance > longest_straight) {
      longest_straight = straight_distance;
    }
  }
}

/** @brief Calculate the segment distance from starting_idx to ending_idx inclusive
 * @param coords Vector of route coordinates
 * @param starting_idx Segment starting point. Must be in the range of [0, coords.size())
 * @param ending_idx Segment ending point. Must be in the range of [0, coords.size())
 */
double calc_segment_distance(const std::vector<Coord>& coords, const size_t starting_idx,
                              const size_t ending_idx) {
  if (starting_idx == ending_idx) {
    return 0.0;
  }

  RUNTIME_EXCEPTION(starting_idx >= 0 && starting_idx < coords.size() && ending_idx >= 0 && ending_idx < coords.size(),
                    "Starting idx {} and ending idx {} are not in the correct range", starting_idx, ending_idx);

  double accumulated_distance = 0.0;
  size_t num_segment_points;
  if (ending_idx < starting_idx) {
    num_segment_points = coords.size() - starting_idx + ending_idx;
  } else {
    num_segment_points = ending_idx - starting_idx;
  }
  for (size_t i=0; i < num_segment_points; i++) {
    const size_t coord_one_idx = starting_idx + i < coords.size() ? starting_idx + i :
                                  i - (coords.size() - starting_idx);
    const size_t coord_two_idx = starting_idx + i + 1 < coords.size() ? starting_idx + i + 1 :
                                  i - (coords.size() - starting_idx) + 1;
    const Coord& coord_one = coords[coord_one_idx];
    const Coord& coord_two = coords[coord_two_idx];
    accumulated_distance += get_distance(coord_one, coord_two);
  }

  return accumulated_distance;
}

std::string RacePlan::get_loop_string(std::vector<std::pair<size_t, size_t>> loop_segments,
                                      std::vector<std::pair<double, double>> loop_segment_speeds,
                                      std::vector<double> loop_acceleration_values,
                                      std::vector<double> loop_segment_distances) {
  const size_t num_loops = loop_segments.size();
  std::ostringstream output;

  auto print_spaces = [&](size_t num_spaces) {
    for (size_t i = 0; i < num_spaces; i++) {
      output << " ";
    }
  };

  const bool print_distances = loop_segment_distances.size() > 0;
  auto print_header = [&]() {
    if (print_distances) {
      output << "+-----------+---------------+-----------------------+----------------\n";
      output << "; Segments  ; Speeds (m/s)  ; Acceleration (m/s^2)  ; Distances (m) ;\n";
      output << "+-----------+---------------+-----------------------+----------------\n";
    } else {
      output << "+-----------+---------------+-----------------------+\n";
      output << "; Segments  ; Speeds (m/s)  ; Acceleration (m/s^2)  ;\n";
      output << "+-----------+---------------+-----------------------+\n";
    }
  };

  print_header();

  const size_t num_segments = loop_segments.size();
  for (size_t seg_idx = 0; seg_idx < num_segments; seg_idx++) {
    output << "| [" << std::setw(3) << loop_segments[seg_idx].first << ","
            << std::setw(3) << loop_segments[seg_idx].second << "] |";
    print_spaces(5);
    output << "[" << std::setw(3) << loop_segment_speeds[seg_idx].first << ","
                  << std::setw(3) << loop_segment_speeds[seg_idx].second << "] |";
    print_spaces(18);
    // Truncate the acceleration
    std::ostringstream oss;
    oss << loop_acceleration_values[seg_idx];
    std::string num_str = oss.str();
    if (num_str.size() > 5) {
      num_str = num_str.substr(0, 5);
    }
    output << std::setw(5) << num_str;
    output << "|";

    if (print_distances) {
      // Truncate the distance
      print_spaces(10);
      std::ostringstream dss;
      dss << loop_segment_distances[seg_idx];
      std::string dist_str = dss.str();
      if (dist_str.size() > 5) {
        dist_str = dist_str.substr(0, 5);
      }
      output << std::setw(5) << dist_str;
      output << "|";
    }
    output << "\n";
  }
  if (print_distances) {
    output << "---------------------------------------------------------------------\n";
  } else {
    output << "-----------------------------------------------------\n";
  }

  return output.str();
}

void RacePlan::print_plan() const {
  const size_t num_loops = segments.size();

  for (size_t loop_idx = 0; loop_idx < num_loops; loop_idx++) {
    std::cout << "\nLOOP " << loop_idx << std::endl;
    std::vector<double> loop_distance = distances.size() > 0 ? distances[loop_idx] : std::vector<double>(0);
    std::cout << get_loop_string(segments[loop_idx], segment_speeds[loop_idx], acceleration[loop_idx],
                                 loop_distance) << std::endl;
  }
}

bool RacePlan::validate_members(const std::vector<Coord>& route_points) const {
  RUNTIME_EXCEPTION(!empty, "RacePlan is empty");
  RUNTIME_EXCEPTION(segments.size() > 0, "Validate_members() for RacePlan called before segments were set.");
  RUNTIME_EXCEPTION(segments.size() == segment_speeds.size() && segments.size() == acceleration_segments.size() &&
                    acceleration_segments.size() == acceleration.size(),
                    "RacePlan not properly created. Speed profile, route segments and acceleration segments"
                    "have unequal number of loops");
  if (distances.size() > 0) {
    RUNTIME_EXCEPTION(distances.size() == segments.size(), "Loop segment distances have different "
                      "number of loops compared to segments");
  }
  const size_t num_loops = segments.size();
  const double tolerance = 0.0001;  // Tolerance for comparing acceleration values
  RUNTIME_EXCEPTION(orig_loop_segments.size() == orig_loop_speeds.size() &&
                    orig_loop_segments.size() == orig_loop_accelerations.size() &&
                    orig_loop_segments.size() == orig_loop_acceleration_values.size() &&
                    orig_loop_segments.size() == orig_loop_segment_distances.size(),
                    "Raw segment speeds, acceleration values, distances must have the same number "
                    "of loops");
  std::pair<double, double> last_segment_speeds;
  for (size_t loop_idx=0; loop_idx < num_loops; loop_idx++) {
    const std::vector<std::pair<size_t, size_t>>& loop_segments = segments[loop_idx];
    const std::vector<std::pair<double, double>>& loop_segments_speeds = segment_speeds[loop_idx];
    const std::vector<bool>& loop_segments_acceleration = acceleration_segments[loop_idx];
    const std::vector<double>& loop_segments_acceleration_values = acceleration[loop_idx];

    std::vector<double> loop_segment_distances;
    if (distances.size() > 0) {
      loop_segment_distances = distances[loop_idx];
    }

    RUNTIME_EXCEPTION(loop_segments.size() == loop_segments_speeds.size() &&
                      loop_segments.size() == loop_segments_acceleration.size() &&
                      loop_segments.size() == loop_segments_acceleration_values.size(),
                      "Segment speeds, acceleration values and distances must be the same length for each loop");
    if (distances.size() > 0) {
      RUNTIME_EXCEPTION(loop_segment_distances.size() == loop_segments.size(), "Segment distances and segments "
                        "must have the same length for each loop");
    }

    const size_t num_segments = loop_segments.size();
    // Don't have this check in case we are starting at a different start point. More relevant to WSC than
    // FSGP
    // RUNTIME_EXCEPTION(loop_segments[0].first == 0, "Segments must start at index 0");

    if (num_loops > 1) {
      RUNTIME_EXCEPTION(loop_segments[num_segments-1].second == 0,
        "Last segment's ending point must be 0 i.e. wrap-around. Loop {}", loop_idx);
    } else {
      RUNTIME_EXCEPTION(loop_segments[num_segments-1].second == route_points.size() - 1, "Non-loop race plan "
      "must have ending point equal to the last point of the route");
    }
    if (loop_idx > 0) {
      RUNTIME_EXCEPTION(loop_segments_speeds[0].first == last_segment_speeds.second,
                        "Last loop's ending speed must be the starting speed of the first segment in the next loop. "
                        "Error found in loop {}", loop_idx);
    }
    for (size_t i=0; i < num_segments; i++) {
      const double segment_distance = calc_segment_distance(route_points, loop_segments[i].first,
                                                            loop_segments[i].second);
      if (loop_segment_distances.size() > 0) {
        RUNTIME_EXCEPTION(std::abs(segment_distance - loop_segment_distances[i]) < tolerance,
                          "Segment distance is not equal");
      }
      RUNTIME_EXCEPTION(loop_segments_speeds[i].first >= 0.0 && loop_segments_speeds[i].second >= 0.0,
                        "Segment speed in segment {} is not positive", i);

      if (i < num_segments-1) {
        RUNTIME_EXCEPTION(loop_segments[i].first <= loop_segments[i].second, "Segment start must be <= segment end in "
                          "loop {} segment {}", loop_idx, i);
        RUNTIME_EXCEPTION(loop_segments[i].second == loop_segments[i+1].first,
                          "Segments must be continuous i.e. segment end = next segment start."
                          " Invalid on segment {}, loop {}", i, loop_idx);
        RUNTIME_EXCEPTION(loop_segments_speeds[i].second == loop_segments_speeds[i+1].first,
                          "Ending speed of segment {} must equal starting speed of next segment in loop {}. {} {}", i,
                          loop_idx, loop_segments_speeds[i].second, loop_segments_speeds[i+1].first);
      }

      if (!loop_segments_acceleration[i]) {
        RUNTIME_EXCEPTION(loop_segments_speeds[i].first == loop_segments_speeds[i].second,
                          "Speed profile for non-acceleration segment {} must have equal and positive starting "
                          "and ending speeds", i);
      } else {
        // Verify acceleration value
        const double acceleration_value = loop_segments_acceleration_values[i];
        const double starting_speed = loop_segments_speeds[i].first;
        const double ending_speed = loop_segments_speeds[i].second;
        const double calculated_acceleration_distance = calc_distance_a(starting_speed, ending_speed,
                                                                        acceleration_value);
        RUNTIME_EXCEPTION(calculated_acceleration_distance < segment_distance ||
                          std::abs(calculated_acceleration_distance - segment_distance) < tolerance,
                          "Acceleration distance {} is invalid. Must be less than the segment distance {} "
                          "for loop {} segment {}", calculated_acceleration_distance, segment_distance,
                          loop_idx, i);
      }
    }

    last_segment_speeds = loop_segments_speeds[num_segments-1];
  }

  return true;
}

RacePlan::RacePlan(std::vector<std::vector<std::pair<size_t, size_t>>> segments,
                   std::vector<std::vector<std::pair<double, double>>> segment_speeds,
                   std::vector<std::vector<bool>> acceleration_segments,
                   std::vector<std::vector<double>> acceleration,
                   std::vector<std::vector<double>> distances,
                   int num_repetitions,
                   std::vector<std::vector<std::pair<size_t, size_t>>> orig_loop_segments,
                   std::vector<std::vector<std::pair<double, double>>> orig_loop_speeds,
                   std::vector<std::vector<bool>> orig_loop_accelerations,
                   std::vector<std::vector<double>> orig_loop_acceleration_values,
                   std::vector<std::vector<double>> orig_loop_segment_distances)
                  : segments(segments), segment_speeds(segment_speeds), acceleration_segments(acceleration_segments),
                  acceleration(acceleration), distances(distances), num_repetitions(num_repetitions),
                  orig_loop_segments(orig_loop_segments), orig_loop_speeds(orig_loop_speeds),
                  orig_loop_acceleration_values(orig_loop_acceleration_values),
                  orig_loop_accelerations(orig_loop_accelerations),
                  orig_loop_segment_distances(orig_loop_segment_distances) {
  this->num_loops = segments.size();
  RUNTIME_EXCEPTION(num_repetitions >= 1, "Number of repetitions per loop block must be at least 1");
  this->num_blocks = static_cast<int>(std::ceil(this->num_loops / this->num_repetitions));

  if (this->segments.size() == 0 && this->segment_speeds.size() == 0 &&
      this->acceleration_segments.size() == 0 && this->acceleration.size() == 0 &&
      this->distances.size() == 0) {
    empty = true;
  } else {
    empty = false;
  }

  int num_blocks{0};
  for (num_blocks=1; num_blocks<segments.size(); num_blocks++){
    if (segments[num_blocks]!=segments[num_blocks-1]) break;
  }
}
