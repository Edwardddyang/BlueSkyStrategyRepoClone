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

double calculate_segment_distance(const std::vector<Coord>& coords,
                                  const size_t starting_idx,
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

Route::Route(const std::filesystem::path route_path,
            bool telem_flow, const bool init_control_stops,
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
    // std::cout << line << std::endl;
    if (line.empty()) {
      break;
    }
    std::stringstream linestream(line);

    std::string cell;
    Coord coord{};

    std::getline(linestream, cell, ',');
    RUNTIME_EXCEPTION(isDouble(cell), "Value {} in route file {} is not a number", cell, route_path.string());
    coord.lat = std::stod(cell);
    // std::cout << cell << std::endl;

    std::getline(linestream, cell, ',');
    RUNTIME_EXCEPTION(isDouble(cell), "Value {} in route file {} is not a number", cell, route_path.string());
    coord.lon = std::stod(cell);
    // std::cout << cell << std::endl;

    std::getline(linestream, cell, ',');
    RUNTIME_EXCEPTION(isDouble(cell), "Value {} in route file {} is not a number", cell, route_path.string());
    coord.alt = std::stod(cell);
    // std::cout << cell << std::endl;

    if (telem_flow) {
      // if (linestream.eof()) {
      //   std::cout << cell << std::endl;
      //   std::cout << "Yo what?" << std::endl;
      // }
      if (linestream.peek() == '\n' || linestream.eof()) {
        RUNTIME_EXCEPTION(false, "No new values detected for telemetry flow. Does {} have |time|speed| columns?",
                          route_path.string());
      }
      std::getline(linestream, cell, ',');
      // std::cout << cell << std::endl;
      Time time_point(cell, Config::get_instance()->get_utc_adjustment());
      timestamps.emplace_back(time_point);

      std::getline(linestream, cell, ',');
      RUNTIME_EXCEPTION(isDouble(cell), "Value {} in route file {} is not a number", cell, route_path.string());
      double speed = std::stod(cell);
      // std::cout << cell << std::endl;
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
    corner_start_indices.insert(bound.first);
    corner_end_indices.insert(bound.second);
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

double Route::calc_segment_distance(const size_t starting_idx,
                                    const size_t ending_idx) {
  return calculate_segment_distance(route_points, starting_idx, ending_idx);
}

std::string RacePlan::get_loop_string(LoopData loop_segments) {
  const size_t num_loops = loop_segments.size();
  std::ostringstream output;

  auto print_char_n_times = [&](char ch, int n) {
    for (int i = 0; i < n; ++i) {
      output << ch;
    }
  };

  // Lambda function to truncate number to fit a string of n chars.
  // If number has no. chars < n, it does nothing
  auto truncate_number = [](double number, int n) -> std::string {
    std::ostringstream oss;

    // Determine the precision based on initial length estimates
    if (number >= 10) {
      oss << std::fixed << std::setprecision(1) << number;
    } else {
      oss << std::fixed << std::setprecision(2) << number;
    }

    std::string result = oss.str();

    // Only truncate if the result is longer than n characters
    if (result.length() > n) {
      result = result.substr(0, n);

      if (result.back() == '.') {
        result.pop_back();
      }
    }

    return result;
  };

  auto print_header = [&]() {
    output << "+-------+-----------+---------------+-----------------------+---------------+---------\n";
    output << ";  Idx  ; Segments  ; Speeds (m/s)  ; Acceleration (m/s^2)  ; Distances (m) ; Corner ;\n";
    output << "+-------+-----------+---------------+-----------------------+---------------+---------\n";
  };

  print_header();

  const size_t num_segments = loop_segments.size();
  for (size_t seg_idx = 0; seg_idx < num_segments; seg_idx++) {
    // Index (seg_idx)
    output << "|";
    print_char_n_times(' ', 1);
    output << std::setw(3) << seg_idx;
    print_char_n_times(' ', 3);

    // Segment indices
    output << "| [" << std::setw(3) << loop_segments[seg_idx].start_idx << ","
            << std::setw(3) << loop_segments[seg_idx].end_idx << "] |";

    // Segment speeds
    print_char_n_times(' ', 1);
    output << "[" << std::setw(5) << truncate_number(loop_segments[seg_idx].start_speed, 5) << ","
                  << std::setw(5) << truncate_number(loop_segments[seg_idx].end_speed, 5) << "] |";

    // Segment acceleration
    print_char_n_times(' ', 18);
    output << std::setw(5) << truncate_number(loop_segments[seg_idx].acceleration_value, 5);
    output << "|";

    // Truncate the distance
    print_char_n_times(' ', 10);
    output << std::setw(5) << truncate_number(loop_segments[seg_idx].distance, 5);
    output << "|";

    // Has Corner
    print_char_n_times(' ', 2);
    output << (loop_segments[seg_idx].includes_corner ? "True  |\n" : "False |\n");
  }
  print_char_n_times('-', 86);
  output << "\n";

  return output.str();
}

std::string RacePlan::get_segment_string(SegmentData seg) {
  std::stringstream ss;
  ss << "Segment: [" << seg.start_idx << "," << seg.end_idx << "]\n";
  ss << "Segment Distance: " << seg.distance << "m\n";
  ss << "Segment Speeds: [" << seg.start_speed << "," << seg.end_speed << "]\n";
  ss << "Acceleration Value: " << seg.acceleration_value << "\n";
  return ss.str();
}

void RacePlan::print_plan() const {
  std::cout << get_plan_string() << std::endl;
}

std::string RacePlan::get_plan_string() const {
  const size_t num_loops = segments.size();
  std::string ret = "";

  for (size_t loop_idx = 0; loop_idx < num_loops; loop_idx++) {
    ret += "\nLOOP " + std::to_string(loop_idx) + "\n";
    ret += get_loop_string(segments[loop_idx]);
  }
  return ret;
}

std::string RacePlan::get_orig_plan_string() const {
  const size_t num_blocks = orig_segments.size();
  std::string ret = "";

  for (size_t block_idx = 0; block_idx < num_blocks; block_idx++) {
    ret += "\nBlock " + std::to_string(block_idx) + "\n";
    ret += get_loop_string(orig_segments[block_idx]);
  }
  return ret;
}

bool RacePlan::validate_members(const std::vector<Coord>& route_points) const {
  RUNTIME_EXCEPTION(!empty, "RacePlan is empty");
  RUNTIME_EXCEPTION(segments.size() > 0, "No segments in Race Plan!");

  const size_t num_loops = segments.size();
  const double tolerance = 0.0001;  // Tolerance for comparing acceleration values

  double last_segment_start_speed, last_segment_end_speed;
  for (size_t loop_idx=0; loop_idx < num_loops; loop_idx++) {
    const LoopData loop_segments = segments[loop_idx];

    const size_t num_segments = loop_segments.size();
    // Don't have this check in case we are starting at a different start point. More relevant to WSC than
    // FSGP
    // RUNTIME_EXCEPTION(loop_segments[0].first == 0, "Segments must start at index 0");

    if (num_loops > 1) {
      RUNTIME_EXCEPTION(loop_segments[num_segments-1].end_idx == 0,
        "Last segment's ending point must be 0 i.e. wrap-around. Loop {}", loop_idx);
    } else {
      RUNTIME_EXCEPTION(loop_segments[num_segments-1].end_idx == route_points.size() - 1, "Non-loop race plan "
      "must have ending point equal to the last point of the route");
    }
    if (loop_idx > 0) {
      RUNTIME_EXCEPTION(loop_segments[0].start_speed == last_segment_end_speed,
                        "Last loop's ending speed must be the starting speed of the first segment in the next loop. "
                        "Error found in loop {}", loop_idx);
    }
    for (size_t i=0; i < num_segments; i++) {
      const double segment_distance = calculate_segment_distance(route_points, loop_segments[i].start_idx,
                                                                loop_segments[i].end_idx);
      RUNTIME_EXCEPTION(std::abs(segment_distance - loop_segments[i].distance) < tolerance,
                        "Segment distance is not equal");
      RUNTIME_EXCEPTION(loop_segments[i].start_speed >= 0.0 && loop_segments[i].end_speed >= 0.0,
                        "Segment speed in segment {} is not positive", i);

      if (i < num_segments-1) {
        RUNTIME_EXCEPTION(loop_segments[i].start_idx <= loop_segments[i].end_idx,
                          "Segment start must be <= segment end in loop {} segment {}", loop_idx, i);
        RUNTIME_EXCEPTION(loop_segments[i].end_idx == loop_segments[i+1].start_idx,
                          "Segments must be continuous i.e. segment end = next segment start."
                          " Invalid on segment {}, loop {}. Ending index = {}, Next Starting index = {}",
                          i, loop_idx, loop_segments[i].end_idx, loop_segments[i+1].start_idx);
        RUNTIME_EXCEPTION(loop_segments[i].end_speed == loop_segments[i+1].start_speed,
                          "Ending speed of segment {} must equal starting speed of next segment in loop {}. {} {}", i,
                          loop_idx, loop_segments[i].end_speed, loop_segments[i+1].start_speed);
      }

      if (loop_segments[i].acceleration_value == 0.0) {
        RUNTIME_EXCEPTION(loop_segments[i].start_speed == loop_segments[i].end_speed,
                          "Speed profile for non-acceleration segment {} must have equal and positive starting "
                          "and ending speeds", i);
      } else {
        // Verify acceleration value
        const double acceleration_value = loop_segments[i].acceleration_value;
        const double starting_speed = loop_segments[i].start_speed;
        const double ending_speed = loop_segments[i].end_speed;
        const double calculated_acceleration_distance = calc_distance_a(starting_speed, ending_speed,
                                                                        acceleration_value);
        RUNTIME_EXCEPTION(calculated_acceleration_distance < segment_distance ||
                          std::abs(calculated_acceleration_distance - segment_distance) < tolerance,
                          "Acceleration distance {} is invalid. Must be less than the segment distance {} "
                          "for loop {} segment {}", calculated_acceleration_distance, segment_distance,
                          loop_idx, i);
      }
    }

    last_segment_start_speed = loop_segments[num_segments-1].start_speed;
    last_segment_end_speed = loop_segments[num_segments-1].end_speed;
  }

  return true;
}

RacePlan::RacePlan(PlanData segments, PlanData orig_segments, int num_repetitions) :
                   segments(segments), orig_segments(orig_segments),
                   num_repetitions(num_repetitions) {
  this->num_loops = segments.size();
  RUNTIME_EXCEPTION(num_repetitions >= 1, "Number of repetitions per loop block must be at least 1");
  this->num_blocks = static_cast<int>(std::ceil(this->num_loops / this->num_repetitions));

  if (segments.size() == 0) {
    empty = true;
  } else {
    empty = false;
  }
}
