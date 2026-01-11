#include "route/Route.hpp"

#include <spdlog/spdlog.h>  // IWYU pragma: keep

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "SimUtils/Defines.hpp"
#include "SimUtils/Geography.hpp"
#include "SimUtils/Types.hpp"
#include "SimUtils/Utilities.hpp"

double calculate_segment_distance(const CoordVec& coords, size_t starting_idx,
                                  size_t ending_idx) {
  if (starting_idx == ending_idx) {
    return 0.0;
  }

  RUNTIME_EXCEPTION(
      starting_idx >= 0 && starting_idx < coords.size() && ending_idx >= 0 &&
          ending_idx < coords.size(),
      "Starting idx {} and ending idx {} are not in the correct range",
      starting_idx, ending_idx);

  double accumulated_distance = 0.0;
  size_t num_segment_points = 0;
  if (ending_idx < starting_idx) {
    num_segment_points = coords.size() - starting_idx + ending_idx;
  } else {
    num_segment_points = ending_idx - starting_idx;
  }
  for (size_t i = 0; i < num_segment_points; i++) {
    const size_t coord_one_idx = starting_idx + i < coords.size()
                                     ? starting_idx + i
                                     : i - (coords.size() - starting_idx);
    const size_t coord_two_idx = starting_idx + i + 1 < coords.size()
                                     ? starting_idx + i + 1
                                     : i - (coords.size() - starting_idx) + 1;
    const util::type::Coord& coord_one = coords[coord_one_idx];
    const util::type::Coord& coord_two = coords[coord_two_idx];
    accumulated_distance += util::geo::get_distance(coord_one, coord_two);
  }

  return accumulated_distance;
}

std::string truncate_number(double number, int n) {
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
}

void append_char_n_times(char ch, uint64_t n, std::stringstream& output) {
  for (uint64_t i = 0; i < n; i++) {
    output << ch;
  }
}

WSCRoute::WSCRoute(WSCRouteParams params,
                   const std::filesystem::path& route_path)
    : params(std::move(params)) {
  // Read route csv file
  std::fstream base_route(route_path);
  RUNTIME_EXCEPTION(base_route.is_open(), "Base route file not found {}",
                    route_path.string());

  this->route_length = 0.0;
  util::type::Coord last_coord;

  bool first_coord = true;
  std::string line;

  while (std::getline(base_route, line)) {
    std::stringstream linestream(line);
    std::string latitude, longitude, alt;
    std::optional<double> parsed_value;

    util::type::Coord coord{};
    std::getline(linestream, latitude, ',');
    parsed_value = util::detail::convert_num<double>(latitude);
    RUNTIME_EXCEPTION(parsed_value.has_value(),
                      "{} could not be converted to a latitude number",
                      latitude);
    coord.lat = *parsed_value;

    std::getline(linestream, longitude, ',');
    parsed_value = util::detail::convert_num<double>(longitude);
    RUNTIME_EXCEPTION(parsed_value.has_value(),
                      "{} could not be converted to a longitude number",
                      longitude);
    coord.lon = *parsed_value;

    std::getline(linestream, alt, ',');
    parsed_value = util::detail::convert_num<double>(alt);
    RUNTIME_EXCEPTION(parsed_value.has_value(),
                      "{} could not be converted to a latitude number", alt);
    coord.alt = *parsed_value;

    route_points.emplace_back(coord);

    if (!first_coord) {
      this->route_length += util::geo::get_distance(coord, last_coord);
    } else {
      first_coord = false;
    }

    last_coord = coord;
  }

  this->num_points = route_points.size();
  base_route.close();

  spdlog::info("Loaded base route {} with {} coordinates and length {} m",
               route_path.string(), std::to_string(num_points),
               std::to_string(this->route_length));
}

/** @brief Segment a route (array of coordinates) into uniform segments
 * @return A vector of segments denoted by the begin and end indices
 */
std::vector<std::pair<size_t, size_t>> WSCRoute::segment_route_uniform(
    double length) const {
  RUNTIME_EXCEPTION(!route_points.empty(), "Route not yet loaded");

  std::vector<std::pair<size_t, size_t>> segments;

  // Create segments
  double current_segment_distance = 0.0;
  double difference = 0.0;  // Difference between target segment length and
                            // current_segment_distance
  double last_difference = std::numeric_limits<double>::max();
  std::pair<size_t, size_t> segment_indices = {0, 0};

  for (size_t idx = 0; idx < num_points - 1; idx++) {
    const util::type::Coord& coord_one = route_points[idx];
    const util::type::Coord& coord_two = route_points[idx + 1];

    segment_indices.second = idx + 1;
    const double distance = util::geo::get_distance(coord_one, coord_two);
    current_segment_distance += distance;

    difference = std::abs(length - current_segment_distance);

    if (difference > last_difference) {  // Is this condition ever met?
      segment_indices.second = idx;
      segments.push_back(segment_indices);

      segment_indices = {idx, idx + 1};
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

FSGPRoute::FSGPRoute(FSGPRouteParameters params,
                     const std::filesystem::path& route_path,
                     const std::filesystem::path& precomputed_distances_path,
                     const std::filesystem::path& corner_bounds_path)
    : params(params) {
  // Read route csv file
  std::fstream base_route(route_path);
  RUNTIME_EXCEPTION(base_route.is_open(), "Base route file not found {}",
                    route_path.string());

  this->route_length = 0.0;
  util::type::Coord last_coord;

  bool first_coord = true;
  std::string line;

  while (std::getline(base_route, line)) {
    std::stringstream linestream(line);
    std::string latitude, longitude, alt;
    std::optional<double> parsed_value;

    util::type::Coord coord{};
    std::getline(linestream, latitude, ',');
    parsed_value = util::detail::convert_num<double>(latitude);
    RUNTIME_EXCEPTION(parsed_value.has_value(),
                      "{} could not be converted to a latitude number",
                      latitude);
    coord.lat = *parsed_value;

    std::getline(linestream, longitude, ',');
    parsed_value = util::detail::convert_num<double>(longitude);
    RUNTIME_EXCEPTION(parsed_value.has_value(),
                      "{} could not be converted to a longitude number",
                      longitude);
    coord.lon = *parsed_value;

    std::getline(linestream, alt, ',');
    parsed_value = util::detail::convert_num<double>(alt);
    RUNTIME_EXCEPTION(parsed_value.has_value(),
                      "{} could not be converted to an altitud enumber", alt);
    coord.alt = *parsed_value;

    route_points.emplace_back(coord);

    if (!first_coord) {
      this->route_length += util::geo::get_distance(last_coord, coord);
    } else {
      first_coord = false;
    }

    last_coord = coord;
  }

  this->num_track_points = route_points.size();
  base_route.close();

  spdlog::info("Loaded track route {} with {} coordinates", route_path.string(),
               std::to_string(num_track_points));

  // Load corner bounds path
  if (!corner_bounds_path.empty()) {
    this->init_cornering_bounds(corner_bounds_path, params.max_route_speed);
  }

  // Load precomputed distances
  if (std::filesystem::exists(precomputed_distances_path)) {
    this->route_distances = PointsLut(precomputed_distances_path);
    RUNTIME_EXCEPTION(
        this->route_distances.get_num_rows() ==
            this->route_distances.get_num_cols(),
        "Precomputed distance matrix from {} must be a square matrix",
        precomputed_distances_path.string());
  } else {
    this->calculate_distances(precomputed_distances_path);
  }
}

void FSGPRoute::calculate_distances(const std::filesystem::path& csv_path) {
  RUNTIME_EXCEPTION(!route_points.empty() && num_track_points > 0,
                    "Track csv not yet loaded");

  // Pre-allocate data
  std::vector<std::vector<double>> index_distances;
  index_distances.resize(num_track_points);
  for (size_t i = 0; i < num_track_points; i++) {
    index_distances[i].resize(num_track_points, 0.0);
  }
  // We want to calculate the distance between each pair of coordinates along
  // the points of the route e.g. for index_distances[15][200], it's the
  // accumulated distance from route_points[15] to route_points[200]. For
  // index_distances[270][8], it's the accumulated distance from
  // route_point[270] to route_point.end() + distance from route_points[0] to
  // route_points[8]
  index_distances[num_track_points - 1][0] = util::geo::get_distance(
      route_points[num_track_points - 1], route_points[0]);
  for (size_t src = 0; src < num_track_points; src++) {
    for (size_t dest = 0; dest < num_track_points; dest++) {
      if (src == dest) {
        continue;
      } else if (src < dest) {
        index_distances[src][dest] =
            index_distances[src][dest - 1] +
            util::geo::get_distance(route_points[dest - 1], route_points[dest]);
      } else {
        index_distances[src][dest] =
            index_distances[0][num_track_points - 1] - index_distances[0][src] +
            index_distances[0][dest] + index_distances[num_track_points - 1][0];
      }
    }
  }

  // Dump calculated distances to a csv
  if (!csv_path.empty()) {
    std::ofstream output_stream(csv_path);
    RUNTIME_EXCEPTION(output_stream.is_open(),
                      "Output csv {} could not be opened for writing",
                      csv_path.string());
    output_stream << std::fixed << std::setprecision(8);
    for (const auto& row : index_distances) {
      for (size_t i = 0; i < num_track_points; i++) {
        output_stream << row[i];
        if (i < row.size() - 1) {
          output_stream << ",";
        }
      }
      output_stream << "\n";
    }

    output_stream.close();
  }

  this->route_distances = PointsLut(std::move(index_distances));
}

void FSGPRoute::init_cornering_bounds(
    const std::filesystem::path& corner_bounds_path, double max_car_speed) {
  this->corner_segment_bounds.resize(0);
  std::fstream csv(corner_bounds_path);
  RUNTIME_EXCEPTION(csv.is_open(), "Base route file not found {}",
                    corner_bounds_path.string());

  std::string start_bound;
  std::string end_bound;
  std::string max_speed;
  std::pair<size_t, size_t> last_bound;
  size_t idx = 0;
  // Read and parse the file
  bool is_first_segment = true;
  while (!csv.eof()) {
    std::string line;
    csv >> line;
    std::stringstream linestream(line);

    std::getline(linestream, start_bound, ',');
    std::getline(linestream, end_bound, ',');
    std::getline(linestream, max_speed, ',');
    std::pair<size_t, size_t> bound;

    std::optional<size_t> parsed_value;
    std::optional<double> parsed_double_value;

    parsed_value = util::detail::convert_num<size_t>(start_bound);
    RUNTIME_EXCEPTION(parsed_value.has_value(),
                      "Value {} in corner bounds file is not a whole number",
                      start_bound, corner_bounds_path.string());
    bound.first = *parsed_value;

    parsed_value = util::detail::convert_num<size_t>(end_bound);
    RUNTIME_EXCEPTION(parsed_value.has_value(),
                      "Value {} in corner bounds file is not a whole number",
                      end_bound, corner_bounds_path.string());
    bound.second = *parsed_value;

    RUNTIME_EXCEPTION(
        (is_first_segment && bound.first < bound.second) ||
            (!is_first_segment && bound.first > last_bound.second),
        "Cornering bounds are degenerate");

    parsed_double_value = util::detail::convert_num<double>(max_speed);
    RUNTIME_EXCEPTION(parsed_double_value.has_value(),
                      "Value {} in corner bounds file is not a number",
                      max_speed, corner_bounds_path.string());
    const double max_speed = std::max(*parsed_double_value, max_car_speed);
    corner_speed_bounds.push_back(max_speed);
    corner_segment_bounds.push_back(bound);
    corner_start_indices.insert(bound.first);
    corner_end_indices.insert(bound.second);
    corner_end_to_corner_idx.insert({bound.second, idx});
    corner_start_to_corner_idx.insert({bound.first, idx});
    idx = idx + 1;
    last_bound = bound;
    is_first_segment = false;
  }
}

size_t FSGPRoute::get_closest_corner_idx(size_t route_index) const {
  const size_t num_points = route_points.size();
  RUNTIME_EXCEPTION(route_index >= 0 && route_index < num_points,
                    "Route index is out of bounds");
  RUNTIME_EXCEPTION(!corner_segment_bounds.empty(),
                    "Cornering bounds is empty");
  if (corner_segment_bounds.size() == 1) {
    return 0;
  }

  // Run a binary search
  int low = 0;
  int high = static_cast<int>(corner_segment_bounds.size()) - 1;
  while (low <= high) {
    // Implicitly "rounds" down
    const int mid = low + (high - low) / 2;

    if (mid == corner_segment_bounds.size() - 1) {
      return corner_segment_bounds.size() - 1;
    } else if (mid == 0) {
      return 0;
    } else if (corner_segment_bounds[mid].first <= route_index &&
               corner_segment_bounds[mid + 1].first >= route_index) {
      return mid;
    }

    if (corner_segment_bounds[mid].first < route_index) {
      low = mid + 1;
    } else {
      high = mid - 1;
    }
  }

  RUNTIME_EXCEPTION(
      false,
      "The binary search failed on a continuous range. Check implementation. "
      "This should be impossible");
}

void FSGPRoute::init_longest_straight() {
  double longest_straight = 0.0;
  const size_t num_corners = corner_segment_bounds.size();

  for (size_t i = 0; i < num_corners; i++) {
    double straight_distance = 0.0;
    const std::pair<size_t, size_t> current_corner = corner_segment_bounds[i];
    std::pair<size_t, size_t> next_corner;
    if (i < num_corners - 1) {
      next_corner = corner_segment_bounds[i + 1];
    } else {
      next_corner = corner_segment_bounds[0];
    }
    if (route_distances.is_empty()) {
      straight_distance = calculate_segment_distance(
          this->route_points, current_corner.second, next_corner.first);
    } else {
      straight_distance =
          route_distances.get_value(current_corner.second, next_corner.first);
    }

    longest_straight = std::max(straight_distance, longest_straight);
  }
}

std::vector<size_t> FSGPRoute::get_overlapping_corners(
    const std::pair<size_t, size_t>& segment) const {
  std::vector<size_t> ret;
  size_t counter = 0;
  for (const auto& corner : corner_segment_bounds) {
    if (segment.first < segment.second) {
      // Normal segment
      if (segment.first < corner.second && segment.second > corner.first) {
        ret.push_back(counter);
      }
    } else {
      // Wrap-around segment
      if (segment.first < corner.second || segment.second > corner.second) {
        ret.push_back(counter);
      }
    }
    counter++;
  }
  return ret;
}

ASCRoute::ASCRoute(const std::filesystem::path& route_path,  // NOLINT
                   std::filesystem::path loop_config_dir) {  // NOLINT
  // TODO: Load the base route here
  init_loops(std::move(loop_config_dir));  // NOLINT
}

void ASCRoute::init_loops(const std::filesystem::path& loop_config_dir) {
  RUNTIME_EXCEPTION(std::filesystem::exists(loop_config_dir),
                    "Directory {} does not exist", loop_config_dir.string());
  RUNTIME_EXCEPTION(std::filesystem::is_directory(loop_config_dir),
                    "Loop Directory {} is not a directory",
                    loop_config_dir.string());

  // Iterate through CSV files
  for (const auto& entry :
       std::filesystem::directory_iterator(loop_config_dir)) {
    if (entry.is_regular_file() && entry.path().extension() == ".csv") {
      add_loop(entry);
    }
  }
}

util::type::Coord ASCRoute::find_base_route_start(
    util::type::Coord loop_start_coord) {
  RUNTIME_EXCEPTION(!base_route_points.empty(), "Base leg vector is empty");

  util::type::Coord closest;
  double best_dist = std::numeric_limits<double>::max();

  for (const auto& point : base_route_points) {
    const double distance =
        util::geo::get_distance(loop_start_coord, point);  // or Euclidean
    if (distance < best_dist) {
      best_dist = distance;
      closest = point;
    }
  }

  return closest;
}

bool ASCRoute::is_loop_start(util::type::Coord route_coord) const {
  return coord_to_loop.contains(route_coord);
}

CoordVec* ASCRoute::get_loop_points(util::type::Coord route_coord) const {
  if (!is_loop_start(route_coord)) {
    return nullptr;
  }
  return coord_to_loop.at(route_coord);
}

void ASCRoute::add_loop(const std::filesystem::path& loop_file_path) {
  std::fstream loop_route(loop_file_path);
  RUNTIME_EXCEPTION(loop_route.is_open(), "Loop route file not found {}",
                    loop_file_path.string());

  util::type::Coord base_route_coord;
  CoordVec loop_points;
  std::string line;
  bool is_first = true;

  while (std::getline(loop_route, line)) {
    std::stringstream linestream(line);
    std::string latitude, longitude, alt;
    util::type::Coord coord{};
    std::optional<double> parsed_value;

    std::getline(linestream, latitude, ',');
    parsed_value = util::detail::convert_num<double>(latitude);
    RUNTIME_EXCEPTION(parsed_value.has_value(),
                      "Latitude {} in loop file is not a number", latitude);
    coord.lat = *parsed_value;

    std::getline(linestream, longitude, ',');
    parsed_value = util::detail::convert_num<double>(longitude);
    RUNTIME_EXCEPTION(parsed_value.has_value(),
                      "Longitude {} in loop file is not a number", longitude);
    coord.lon = *parsed_value;

    std::getline(linestream, alt, ',');
    parsed_value = util::detail::convert_num<double>(alt);
    RUNTIME_EXCEPTION(parsed_value.has_value(),
                      "Altitude {} in loop file is not a number", alt);
    coord.alt = *parsed_value;

    loop_points.emplace_back(coord);

    if (is_first) {
      base_route_coord = find_base_route_start(coord);
      is_first = false;
    }
  }

  spdlog::info("Loaded loop route {} with {} coordinates",
               loop_file_path.string(), std::to_string(loop_points.size()));

  loops.push_back(std::move(loop_points));
  coord_to_loop[base_route_coord] = &loops.back();
  loop_route.close();
}

TelemRoute::TelemRoute(const std::filesystem::path& route_path) {
  // Read route csv file
  std::fstream base_route(route_path);
  RUNTIME_EXCEPTION(base_route.is_open(), "Base route file {} not found",
                    route_path.string());

  this->route_length = 0.0;
  util::type::Coord last_coord;

  bool first_coord = true;
  std::string line;

  while (std::getline(base_route, line)) {
    std::stringstream linestream(line);
    std::string latitude, longitude, alt, timestamp, speed;
    std::optional<double> parsed_double_value;

    util::type::Coord coord{};
    std::getline(linestream, latitude, ',');
    parsed_double_value = util::detail::convert_num<double>(latitude);
    RUNTIME_EXCEPTION(parsed_double_value.has_value(),
                      "{} could not be converted to a latitude number",
                      latitude);
    coord.lat = *parsed_double_value;

    std::getline(linestream, longitude, ',');
    parsed_double_value = util::detail::convert_num<double>(longitude);
    RUNTIME_EXCEPTION(parsed_double_value.has_value(),
                      "{} could not be converted to a longitude number",
                      longitude);
    coord.lon = *parsed_double_value;

    std::getline(linestream, alt, ',');
    parsed_double_value = util::detail::convert_num<double>(alt);
    RUNTIME_EXCEPTION(parsed_double_value.has_value(),
                      "{} could not be converted to a latitude number", alt);
    coord.alt = *parsed_double_value;

    route_points.emplace_back(coord);

    std::getline(linestream, timestamp, ',');
    timestamps.emplace_back(timestamp);

    std::getline(linestream, speed, ',');
    parsed_double_value = util::detail::convert_num<double>(speed);
    RUNTIME_EXCEPTION(parsed_double_value.has_value(),
                      "{} could not be converted to a longitude number", speed);
    speeds.emplace_back(*parsed_double_value);

    if (!first_coord) {
      this->route_length += util::geo::get_distance(last_coord, coord);
    } else {
      first_coord = false;
    }

    last_coord = coord;
  }
}
