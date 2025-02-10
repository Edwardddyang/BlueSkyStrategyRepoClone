#include <stdlib.h>

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

#include "utils/Defines.hpp"
#include "route/Route.hpp"
#include "config/Config.hpp"
#include "utils/Utilities.hpp"
#include "utils/Geography.hpp"

Route::Route(const std::filesystem::path route_path, const bool init_control_stops,
            const std::filesystem::path cornering_bounds_path,
            const std::filesystem::path precomputed_distances_path,
            const bool precompute_distances) {
  init_base_route(route_path);

  if (init_control_stops) {
    this->init_control_stops();
  }

  if (!cornering_bounds_path.empty()) {
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

void Route::init_base_route(const std::filesystem::path route_path) {
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

/** @brief Calculate acceleration in m/s^2 from the starting coordinate to the ending coordinate
 * @param coords Coordinate points from the route
 * @param starting_idx Starting coordinate i.e. coords[starting_idx]
 * @param ending_idx Ending coordinate i.e. coords[ending_idx]
 * @param init_speed Speed at the starting coordinate in m/s
 * @param ending_speed Speed at the ending coordinate in m/s
 */
static double calc_segment_acceleration(const std::vector<Coord>& coords, const size_t starting_idx,
                                        const size_t ending_idx, const double init_speed, const double ending_speed) {
  if (starting_idx == ending_idx || init_speed == ending_speed) {
    return 0.0;
  }

  RUNTIME_EXCEPTION(starting_idx >= 0 && starting_idx < coords.size() && ending_idx >= 0 && ending_idx < coords.size(),
                    "Starting idx {} and ending idx {} are not properly ordered", starting_idx, ending_idx);

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

  return calc_acceleration(init_speed, ending_speed, accumulated_distance);
}

RacePlan Route::segment_route_acceleration(const unsigned segment_idx_seed,
                                          const unsigned speed_seed,
                                          const unsigned acceleration_seed,
                                          const unsigned skip_seed,
                                          const unsigned loop_seed,
                                          const int max_num_loops,
                                          const double max_speed,
                                          const double max_acceleration,
                                          const int max_iters) {
  RUNTIME_EXCEPTION(route_points.size() > 0, "Route points not yet loaded");
  RUNTIME_EXCEPTION(cornering_segment_bounds.size() > 0, "Cornering bounds CSV not yet read");
  RUNTIME_EXCEPTION(cornering_segment_bounds.size() == cornering_speed_bounds.size(),
                    "Number of corners must equal the length of the cornering speeds");
  RUNTIME_EXCEPTION(max_speed > 0.0 && max_acceleration > 0.0, "Max. speed and max. acceleration must be > 0");
  RUNTIME_EXCEPTION(!route_distances.is_empty(), "Route distances must be pre-computed for acceleration segmenting");
  std::mt19937 idx_rng(segment_idx_seed);
  std::mt19937 speed_rng(speed_seed);
  std::mt19937 acceleration_rng(acceleration_seed);
  std::mt19937 loop_gen(loop_seed);
  std::mt19937 skip_gen(skip_seed);

  // Route attributes
  const size_t num_corners = cornering_segment_bounds.size();

  // RacePlan attributes
  std::vector<std::vector<std::pair<size_t, size_t>>> all_segments;
  std::vector<std::vector<std::pair<double, double>>> all_segment_speeds;
  std::vector<std::vector<bool>> all_acceleration_segments;
  std::vector<std::vector<double>> all_acceleration_values;

  // Randomly select the number of loops to complete
  std::uniform_int_distribution<unsigned int> dis(1, max_num_loops);
  const size_t num_loops = static_cast<size_t>(dis(loop_gen));

  // Temp data holders for each segment
  std::pair<size_t, size_t> segment = {0, 0};
  std::pair<double, double> segment_speed = {0.0, 0.0};
  bool acceleration = false;
  double acceleration_value = 0.0;
  double next_loop_starting_speed = 0.0;
  double loop_ending_speed = 0.0;

  // Uniform distributions used to randomly select locations, speeds
  // and accelerations
  // Note that std::uniform_<type>_distribution is inclusive on both sides
  std::uniform_int_distribution<size_t> idx_dist;
  std::uniform_real_distribution<double> speed_dist;
  std::uniform_real_distribution<double> acceleration_dist = std::uniform_real_distribution<double>(0.1, max_acceleration);
  std::uniform_real_distribution<double> skip_dist = std::uniform_real_distribution<double>(0.0, 1.0);

  // Maximum number of sampling iterations
  int count = 0;
  for (size_t loop_idx=0; loop_idx < num_loops; loop_idx++) {
    // For each loop:
    // Iterate through the cornering intervals. For each corner, we must create three/four segments
    // according to the following drawing:
    //          | |     |
    //          |0|     |    Direction of travel
    //          |_|    \|/
    //          | |
    //          |1|
    //          |_|
    //          / /
    //         /2/
    // _______/ /
    // __3___|_/
    // 0. Constant speed segment coming from the last segment. This can be an acceleration segment
    //    if it's the first corner of the first loop(we're starting at 0m/s)
    // 1. A deceleration/acceleration segment going into the corner
    // 2. A constant speed segment for taking the corner. These segments
    //    are fixed as coming from the corners.csv
    // 3. An acceleration/deceleration segment leaving the corner
    //
    // Note that segment 0 does not exist for the 1st corner
    // (Don't try to get gpt to draw something like this for you. It doesn't really do well
    // or i'm just bad at prompting)

    // Store results for current loop
    std::vector<std::pair<size_t, size_t>> loop_segments;
    std::vector<std::pair<double, double>> loop_segment_speeds;
    std::vector<bool> loop_acceleration_segments;
    std::vector<double> loop_acceleration_values;

    // Labmda to add a new segment to the loop
    auto add_segment = [&]() {
      loop_segments.emplace_back(segment);
      loop_segment_speeds.emplace_back(segment_speed);
      loop_acceleration_segments.emplace_back(acceleration);
      loop_acceleration_values.emplace_back(acceleration_value);
    };

    size_t next_corner_start;
    double next_corner_max_speed;
    for (size_t corner_idx=0; corner_idx < num_corners; corner_idx++) {
      const size_t corner_start = cornering_segment_bounds[corner_idx].first;
      const size_t corner_end = cornering_segment_bounds[corner_idx].second;
      const double max_corner_speed = cornering_speed_bounds[corner_idx];

      if (corner_idx != num_corners - 1) {
        next_corner_start = cornering_segment_bounds[corner_idx + 1].first;
        next_corner_max_speed = cornering_speed_bounds[corner_idx + 1];
      }

      // Create distributions for selecting a deceleration starting index
      if (corner_idx == 0) {
        // For the first corner, the range of possible values are from [0, corner_idx-1]
        idx_dist = std::uniform_int_distribution<size_t>(0, corner_start - 1);
      } else {
        // For any corner > 0, we start anywhere from the end of the last segment to
        // the starting index of the corner
        idx_dist = std::uniform_int_distribution<size_t>(segment.second, corner_start - 1);
      }
  
      size_t deceleration_start_idx = idx_dist(idx_rng);
      double deceleration_starting_speed = 0.0;

      // Create segment 0
      if (corner_idx > 0 || loop_idx != 0) {
        // In the general case (non first corner of the first loop):
        // We need to ensure that the selected deceleration start index allows for enough
        // distance to decelerate from the constant speed of this segment to at least the
        // the maximum corner speed. If not, we back off the deceleration start index by 1
        // repeatedly
        bool valid_deceleration_idx = false;
        deceleration_starting_speed = segment_speed.second;
        while (!valid_deceleration_idx) {
          const double distance_to_corner = route_distances.get_value(deceleration_start_idx, corner_start);
          const double deceleration = calc_segment_acceleration(route_points,
                                                              deceleration_start_idx,
                                                              corner_start,
                                                              segment_speed.second,
                                                              max_corner_speed * 0.95);  // Keep the driver safe
          if (std::abs(deceleration) > max_acceleration) {
            deceleration_start_idx--;
          } else {
            valid_deceleration_idx = true;
          }

          if (deceleration_start_idx < segment.second) {
            return RacePlan("Too much backoff");
          }

          count++;
          if (count == max_iters) {
            return RacePlan("Max iters reached on back off");
          }
        }

        // Add this segment if the deceleration start index > last segment's end index
        if (segment.second != deceleration_start_idx) {
          segment = {segment.second, deceleration_start_idx};
          segment_speed = {deceleration_starting_speed, deceleration_starting_speed};
          acceleration = false;
          acceleration_value = 0.0;
          add_segment();
        }
      } else if (deceleration_start_idx != 0.0) {
        // For segment 0 of the very first corner of the very first loop, we accelerate from
        // 0.0 to some positive speed x. We select this positive speed x with two constraints:
        //  1. The acceleration from 0.0 to x is of valid magnitude
        //  2. The deceleration from x to the maximum corner speed is valid
        // Repeatedly sample from valid acceleration magnitudes
        bool valid_deceleration_idx = false;
        while (!valid_deceleration_idx) {
          acceleration_value = acceleration_dist(acceleration_rng);
          deceleration_starting_speed = calc_final_speed_a(0.0, acceleration_value,
                                                      route_distances.get_value(0, deceleration_start_idx));
          const double deceleration_value = calc_acceleration(deceleration_starting_speed,
                                                              max_corner_speed * 0.95,
                                                              route_distances.get_value(deceleration_start_idx,
                                                                                        corner_start));
          if (std::abs(deceleration_value) < max_acceleration) {
            valid_deceleration_idx = true;
          }
        }
        segment_speed = {0, deceleration_starting_speed};
        segment = {0, deceleration_start_idx};
        acceleration = acceleration_value != 0.0;
        add_segment();

        count++;
        if (count == max_iters) {
          return RacePlan("Invalid deceleration could not be found for very first corner");
        }
      }

      // Define segment 1 (deceleration).
      // Select deceleration value based on two constraints:
      // 1. Deceleration's ending speed <= max_corner_speed * 0.95
      // 2. Deceleration's ending speed allows the car to get to the next corner's
      // maximum speed
      bool valid_corner_speed = false;
      bool can_reach_next_corner = false;
      speed_dist = std::uniform_real_distribution<double>(0.2 * max_corner_speed, max_corner_speed);
      count = 0;
      while (!valid_corner_speed || !can_reach_next_corner) {
        const double corner_speed = speed_dist(speed_rng);

        // Deceleration value
        const double proposed_acceleration_value = calc_acceleration(deceleration_starting_speed,
                                                                     corner_speed, route_distances.get_value(
                                                                      deceleration_start_idx, corner_start
                                                                     ));
        
        // Calculate required acceleration to reach the next corner based on the projected corner speed
        if (corner_idx != num_corners - 1) {
          const double necessary_deceleration = calc_acceleration(corner_speed, next_corner_max_speed * 0.95,
                                                                  route_distances.get_value(corner_end + 1,
                                                                  next_corner_start));
          can_reach_next_corner = std::abs(necessary_deceleration) < max_acceleration;
        } else {
          can_reach_next_corner = true;
        }

        if (std::abs(proposed_acceleration_value) < max_acceleration && can_reach_next_corner) {
          segment_speed = {deceleration_starting_speed, corner_speed};
          segment = {deceleration_start_idx, corner_start};
          acceleration_value = proposed_acceleration_value;
          acceleration = acceleration_value != 0.0;
          add_segment();
          valid_corner_speed = true;
        }

        count++;
        if (count == max_iters) {
          return RacePlan("Could not find required deceleration into the corner");
        }
      }

      // Create segment 2 - corner (constant speed)
      segment = {segment.second, corner_end};
      segment_speed = {segment_speed.second, segment_speed.second};
      acceleration = false;
      acceleration_value = 0.0;
      add_segment();

      // Create segment 3 - accelerate out of the corner
      // Sample a chance to not accelerate at all and continue at the cornering speed
      const double straight_distance = route_distances.get_value(corner_end, next_corner_start);

      // Shorter the straight, higher the probability for skipping the accleration segment
      // For the longest straight, 
      const double probability = 1.0 - straight_distance / longest_straight;
      const double sample = skip_dist(skip_gen);
      if (sample < probability) {
        continue;
      }

      if (corner_idx < num_corners - 1) {
        idx_dist = std::uniform_int_distribution<size_t>(corner_end + 1, next_corner_start - 1);
      } else {
        idx_dist = std::uniform_int_distribution<size_t>(corner_end + 1, route_points.size() - 1);
      }

      // We need to ensure that the acceleration chosen and the ending acceleration idx allows for
      // enough distance to decelerate into the next corner
      bool valid_acceleration = false;
      double acceleration_bound = max_acceleration;
      count = 0;
      while (!valid_acceleration) {
        const size_t acceleration_end_idx = idx_dist(idx_rng);
        acceleration_value = acceleration_dist(acceleration_rng);
        const double acceleration_ending_speed = calc_final_speed_a(segment_speed.second, acceleration_value,
                                                route_distances.get_value(corner_end, acceleration_end_idx));
        
        const double required_deceleration = calc_acceleration(acceleration_ending_speed,
                                                               next_corner_max_speed * 0.95,
                                                               route_distances.get_value(acceleration_end_idx,
                                                                                        next_corner_start));
        
        if (std::abs(required_deceleration) < max_acceleration) {
          segment = {corner_end, acceleration_end_idx};
          segment_speed = {segment_speed.second, acceleration_ending_speed};
          acceleration = acceleration_value != 0.0;
          add_segment();
          valid_acceleration = true;
        }
        count++;
        if (count == max_iters) {
          // If no possible acceleration can be found, just skip it and continue at the cornering speed
          break;
        }
      }
    }

    // We need one last segment connecting the last index to the 0th index.
    // Designate this as constant speed
    bool valid_acceleration = false;
    const size_t last_loop_idx = segment.second;
    segment = {last_loop_idx, 0};
    loop_ending_speed = segment_speed.second;
    segment_speed = {loop_ending_speed, loop_ending_speed};
    acceleration_value = 0.0;
    acceleration = acceleration_value != 0.0;
    add_segment();

    // Store results for this loop
    all_segments.push_back(loop_segments);
    all_segment_speeds.push_back(loop_segment_speeds);
    all_acceleration_segments.push_back(loop_acceleration_segments);
    all_acceleration_values.push_back(loop_acceleration_values);
  }
  return RacePlan(all_segments, all_segment_speeds, all_acceleration_segments, all_acceleration_values);
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
  std::cout << std::flush;
}

bool RacePlan::validate_members(const std::vector<Coord>& route_points) const {
  RUNTIME_EXCEPTION(!empty, "RacePlan is empty");
  RUNTIME_EXCEPTION(segments.size() > 0, "Validate_members() for RacePlan called before segments were set.");
  RUNTIME_EXCEPTION(segments.size() == segment_speeds.size() && segments.size() == acceleration_segments.size() &&
                    acceleration_segments.size() == acceleration.size(),
                    "RacePlan not properly created. Speed profile, route segments and acceleration segments"
                    "have unequal number of loops");
  const size_t num_loops = segments.size();
  const double tolerance = 0.0001;  // Tolerance for comparing acceleration values
  std::pair<double, double> last_segment_speeds;
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
    RUNTIME_EXCEPTION(loop_segments[0].first == 0, "Segments must start at index 0");

    if (num_loops == 1) {
      // RUNTIME_EXCEPTION(loop_segments[num_segments-1].second == route_points.size()-1,
      //                   "Last segment's ending point must "
      //                   " be the last index of the route");
    } else {
      RUNTIME_EXCEPTION(loop_segments[num_segments-1].second == 0,
                        "Last segment's ending point must be 0 i.e. wrap-around");
    }
    if (loop_idx > 0) {
      RUNTIME_EXCEPTION(loop_segments_speeds[0].first == last_segment_speeds.second,
                        "Last loop's ending speed must be the starting speed of the first segment in the next loop.");
    }
    for (size_t i=0; i < num_segments; i++) {
      RUNTIME_EXCEPTION(loop_segments_speeds[i].first >= 0.0 && loop_segments_speeds[i].second >= 0.0,
                        "Segment speed in segment {} is not positive", i);

      if (i < num_segments-1) {
        RUNTIME_EXCEPTION(loop_segments[i].first <= loop_segments[i].second, "Segment start must be <= segment end");
        RUNTIME_EXCEPTION(loop_segments[i].second == loop_segments[i+1].first,
                          "Segments must be continuous i.e. segment end = next segment start."
                          " Invalid on segment {}, loop {}", i, loop_idx);
        RUNTIME_EXCEPTION(loop_segments_speeds[i].second == loop_segments_speeds[i+1].first,
                          "Ending speed of segment {} must equal starting speed of next segment", i);
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
        const double calculated_acceleration = calc_segment_acceleration(route_points, loop_segments[i].first,
                                                                         loop_segments[i].second, starting_speed,
                                                                         ending_speed);
        RUNTIME_EXCEPTION(std::abs(acceleration_value - calculated_acceleration) < tolerance,
                          "Acceleration is invalid: segment {} in loop {}. Calculated is {}",
                          i, loop_idx, calculated_acceleration);
      }
    }

    last_segment_speeds = loop_segments_speeds[num_segments-1];
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

  if (this->segments.size() == 0 && this->segment_speeds.size() == 0 &&
      this->acceleration_segments.size() == 0 && this->acceleration.size() == 0) {
    empty = true;
  } else {
    empty = false;
  }
}
