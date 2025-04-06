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

#include "utils/Defines.hpp"
#include "route/Route.hpp"
#include "config/Config.hpp"
#include "utils/Utilities.hpp"
#include "utils/Logger.hpp"
#include "utils/Geography.hpp"
#include "utils/CustomException.hpp"

Route::Route(const std::filesystem::path route_path, const bool init_control_stops,
            const std::filesystem::path cornering_bounds_path,
            const std::filesystem::path precomputed_distances_path,
            const bool precompute_distances) {
  init_base_route(route_path);

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

std::pair<double, double> Route::find_valid_speeds(const double& acceleration, const double& mass,
  const double& max_car_speed, const double& max_motor_power) {
  // If 1 m/s isn't viable, there's no point considering anything higher
  if (acceleration * mass >= max_motor_power) {
    return {-1.0, -1.0};
  }

  double min_speed = 0.0;
  double max_speed = 1.0;

  // Don't binary search because the max speed will likely be small
  while (max_speed < max_car_speed) {
    double next_max_speed = max_speed + 0.1;

    if (next_max_speed * acceleration * mass >= max_motor_power) {
      return {min_speed, max_speed};
    }

    max_speed = next_max_speed;
  }

  return {min_speed, max_speed};
}

bool Route::does_acceleration_exist(double init_speed, double distance, std::pair<double, double> speed_range,
                                    double deceleration_bound, double acceleration_bound) {
  RUNTIME_EXCEPTION(speed_range.first <= speed_range.second, "Speed range must be ordered as {smaller, bigger}");
  RUNTIME_EXCEPTION(init_speed >= 0.0, "Initial speed must be >= 0 m/s");
  RUNTIME_EXCEPTION(distance >= 0.0, "Distance must be >= 0.0");
  RUNTIME_EXCEPTION(acceleration_bound > 0.0, "Acceleration bound must be positive");
  double decel_bound = std::abs(deceleration_bound);

  if (init_speed >= speed_range.first && init_speed <= speed_range.second) {
    return true;
  }

  if (init_speed < speed_range.first) {
    double acceleration = 0.0;
    while (acceleration < acceleration_bound) {
      const double final_speed = calc_final_speed_a(init_speed, acceleration, distance);
      if (final_speed >= speed_range.first && final_speed <= speed_range.second) {
        return true;
      }
      acceleration = acceleration + 0.1;
    }
  }

  if (init_speed > speed_range.second) {
    double deceleration = 0.0;
    while (std::abs(deceleration) < decel_bound) {
      const double final_speed = calc_final_speed_a(init_speed, deceleration, distance);
      if (final_speed >= speed_range.first && final_speed <= speed_range.second) {
        return true;
      }
      deceleration = deceleration - 0.1;
    }
  }

  return false;
}

RacePlan Route::segment_route_acceleration(const unsigned segment_idx_seed,
                                          const unsigned speed_seed,
                                          const unsigned acceleration_seed,
                                          const unsigned skip_seed,
                                          const unsigned loop_seed,
                                          const int max_num_loops,
                                          const double car_mass,
                                          const double max_speed,
                                          const double max_motor_power,
                                          const double max_acceleration,
                                          const double max_deceleration,
                                          const int max_iters) {
  RUNTIME_EXCEPTION(route_points.size() > 0, "Route points not yet loaded");
  RUNTIME_EXCEPTION(cornering_segment_bounds.size() > 0, "Cornering bounds CSV not yet read");
  RUNTIME_EXCEPTION(cornering_segment_bounds.size() == cornering_speed_bounds.size(),
                    "Number of corners must equal the length of the cornering speeds");
  RUNTIME_EXCEPTION(max_speed > 0.0 && max_acceleration > 0.0 && max_deceleration > 0,
                    "Max. speed, max. deceleration and max. acceleration must be > 0");
  RUNTIME_EXCEPTION(max_motor_power > 0.0, "Maximum motor power must be > 0");
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
  double loop_ending_speed = 0.0;

  // Uniform distributions used to randomly select locations, speeds
  // and accelerations
  // Note that std::uniform_<type>_distribution is inclusive on both sides
  std::uniform_int_distribution<size_t> idx_dist;
  std::uniform_real_distribution<double> speed_dist;
  std::uniform_real_distribution<double> acceleration_dist(0.1, max_acceleration);
  std::uniform_real_distribution<double> skip_dist(0.0, 1.0);

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

    // Lambda to print a segment (debugging purposes)
    auto print_segment = [&]() {
      std::cout << "Segment: [" << segment.first << "," << segment.second << "]" << std::endl;
      std::cout << "Segment Speed: [" << segment_speed.first << "," << segment_speed.second << "]" << std::endl;
      std::cout << "Acceleration: " << (acceleration ? "True" : "False") << std::endl;
      std::cout << "Acceleration Value: " << acceleration_value << std::endl;
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
        // distance to decelerate from the constant speed of this segment to a valid speed
        // in the corner's range of speeds. If not, we back off the deceleration start index
        // by 1 repeatedly
        bool valid_deceleration_idx = false;
        deceleration_starting_speed = segment_speed.second;
        while (!valid_deceleration_idx) {
          count++;
          if (count == max_iters) {
            return RacePlan("Max iters reached on back off");
          }

          const double distance_to_corner = route_distances.get_value(deceleration_start_idx, corner_start);

          if (!does_acceleration_exist(segment.second, distance_to_corner,
                                      {max_corner_speed * 0.05, max_corner_speed * 0.95},
                                      max_deceleration, max_acceleration)) {
            deceleration_start_idx--;
          } else {
            valid_deceleration_idx = true;
          }

          if (deceleration_start_idx < segment.second) {
            return RacePlan("Too much backoff");
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
        //  1. The acceleration from 0.0 to x is of valid magnitude - cannot exceed maximum motor power
        //  2. The deceleration from x to the maximum corner speed is valid
        // Repeatedly sample from valid acceleration magnitudes
        bool valid_deceleration_idx = false;
        while (!valid_deceleration_idx) {
          count++;
          if (count == max_iters) {
            return RacePlan("Invalid deceleration could not be found for very first corner");
          }

          acceleration_value = acceleration_dist(acceleration_rng);
          const double distance_to_corner = route_distances.get_value(deceleration_start_idx, corner_start);
          const double distance_to_deceleration_start = route_distances.get_value(0, deceleration_start_idx);
          deceleration_starting_speed = calc_final_speed_a(0.0, acceleration_value, distance_to_deceleration_start);
          const double motor_power = car_mass * acceleration_value * deceleration_starting_speed;  // Use highest speed

          if (motor_power < motor_power && acceleration_value < max_acceleration &&
              does_acceleration_exist(0.0, distance_to_corner, {max_corner_speed * 0.05, max_corner_speed},
                                      max_deceleration, max_acceleration)) {
            valid_deceleration_idx = true;
          }
        }
        segment_speed = {0, deceleration_starting_speed};
        segment = {0, deceleration_start_idx};
        acceleration = acceleration_value != 0.0;
        add_segment();
      }

      // Define segment 1 (deceleration).
      // Select deceleration value based on two constraints:
      // 1. Deceleration's ending speed <= max_corner_speed * 0.95
      // 2. Chosen corner speed allows the car to get to the next corner's maximum speed
      bool valid_corner_speed = false;
      bool can_reach_next_corner = false;
      speed_dist = std::uniform_real_distribution<double>(0.1 * max_corner_speed, max_corner_speed);
      count = 0;
      while (!valid_corner_speed || !can_reach_next_corner) {
        const double corner_speed = speed_dist(speed_rng);

        // Deceleration value
        const double distance_to_corner = route_distances.get_value(deceleration_start_idx, corner_start);
        const double proposed_acceleration_value = calc_acceleration(deceleration_starting_speed,
                                                                     corner_speed, distance_to_corner);

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

RacePlan Route::segment_route_corners(const int max_num_loops,
                                      const double car_mass,
                                      const double max_speed,
                                      const double max_motor_power,
                                      const double max_acceleration,
                                      const double max_deceleration,
                                      const double preferred_acceleration,
                                      const double preferred_deceleration,
                                      const unsigned speed_seed,
                                      const unsigned loop_seed,
                                      const unsigned aggressive_seed,
                                      const unsigned idx_seed,
                                      const unsigned acceleration_seed,
                                      const double corner_speed_min,
                                      const double corner_speed_max,
                                      const double aggressive_straight_threshold,
                                      const int num_repetitions,
                                      const double acceleration_power_budget,
                                      const int max_iters,
                                      bool log) {
  RUNTIME_EXCEPTION(route_points.size() > 0, "Route points not yet loaded");
  RUNTIME_EXCEPTION(cornering_segment_bounds.size() > 0, "There must exist at least 1 corner");
  RUNTIME_EXCEPTION(cornering_segment_bounds.size() == cornering_speed_bounds.size(),
                    "Number of corners must equal the length of the cornering speeds");
  RUNTIME_EXCEPTION(max_speed > 0.0 && max_acceleration > 0.0,
                    "Max. speed, and max. acceleration must be > 0");
  RUNTIME_EXCEPTION(max_deceleration < 0.0, "Max. deceleration must be negative");
  RUNTIME_EXCEPTION(max_motor_power > 0.0, "Maximum motor power must be > 0");
  RUNTIME_EXCEPTION(!route_distances.is_empty(), "Route distances must be pre-computed for acceleration segmenting");
  RUNTIME_EXCEPTION(corner_speed_min >= 0.0 && corner_speed_max <= 1.0,
                    "Corner speed minimum parameter must be in range [0.0, 1.0]");
  RUNTIME_EXCEPTION(corner_speed_max >= 0.0 && corner_speed_max <= 1.0,
                    "Corner speed maximum parameter must be in range [0.0, 1.0]");
  RUNTIME_EXCEPTION(corner_speed_min < corner_speed_max,
                    "Corner speed minimum must be strictly less than corner speed maximum");
  RUNTIME_EXCEPTION(aggressive_straight_threshold >= 0.0, "Aggressive straight threshold must be >= 0.0");
  RUNTIME_EXCEPTION(num_repetitions >= 1, "Num repetitions per chunk must >= 1");
  RUNTIME_EXCEPTION(preferred_deceleration >= max_deceleration,
                    "Preferred deceleration must have lower magnitude than maximum deceleration");
  RUNTIME_EXCEPTION(preferred_acceleration <= max_acceleration,
                    "Preferred acceleration must have lower magnitude than maximum acceleration");

  // Create random number generators
  std::mt19937 speed_rng(speed_seed);
  std::mt19937 loop_rng(loop_seed);
  std::mt19937 aggressive_rng(aggressive_seed);
  std::mt19937 idx_rng(idx_seed);
  std::mt19937 acceleration_rng(acceleration_seed);

  // Route attributes
  const size_t num_corners = cornering_segment_bounds.size();

  // RacePlan attributes
  std::vector<std::vector<std::pair<size_t, size_t>>> all_segments;
  std::vector<std::vector<std::pair<double, double>>> all_segment_speeds;
  std::vector<std::vector<bool>> all_acceleration_segments;
  std::vector<std::vector<double>> all_acceleration_values;
  std::vector<std::vector<double>> all_segment_distances;

  // Randomly select the number of loops to complete
  std::uniform_int_distribution<unsigned int> dis(1, max_num_loops);
  const size_t num_loops = static_cast<size_t>(dis(loop_rng));

  const double acceleration_power_allowance = max_motor_power * acceleration_power_budget;

  FileLogger logger;
  logger = FileLogger("segment_route_corners.log", log);
  logger("Starting route segmentation with the following parameters:");
  logger("Loop seed: " + std::to_string(loop_seed));
  logger("Speed seed: " + std::to_string(speed_seed));
  logger("Aggressive straight sampling seed: " + std::to_string(aggressive_seed));
  logger("Index selection seed: " + std::to_string(idx_seed));
  logger("Acceleration seed: " + std::to_string(acceleration_seed));
  logger("Maximum number of loops: " + std::to_string(max_num_loops));
  logger("Maximum car speed: " + std::to_string(max_speed) + " m/s");
  logger("Maximum acceleration: " + std::to_string(max_acceleration) + " m/s^2");
  logger("Maximum deceleration: " + std::to_string(max_deceleration) + " m/s^2");
  logger("Maximum motor power: " + std::to_string(max_motor_power) + " W");
  logger("Preferred acceleration: " + std::to_string(preferred_acceleration) + " m/s^2");
  logger("Preferred deceleration: " + std::to_string(preferred_deceleration) + " m/s^2");
  logger("Corner speed minimum clamp: " + std::to_string(corner_speed_min));
  logger("Corner speed maximum clamp: " + std::to_string(corner_speed_max));
  logger("Aggressive straight threshold: " + std::to_string(aggressive_straight_threshold) + " m");
  logger("Number of loop repetitions per chunk: " + std::to_string(num_repetitions));
  logger("Maximum iteration count for sampling: " + std::to_string(max_iters));
  logger("Fractional acceleration power budget: " + std::to_string(acceleration_power_budget));
  logger("Randomly selected number of loops to complete: " + std::to_string(num_loops));
  logger("Motor power allowance is " + std::to_string(acceleration_power_allowance) + " W\n");

  // NOTE:
  // We use the phrase "route index" to denote an index in the route_points array
  // We use the phrase "corner index" to denote an index in the cornering_segment_bounds array

  // Construct plan loop by loop
  bool is_first_segment = true;
  ///////////////////////////////////////////////////////////////////////////////////
  ///////////////////////// State variables for each corner /////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////

  // Whether a straight is taken aggressively
  bool aggressive_straight = false;

  // Whether a straight is taking normally
  bool normal_straight = false;

  // Attributes of the next and last corner
  double prev_corner_max_speed = 0.0;  // Maximum speed of the previous corner
  double next_corner_max_speed = 0.0;  // Maximum speed of the next speed
  size_t next_corner_start = 0;        // Starting route index of the next corner
  size_t prev_corner_end = 0;          // Ending route index of the previous corner

  // Chosen speed of the last corner whose maximum speed is less than the route speed
  double last_real_corner_speed = 0.0;

  // The corner index of the last corner whose maximum speed is less than the route speed
  size_t last_real_corner_idx = 0;

  // The route index of the end of the last corner whose maximum speed is less than
  // the route speed
  size_t last_real_corner_end = 0;

  // The route index where a non acceleration zone starts on a long straight
  size_t non_acceleration_zone_start = 0;

  // The maximum distance the car can accelerate on a straight
  double max_acceleration_distance = 0.0;

  // The proposed corner speed
  int proposed_corner_speed = 0.0;

  // Distance to the next corner
  double distance_to_next_corner = 0.0;

  // General boolean variable for determining validity of a randomly chosen speed, acceleration, index etc.
  bool valid = false;

  // Counter for tracking the number of sampling iterations
  int count = 0;

  // Estimated instataneous motor power in W
  double instataneous_motor_power = 0.0;

  // Upper and lower bounds for speed
  int lower_bound_speed = 0;
  int upper_bound_speed = 0;

  ///////////////////////////////////////////////////////////////////////////////////

  for (size_t loop_idx = 0; loop_idx < num_loops; loop_idx++) {
    // Temp data holders for each segment
    std::pair<size_t, size_t> segment = {0, 0};
    std::pair<double, double> segment_speed = {0.0, 0.0};
    bool acceleration = false;
    double acceleration_value = 0.0;
    double segment_distance = 0.0;

    // Uniform distributions used to randomly select locations, speeds
    // and accelerations
    // Note that std::uniform_<type>_distribution is inclusive on both sides
    std::uniform_int_distribution<size_t> idx_dist;
    std::uniform_int_distribution<int> speed_dist;
    std::uniform_real_distribution<double> acceleration_dist(0.1, max_acceleration);
    std::uniform_real_distribution<double> aggressive_dist(0.0, 1.0);

    // Store results for current loop
    std::vector<std::pair<size_t, size_t>> loop_segments;
    std::vector<std::pair<double, double>> loop_segment_speeds;
    std::vector<bool> loop_acceleration_segments;
    std::vector<double> loop_acceleration_values;
    std::vector<double> loop_segment_distances;

    // Labmda to add a new segment to the loop
    auto add_segment = [&]() {
      loop_segments.emplace_back(segment);
      loop_segment_speeds.emplace_back(segment_speed);
      loop_acceleration_segments.emplace_back(acceleration);
      loop_acceleration_values.emplace_back(acceleration_value);
      loop_segment_distances.emplace_back(segment_distance);
    };

    // Lambda to convert segment information into a string (debugging purposes)
    auto get_segment_string = [&]() {
      std::stringstream ss;
      ss << "Segment: [" << segment.first << "," << segment.second << "]\n";
      ss << "Segment Distance: " << segment_distance << "m\n";
      ss << "Segment Speeds: [" << segment_speed.first << "," << segment_speed.second << "]\n";
      ss << "Acceleration: " << (acceleration ? "True\n" : "False\n");
      ss << "Acceleration Value: " << acceleration_value << "\n";
      return ss.str();
    };

    // We construct the segments between the previous corner to the current corner
    for (size_t corner_idx = 0; corner_idx < num_corners; corner_idx++) {
      // // Lambda function to create the segments for a specific corner
      // auto create_corner_segments = [&]() -> bool{

      logger("--------------CREATING SEGMENTS FOR CORNER " + std::to_string(corner_idx) +
             " OF LOOP " + std::to_string(loop_idx) + "--------------");
      // Determine attributes for current corner
      const size_t corner_start = cornering_segment_bounds[corner_idx].first;
      const size_t corner_end = cornering_segment_bounds[corner_idx].second;
      const double max_corner_speed = cornering_speed_bounds[corner_idx];
      logger("Corner maximum speed is " + std::to_string(max_corner_speed) + "m/s");
      logger("Corner start is index " + std::to_string(corner_start));
      logger("Corner end is index " + std::to_string(corner_end));

      // If the maximum cornering speed is greater than the route speed, then we can treat this as a
      // no acceleration zone and move onto the next corner
      if (max_corner_speed > this->max_route_speed) {
        logger("This corner has a speed greater than the route speed. "
               "Skip segment generation and treat as no-acceleration zone\n");
        continue;
      }

      // Determine attributes for previous corner
      if (corner_idx > 0) {
        prev_corner_end = cornering_segment_bounds[corner_idx-1].second;
        prev_corner_max_speed = cornering_speed_bounds[corner_idx-1];
      } else {
        prev_corner_end = cornering_segment_bounds[num_corners-1].second;
        prev_corner_max_speed = cornering_speed_bounds[num_corners-1];
      }

      // Determine attributes for next corner
      if (corner_idx < num_corners - 1) {
        next_corner_start = cornering_segment_bounds[corner_idx+1].first;
        next_corner_max_speed = cornering_speed_bounds[corner_idx+1];
      } else {
        next_corner_start = cornering_segment_bounds[0].first;
        next_corner_max_speed = cornering_speed_bounds[0];
      }

      // Get the length of the straight between the last real corner and the current corner
      last_real_corner_end = is_first_segment ? 0 : cornering_segment_bounds[last_real_corner_idx].second;

      const double straight_distance = route_distances.get_value(last_real_corner_end, corner_start);
      logger("Last real corner index was " + std::to_string(last_real_corner_idx));
      logger("Straight distance between index " + std::to_string(last_real_corner_end) + " and " +
             std::to_string(corner_start) + " is " + std::to_string(straight_distance) + "m");

      // In the case where the straight is composed of a non-acceleration zone (a corner with high speed limit),
      // then there is a constraint to complete the acceleration prior to the beginning of the non-acceleration
      // zone
      bool straight_includes_corners = !is_first_segment && last_real_corner_idx != corner_idx - 1 &&
                                       !(last_real_corner_idx == num_corners - 1 && corner_idx == 0);
      if (straight_includes_corners) {
        non_acceleration_zone_start = cornering_segment_bounds[last_real_corner_idx+1].first;
        max_acceleration_distance = route_distances.get_value(last_real_corner_end, non_acceleration_zone_start);
        logger("The straight includes corners with maximum speeds less than the route speed");
        logger("Maximum acceleration distance is: " + std::to_string(max_acceleration_distance) +
               " from index " + std::to_string(last_real_corner_end) + " to index " +
               std::to_string(non_acceleration_zone_start));
      } else {
        non_acceleration_zone_start = corner_start;
        // Spending half the straight accelerating seems wasteful. Maybe change later
        max_acceleration_distance = straight_distance * 0.5;
        logger("Maximum acceleration distance is: " + std::to_string(max_acceleration_distance) +
               " which is half the distance from index " + std::to_string(last_real_corner_end) + " to index " +
               std::to_string(corner_start));
      }

      // If the straight is long, sample a chance to take the straight aggressively
      double probability = 0.0;
      if (straight_distance > aggressive_straight_threshold) {
        // If the straight distance >= 2 * aggressive straight threshold, then we are guaranteed to take
        // the straight aggressively
        probability = std::min(1.0, (straight_distance - aggressive_straight_threshold)
                                    / aggressive_straight_threshold);
      }
      const double sample = aggressive_dist(aggressive_rng);
      if (sample < probability) {
        normal_straight = false;
        // Parameters to search for
        double proposed_deceleration;
        double proposed_acceleration;
        double acceleration_end_speed;
        double acceleration_distance;
        size_t acceleration_end_idx;

        // Route index of the starting point for deceleration. This must be in the range
        // of [last_real_corner_end, corner_start]
        size_t deceleration_start_idx;

        // Aggressive straight
        if (is_first_segment) {
          logger("Taking straight between starting line and " +
                  std::to_string(corner_idx) + " aggressively. Straight distance is " +
                  std::to_string(straight_distance) + "m");
        } else {
          logger("Taking straight between corner " + std::to_string(last_real_corner_idx) + " and " +
                  std::to_string(corner_idx) + " aggressively. Straight distance is " +
                  std::to_string(straight_distance) + "m");
        }
        logger("Last real corner speed: " + std::to_string(last_real_corner_speed) + "m/s");

        acceleration_dist = std::uniform_real_distribution<double>(0.0, max_acceleration);

        // Loop until we get valid parameters or we reach the maximum number of iterations
        valid = false;
        count = 0;
        while (!valid) {
          count++;
          if (count == max_iters) {
            return RacePlan("Maximum iterations reached");
          }
          proposed_acceleration = acceleration_dist(acceleration_rng);
          logger("Trying acceleration " + std::to_string(proposed_acceleration) + "m/s^2");

          // Ensure that the acceleration power budget is not exceeded
          instataneous_motor_power = proposed_acceleration * car_mass * last_real_corner_speed;
          if (instataneous_motor_power > acceleration_power_allowance) {
            logger(std::to_string(instataneous_motor_power) + " W exceeds maximum acceleration power budget");
            continue;
          }

          // Identify the maximum possible speed for travelling at the proposed acceleration
          double max_ending_speed = last_real_corner_speed - 0.1;
          double distance_travelled;
          do {
            max_ending_speed += 0.1;
            instataneous_motor_power = max_ending_speed * proposed_acceleration * car_mass;
            distance_travelled = calc_distance_a(last_real_corner_speed,
                                                 acceleration_end_speed,
                                                 proposed_acceleration);
          } while (instataneous_motor_power < acceleration_power_allowance &&
                   distance_travelled < max_acceleration_distance);

          logger("Maximum ending speed of the acceleration is " + std::to_string(max_ending_speed) + "m/s");

          if (static_cast<int>(last_real_corner_speed + 1) == static_cast<int>(max_ending_speed)) {
            logger("Maximum ending speed is too close to the previous corner speed. "
                   "The acceleration is too high. Trying again");
            continue;
          }

          // Pick a speed to end the acceleration
          valid = false;
          lower_bound_speed = static_cast<int>(last_real_corner_speed + 1.0);
          upper_bound_speed = static_cast<int>(max_ending_speed);
          speed_dist = std::uniform_int_distribution<int>(lower_bound_speed, upper_bound_speed);
          logger("Acceleration ending speed distribution created with lower bound " +
                  std::to_string(lower_bound_speed) + " m/s and upper bound " +
                  std::to_string(upper_bound_speed) + " m/s");

          acceleration_end_speed = speed_dist(speed_rng);
          logger("Trying acceleration ending speed " + std::to_string(acceleration_end_speed) + " m/s");
          acceleration_distance = calc_distance_a(last_real_corner_speed, acceleration_end_speed,
                                                  proposed_acceleration);
          logger("Acceleration distance is " + std::to_string(acceleration_distance) + " m");

          // Locate the smallest route index such that [last_real_corner_end, idx] is greater
          // than the acceleration distance
          acceleration_end_idx = last_real_corner_end;
          while (route_distances.get_value(last_real_corner_end, acceleration_end_idx) < acceleration_distance) {
            acceleration_end_idx = acceleration_end_idx == this->num_points - 1 ? 0 : acceleration_end_idx + 1;
          }

          // Select corner speed
          // This should likely be a distribution where the mean or mode (bump) considers both the acceleration
          // ending speed AND the next corner speed
          lower_bound_speed = static_cast<int>(max_corner_speed * corner_speed_min);
          upper_bound_speed = static_cast<int>(max_corner_speed * corner_speed_max);
          speed_dist = std::uniform_int_distribution<int>(lower_bound_speed, upper_bound_speed);
          logger("Created corner speed distribution with lower bound " + std::to_string(lower_bound_speed) +
                 " m/s and upper bound " + std::to_string(upper_bound_speed) + " m/s");

          // Loop until we find a valid corner speed
          int count2 = 0;
          bool valid_corner_speed = false;
          while (!valid_corner_speed) {
            count2++;
            if (count2 == max_iters) {
              return RacePlan("Maximum iterations reached");
            }

            proposed_corner_speed = speed_dist(speed_rng);
            logger("Trying corner speed " + std::to_string(proposed_corner_speed) + "m/s");

            // Select a location to start decelerating
            if (corner_start - 1 < acceleration_end_idx) {
              // Crossover from one loop to the next
              idx_dist = std::uniform_int_distribution<size_t>(acceleration_end_idx + 1,
                                                               this->num_points + corner_start - 1);
              logger("Created deceleration index distribution with lower bound " +
                    std::to_string(acceleration_end_idx + 1) +
                    " and upper bound " + std::to_string(this->num_points + corner_start) +
                    ". This is crossover from one loop to the next");
            } else {
              idx_dist = std::uniform_int_distribution<size_t>(acceleration_end_idx + 1, corner_start - 1);
              logger("Created deceleration index distribution with lower bound " +
                    std::to_string(acceleration_end_idx + 1) +
                    " and upper bound " + std::to_string(corner_start-1));
            }
            deceleration_start_idx = idx_dist(idx_rng);
            if (deceleration_start_idx >= this->num_points) {
              deceleration_start_idx -= this->num_points;
            }

            logger("Trying deceleration starting index of " + std::to_string(deceleration_start_idx));
            const double distance_to_corner = route_distances.get_value(deceleration_start_idx, corner_start);
            logger("Deceleration distance is " + std::to_string(distance_to_corner) + "m");
            proposed_deceleration = calc_acceleration(acceleration_end_speed,
                                                      proposed_corner_speed,
                                                      distance_to_corner);
            logger("Proposed deceleration is " + std::to_string(proposed_deceleration) + "m/s^2");
            // Deceleration could be positive if the selected corner speed is greater than
            // the acceleration ending speed.
            if (proposed_deceleration > 0.0 && proposed_deceleration < max_acceleration) {
              valid_corner_speed = true;
            } else if (proposed_deceleration < 0.0 && proposed_deceleration > max_deceleration) {
              valid_corner_speed = true;
            } else {
              logger("Necessary deceleration to reach next corner exceeds maximum acceleration/deceleration or exceeds "
                     "maximum acceleration power allowance");
            }

            // Check that the selected corner speed allows the car to reach the next corner's range of speeds in half
            // the straight distance
            distance_to_next_corner = route_distances.get_value(corner_end, next_corner_start) / 2.0;
            const std::pair<double, double> next_corner_range = {next_corner_max_speed * corner_speed_min,
                                                                 next_corner_max_speed * corner_speed_max};
            if (!does_acceleration_exist(proposed_corner_speed, distance_to_next_corner, next_corner_range,
                                         max_deceleration, max_acceleration)) {
              logger("Corner speed is too high to reach the next corner");
              valid_corner_speed = false;
              continue;
            }
          }

          logger("Aggressive straight with valid parameters were found");

          // Add acceleration segment
          segment = {last_real_corner_end, acceleration_end_idx};
          segment_distance = route_distances.get_value(last_real_corner_end, acceleration_end_idx);
          acceleration_value = proposed_acceleration;
          segment_speed = {last_real_corner_speed, acceleration_end_speed};
          acceleration = true;
          add_segment();

          logger("\nACCELERATION SEGMENT");
          logger(get_segment_string());

          // Add constant speed segment
          segment = {acceleration_end_idx, deceleration_start_idx};
          segment_distance = route_distances.get_value(acceleration_end_idx, deceleration_start_idx);
          acceleration_value = 0.0;
          acceleration = false;
          segment_speed = {acceleration_end_speed, acceleration_end_speed};
          add_segment();

          logger("\nCONSTANT SPEED SEGMENT");
          logger(get_segment_string());

          // Add deceleration speed segment
          segment = {deceleration_start_idx, corner_start};
          segment_distance = route_distances.get_value(deceleration_start_idx, corner_start);
          acceleration_value = proposed_deceleration;
          acceleration = acceleration_value != 0.0;
          segment_speed = {acceleration_end_speed, proposed_corner_speed};
          add_segment();

          logger("\nDECELERATION SPEED SEGMENT");
          logger(get_segment_string());

          // Add the cornering segment
          segment = {corner_start, corner_end};
          segment_distance = route_distances.get_value(corner_start, corner_end);
          acceleration_value = 0.0;
          acceleration = acceleration_value != 0.0;
          segment_speed = {proposed_corner_speed, proposed_corner_speed};
          add_segment();

          logger("\nCORNER SPEED SEGMENT");
          logger(get_segment_string());

          last_real_corner_speed = proposed_corner_speed;
          last_real_corner_idx = corner_idx;

          valid = true;
        }
      } else {
        normal_straight = true;
      }

      // This if statement looks weird because I'm imagining that in the future, we'll fallback to a normal_straight
      // if an aggressive_straight doesn't work out
      if (normal_straight) {
        if (is_first_segment) {
          logger("Taking straight between starting line and " +
          std::to_string(corner_idx) + " normally. Straight distance is " + std::to_string(straight_distance)
          + "m");
        } else {
          logger("Taking straight between corner " + std::to_string(last_real_corner_idx) + " and " +
          std::to_string(corner_idx) + " normally. Straight distance is " + std::to_string(straight_distance)
          + "m");
        }

        // Again, this should probably be a distribution around a mean value that takes into account the
        // previous corner speed and the next corner's maximum speed
        lower_bound_speed = std::max<int>(1, static_cast<int>(max_corner_speed * corner_speed_min));
        upper_bound_speed = static_cast<int>(max_corner_speed * corner_speed_max);
        speed_dist = std::uniform_int_distribution<int>(lower_bound_speed, upper_bound_speed);
        logger("Created corner speed distribution with lower bound " + std::to_string(lower_bound_speed) +
               "m/s and upper bound " + std::to_string(upper_bound_speed) + "m/s");

        valid = false;
        count = 0;
        while (!valid) {
          // Pick a corner speed
          proposed_corner_speed = speed_dist(speed_rng);
          distance_to_next_corner = route_distances.get_value(corner_end, next_corner_start);
          const std::pair<double, double> next_corner_speed_range = {next_corner_max_speed * corner_speed_min,
                                                                    next_corner_max_speed * corner_speed_max};
          if (proposed_corner_speed > last_real_corner_speed) {
            // Need to accelerate to the current corner
            // Parameters to search for
            double acceleration_distance;
            double acceleration_ending_speed;
            size_t acceleration_end_idx;

            logger("Selected corner speed is " + std::to_string(proposed_corner_speed) +
                   " m/s which is greater than the last corner speed of " +
                   std::to_string(last_real_corner_speed) + "m/s");

            // First check if this corner speed is possible to reach with the preferred acceleration
            double max_ending_speed = calc_final_speed_a(last_real_corner_speed, preferred_acceleration,
                                                         max_acceleration_distance);
            if (max_ending_speed < proposed_corner_speed) {
              logger("The corner speed is too high to be reached within the maximum acceleration distance of " +
                     std::to_string(max_acceleration_distance) + "m");
              continue;
            }

            // Use preferred acceleration
            acceleration_distance = calc_distance_a(last_real_corner_speed,
                                                    proposed_corner_speed,
                                                    preferred_acceleration);
            acceleration_ending_speed = calc_final_speed_a(last_real_corner_speed,
                                                           preferred_acceleration,
                                                           max_acceleration_distance);
            instataneous_motor_power = car_mass * preferred_acceleration * proposed_corner_speed;
            if (instataneous_motor_power >= acceleration_power_allowance) {
              logger("Instataneous motor power " + std::to_string(instataneous_motor_power) +
                     " W exceeds maximum acceleration power budget");
              continue;
            }

            // See if the car can reach the speed range of the next corner
            if (!does_acceleration_exist(proposed_corner_speed, distance_to_next_corner,
                                        next_corner_speed_range,
                                        max_deceleration, max_acceleration)) {
              logger("This corner speed does not allow the car to reach the next corner's range of speeds");
              continue;
            }

            // Find the route index such that [last_real_corner_end, route index] is greater than the acceleration
            // distance
            acceleration_end_idx = last_real_corner_end;
            while (route_distances.get_value(last_real_corner_end, acceleration_end_idx) < acceleration_distance) {
              acceleration_end_idx = acceleration_end_idx == this->num_points - 1 ? 0 : acceleration_end_idx + 1;
            }

            logger("Proposed corner speed is valid");

            // Add the acceleration segment
            segment = {last_real_corner_end, acceleration_end_idx};
            segment_speed = {last_real_corner_speed, proposed_corner_speed};
            segment_distance = route_distances.get_value(last_real_corner_end, acceleration_end_idx);
            acceleration = true;
            acceleration_value = preferred_acceleration;
            add_segment();

            logger("\nACCELERATION SEGMENT");
            logger(get_segment_string());

            // Add the corner segment
            segment_distance = route_distances.get_value(acceleration_end_idx, corner_end);
            segment = {acceleration_end_idx, corner_end};
            segment_speed = {proposed_corner_speed, proposed_corner_speed};
            acceleration_value = 0.0;
            acceleration = false;
            add_segment();

            logger("\nCORNER SEGMENT");
            logger(get_segment_string());

            last_real_corner_idx = corner_idx;
            last_real_corner_speed = proposed_corner_speed;
            valid = true;
          } else {
            // Decelerate to corner speed
            // Parameters to search for
            size_t deceleration_end_idx;
            size_t deceleration_distance;

            logger("Selected corner speed is " + std::to_string(proposed_corner_speed) + "m/s which is less than "
                   "the last corner speed of " + std::to_string(last_real_corner_speed) + "m/s");
            // Chosen corner speed is less than the previous corner speed, need to decelerate
            // First check if this corner speed is possible to reach with the preferred deceleration
            double max_ending_speed = 0.0;
            try {
              max_ending_speed = calc_final_speed_a(last_real_corner_speed, preferred_deceleration,
                                                    max_acceleration_distance);
            } catch (const InvalidCalculation& e) {
              // If the discriminant was negative, then the speed goes to 0 before the distance
              // is achieved. This means that the deceleration can definitely be achieved
              max_ending_speed = 0.0;
            }
            if (max_ending_speed > proposed_corner_speed) {
              logger("Preferred deceleration cannot achieve the desired corner speed. Trying again");
              continue;
            }

            // // See if the car can reach the speed range of the next corner
            // if (proposed_corner_speed > next_corner_max_speed * corner_speed_max) {
            //   const double ending_speed = calc_final_speed_a(proposed_corner_speed,
            //                                                  preferred_deceleration,
            //                                                  max_acceleration_distance);
            // }
            if (!does_acceleration_exist(proposed_corner_speed, distance_to_next_corner,
                                        next_corner_speed_range,
                                        max_deceleration, max_acceleration)) {
              logger("The corner speed is too low to be reached within the maximum acceleration distance of " +
                     std::to_string(max_acceleration_distance) + "m");
              continue;
            }

            deceleration_end_idx = last_real_corner_end;
            deceleration_distance = calc_distance_a(last_real_corner_speed,
                                                    proposed_corner_speed,
                                                    preferred_deceleration);
            while (route_distances.get_value(last_real_corner_end, deceleration_end_idx) < deceleration_distance) {
              deceleration_end_idx++;
            }

            logger("Proposed corner speed is valid");
            // Add the deceleration
            segment = {last_real_corner_end, deceleration_end_idx};
            segment_speed = {last_real_corner_speed, proposed_corner_speed};
            acceleration_value = preferred_deceleration;
            acceleration = true;
            segment_distance = route_distances.get_value(last_real_corner_end, deceleration_end_idx);
            add_segment();

            logger("\nDECELERATION SEGMENT");
            logger(get_segment_string());

            // Add the corner segment
            segment = {deceleration_end_idx, corner_end};
            segment_speed = {proposed_corner_speed, proposed_corner_speed};
            acceleration_value = 0.0;
            acceleration = false;
            segment_distance = route_distances.get_value(deceleration_end_idx, corner_end);
            add_segment();

            logger("\nCORNER SEGMENT");
            logger(get_segment_string());

            valid = true;
            last_real_corner_idx = corner_idx;
            last_real_corner_speed = proposed_corner_speed;
          }
        }
      }
      is_first_segment = false;
    }

    // Store results for this loop
    all_segments.push_back(loop_segments);
    all_segment_speeds.push_back(loop_segment_speeds);
    all_acceleration_segments.push_back(loop_acceleration_segments);
    all_acceleration_values.push_back(loop_acceleration_values);
    all_segment_distances.push_back(loop_segment_distances);
  }

  return RacePlan(all_segments, all_segment_speeds, all_acceleration_segments,
                  all_acceleration_values, all_segment_distances);
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
    double max_corner_speed = std::stod(max_speed);
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
  const size_t num_loops = segments.size();

  auto print_spaces = [](size_t num_spaces) {
    for (size_t i = 0; i < num_spaces; i++) {
      std::cout << " ";
    }
    std::cout << std::flush;
  };

  const bool print_distances = distances.size() > 0;
  auto print_header = [&]() {
    if (print_distances) {
      std::cout << "+-----------+---------------+-----------------------+----------------" << std::endl;
      std::cout << "; Segments  ; Speeds (m/s)  ; Acceleration (m/s^2)  ; Distances (m) ;" << std::endl;
      std::cout << "+-----------+---------------+-----------------------+----------------" << std::endl;
    } else {
      std::cout << "+-----------+---------------+-----------------------+" << std::endl;
      std::cout << "; Segments  ; Speeds (m/s)  ; Acceleration (m/s^2)  ;" << std::endl;
      std::cout << "+-----------+---------------+-----------------------+" << std::endl;
    }
  };

  for (size_t loop_idx = 0; loop_idx < num_loops; loop_idx++) {
    std::cout << "\nLOOP " << loop_idx << std::endl;
    print_header();

    const size_t num_segments = segments[loop_idx].size();
    for (size_t seg_idx = 0; seg_idx < num_segments; seg_idx++) {
      std::cout << "| [" << std::setw(3) << segments[loop_idx][seg_idx].first << ","
                << std::setw(3) << segments[loop_idx][seg_idx].second << "] |";
      print_spaces(6);
      std::cout << "[" << std::setw(3) << segment_speeds[loop_idx][seg_idx].first << ","
                << std::setw(3) << segment_speeds[loop_idx][seg_idx].second << "] |";
      print_spaces(18);
      // Truncate the acceleration
      std::ostringstream oss;
      oss << acceleration[loop_idx][seg_idx];
      std::string num_str = oss.str();
      if (num_str.size() > 5) {
        num_str = num_str.substr(0, 5);
      }
      std::cout << std::setw(5) << num_str;
      std::cout << "|";

      if (print_distances) {
        // Truncate the distance
        print_spaces(9);
        std::ostringstream dss;
        dss << distances[loop_idx][seg_idx];
        std::string dist_str = dss.str();
        if (dist_str.size() > 5) {
          dist_str = dist_str.substr(0, 5);
        }
        std::cout << std::setw(5) << dist_str;
        std::cout << "|";
      }
      std::cout << std::endl;
    }
    if (print_distances) {
      std::cout << "---------------------------------------------------------------------" << std::endl;
    } else {
      std::cout << "-----------------------------------------------------" << std::endl;
    }
  }
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
                   std::vector<std::vector<double>> acceleration,
                   std::vector<std::vector<double>> distances)
                  : segments(segments), segment_speeds(segment_speeds), acceleration_segments(acceleration_segments),
                  acceleration(acceleration), distances(distances) {
  if (acceleration_segments.size() == 0 || acceleration.size() == 0) {
    this->acceleration_segments.resize(segments.size());
    this->acceleration.resize(segments.size());
    this->distances.resize(segments.size());
    for (size_t i=0; i < segments.size(); i++) {
      this->acceleration_segments[i] = std::vector<bool>(segments[i].size(), false);
      this->acceleration[i] = std::vector<double>(segments.size(), 0.0);
      this->distances[i] = std::vector<double>(segments.size(), 0.0);
    }
  }

  if (this->segments.size() == 0 && this->segment_speeds.size() == 0 &&
      this->acceleration_segments.size() == 0 && this->acceleration.size() == 0) {
    empty = true;
  } else {
    empty = false;
  }
}
