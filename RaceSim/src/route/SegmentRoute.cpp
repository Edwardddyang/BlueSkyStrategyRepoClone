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
#include <unordered_set>

#include "utils/Defines.hpp"
#include "route/Route.hpp"
#include "config/Config.hpp"
#include "utils/Utilities.hpp"
#include "utils/Logger.hpp"
#include "utils/Geography.hpp"
#include "utils/CustomException.hpp"

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

  const double segment_distance = calc_segment_distance(coords, starting_idx, ending_idx);

  return calc_acceleration(init_speed, ending_speed, segment_distance);
}

template <typename T>
// Note that gen needs to be a reference since the internal state needs to be advanced after calling
static T bounded_gaussian(std::normal_distribution<> dist, std::mt19937& gen,  // NOLINT
                          double lower_bound, double upper_bound, const FileLogger& logger) {  // NOLINT
  double value;
  do {
    value = dist(gen);
  } while (value < lower_bound || value > upper_bound);

  return static_cast<T>(value);
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

bool Route::can_reach_speeds(double initial_speed, double acceleration_power, double preferred_acceleration,
                            double preferred_deceleration, std::pair<double, double> speed_range,
                            double max_distance, double car_mass) {
  RUNTIME_EXCEPTION(speed_range.first <= speed_range.second, "Speed range must be ordered as {smaller, bigger}");
  RUNTIME_EXCEPTION(initial_speed >= 0.0, "Initial speed must be >= 0 m/s");
  RUNTIME_EXCEPTION(max_distance >= 0.0, "Distance must be >= 0.0");

  if (initial_speed >= speed_range.first && initial_speed <= speed_range.second) {
    return true;
  }

  if (initial_speed <= speed_range.first) {
    double distance = 0.0;
    double final_speed;

    do {
      final_speed = calc_final_speed_a(initial_speed, preferred_acceleration, distance);
      if (final_speed > speed_range.first) return true;
      distance += 0.1;
    } while (final_speed * car_mass * preferred_acceleration < acceleration_power && distance < max_distance);

    return false;
  }

  if (initial_speed > speed_range.second) {
    double distance = 0.0;
    double final_speed;

    do {
      final_speed = calc_final_speed_a(initial_speed, preferred_deceleration, distance);
      if (final_speed < speed_range.second) return true;
      distance += 0.1;
    } while (final_speed * car_mass * preferred_acceleration < acceleration_power && distance < max_distance);

    return false;
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
                                      const double min_acceleration,
                                      const double average_speed,
                                      const Time* start_time,
                                      const Time* end_time,
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
  RUNTIME_EXCEPTION(cornering_segment_bounds.size() > 1, "There must exist at least 2 corners");
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
  RUNTIME_EXCEPTION(num_repetitions >= 1, "Num repetitions per loop block must >= 1");
  RUNTIME_EXCEPTION(preferred_deceleration >= max_deceleration,
                    "Preferred deceleration must have lower magnitude than maximum deceleration");
  RUNTIME_EXCEPTION(preferred_acceleration <= max_acceleration,
                    "Preferred acceleration must have lower magnitude than maximum acceleration");
  RUNTIME_EXCEPTION(start_time != nullptr && end_time != nullptr, "Start time or End time is null");
  RUNTIME_EXCEPTION(*start_time < *end_time, "Start time must be before the end time");
  RUNTIME_EXCEPTION(min_acceleration > 0.0, "Minimum acceleration must be greater than 0");

  // Create random number generators
  std::mt19937 speed_rng(speed_seed);
  std::mt19937 loop_rng(loop_seed);
  std::mt19937 aggressive_rng(aggressive_seed);
  std::mt19937 idx_rng(idx_seed);
  std::mt19937 acceleration_rng(acceleration_seed);

  // Route attributes
  const size_t num_corners = cornering_segment_bounds.size();

  // Number of corners for which we need to create segments for
  const size_t num_corners_to_create = num_repetitions > 1 ? num_corners + 1 : num_corners;

  // RacePlan attributes
  std::vector<std::vector<std::pair<size_t, size_t>>> all_segments;
  std::vector<std::vector<std::pair<double, double>>> all_segment_speeds;
  std::vector<std::vector<bool>> all_acceleration_segments;
  std::vector<std::vector<double>> all_acceleration_values;
  std::vector<std::vector<double>> all_segment_distances;

  // Randomly select the number of loops to complete
  std::uniform_int_distribution<unsigned int> dis(1, max_num_loops);
  const size_t num_loops = static_cast<size_t>(dis(loop_rng));

  // Number of blocks to create
  size_t num_blocks;
  if (num_loops % num_repetitions == 0) {
    num_blocks = num_loops / num_repetitions;
  } else if (num_loops < num_repetitions) {
    num_blocks = 1;
  } else {
    num_blocks = num_loops / num_repetitions + 1;
  }

  const double acceleration_power_allowance = max_motor_power * acceleration_power_budget;
  Time curr_time = *start_time;

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
  logger("Minimum acceleration to consider: " + std::to_string(min_acceleration) + "m/s^2");
  logger("Preferred acceleration: " + std::to_string(preferred_acceleration) + " m/s^2");
  logger("Preferred deceleration: " + std::to_string(preferred_deceleration) + " m/s^2");
  logger("Corner speed minimum clamp: " + std::to_string(corner_speed_min));
  logger("Corner speed maximum clamp: " + std::to_string(corner_speed_max));
  logger("Aggressive straight threshold: " + std::to_string(aggressive_straight_threshold) + " m");
  logger("Number of loop repetitions per chunk: " + std::to_string(num_repetitions));
  logger("Maximum iteration count for sampling: " + std::to_string(max_iters));
  logger("Fractional acceleration power budget: " + std::to_string(acceleration_power_budget));
  logger("Randomly selected number of loops to complete: " + std::to_string(num_loops));
  logger("Acceleration power allowance is " + std::to_string(acceleration_power_allowance) + " W\n");
  logger("Race Plan start time is " + curr_time.get_local_readable_time());
  logger("Race Plan end time is " + end_time->get_local_readable_time());

  ///////////////////////////////////////////////////////////////////////////////////
  ////////////////////// NOTE ABOUT TERMINOLOGY USED ///////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////
  // "route index": Denotes an index in the route_points_array
  // "corner index": Denotes an index in the cornering_segment_bouds array
  // "real corner": A corner where the maximum cornering speed is less than the route's speed limit
  // "segment index": An index inside loop_segments, loop_segment_speeds ...
  // "wrap-around corner": The first corner of the route which the car wraps to after taking the entire route

  ///////////////////////////////////////////////////////////////////////////////////
  ///////////////////////// State variables for each corner /////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////
  // First corner of the first loop
  bool is_first_segment = true;

  // Whether a straight is taken aggressively
  bool aggressive_straight = false;

  // Whether a straight is taking normally
  bool normal_straight = false;

  // Attributes of the next and last corner
  double prev_corner_max_speed = 0.0;  // Maximum speed of the previous corner
  double next_corner_max_speed = 0.0;  // Maximum speed of the next speed
  size_t next_corner_start = 0;        // Starting route index of the next corner
  size_t prev_corner_end = 0;          // Ending route index of the previous corner

  // Stack of real corner speeds where the top of the stack is the most recent one
  std::stack<double> real_corner_speeds;
  real_corner_speeds.push(0.0);

  // Stack of real corner indices where the top of the stack is the most recent one
  std::stack<size_t> real_corner_indices;
  real_corner_indices.push(0);

  // Stack of # segments added for each real corner
  std::stack<uint64_t> num_segments_added;
  num_segments_added.push(0);
  uint64_t segment_counter = 0;

  // Stack of first corner speeds - pushed after constructing each new loop
  std::stack<uint64_t> first_corner_speeds;
  first_corner_speeds.push(0);

  // Stack holding the number of segments added to the last loop of the previous loop block
  // as a result of carrying over the wrap-around segments.
  std::stack<uint64_t> num_added_wrap_around_segments;
  num_added_wrap_around_segments.push(0);

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

  // Counter for the number of blocks
  size_t block_idx = 0;

  // Estimated instataneous motor power in W
  double instataneous_motor_power = 0.0;

  // Upper and lower bounds for speed
  int lower_bound_speed = 0;
  int upper_bound_speed = 0;

  // Mean and standard deviation to parameterize the normal distribution
  double speed_mean = average_speed;
  double speed_dev = 5.0;

  // Keep track of speeds that have been tested when sampling
  std::unordered_set<int> sampled_speeds;

  /** @brief Lambda to check if all speeds in the range of [lower_bound_speed, upper_bound_speed] have been tested */
  auto all_speeds_sampled = [&]() -> bool {
    int speed = lower_bound_speed;
    while (speed <= upper_bound_speed) {
      if (sampled_speeds.find(speed) == sampled_speeds.end()) {
        return false;
      }
      speed += 1;
    }
    return true;
  };

  ///////////////////////////////////////////////////////////////////////////////////
  // Temp data holders for each segment
  std::pair<size_t, size_t> segment = {0, 0};
  std::pair<double, double> segment_speed = {0.0, 0.0};
  bool acceleration = false;
  double acceleration_value = 0.0;
  double segment_distance = 0.0;

  // Temp data holders for each loop
  std::vector<std::pair<size_t, size_t>> loop_segments;
  std::vector<std::pair<double, double>> loop_segment_speeds;
  std::vector<bool> loop_acceleration_segments;
  std::vector<double> loop_acceleration_values;
  std::vector<double> loop_segment_distances;

  // When creating loop blocks, we will be modifying the loop vectors. So when rolling back,
  // we need to restore the originally created loop holder
  std::stack<std::vector<std::pair<size_t, size_t>>> loop_segments_stack;
  std::stack<std::vector<std::pair<double, double>>> loop_speeds_stack;
  std::stack<std::vector<bool>> loop_acceleration_stack;
  std::stack<std::vector<double>> loop_acceleration_values_stack;
  std::stack<std::vector<double>> loop_distances_stack;

  auto get_loop_readable = [&]() -> std::string {
    return RacePlan::get_loop_string(loop_segments, loop_segment_speeds,
                                     loop_acceleration_values, loop_segment_distances);
  };

  // Probability distributions used to randomly select locations, speeds
  // and accelerations. Note that they are inclusive on both sides
  // TODO(Ethan): These should probably not be uniform distributions but at the very least,
  // gaussian
  std::uniform_int_distribution<size_t> idx_dist;
  std::normal_distribution<double> speed_dist;
  std::uniform_real_distribution<double> acceleration_dist(0.1, max_acceleration);
  std::uniform_real_distribution<double> aggressive_dist(0.0, 1.0);

  size_t loop_idx = 0;
  while (block_idx < num_blocks) {
    /** @brief Lambda to add a new segment to the loop
    */
    auto add_segment = [&]() {
      loop_segments.emplace_back(segment);
      loop_segment_speeds.emplace_back(segment_speed);
      loop_acceleration_segments.emplace_back(acceleration);
      loop_acceleration_values.emplace_back(acceleration_value);
      loop_segment_distances.emplace_back(segment_distance);
      segment_counter = segment_counter + 1;
    };

    /** @brief Lambda to add the latest loop to the overall race plan 
     * Note: Will be added as vec[start_idx:end_idx] for each loop property
    */
    auto add_loop = [&](const size_t start_idx, const size_t end_idx) {
      RUNTIME_EXCEPTION(start_idx <= end_idx, "End index must be greater than Start index");
      RUNTIME_EXCEPTION(end_idx <= loop_segments.size(), "Ending index must be less than or equal to the "
                                                         "loop segments vector");
      // Store results for this loop
      std::vector<std::pair<size_t, size_t>> segments_slice(loop_segments.begin() + start_idx,
                                                            loop_segments.begin() + end_idx);
      all_segments.push_back(segments_slice);

      std::vector<std::pair<double, double>> segment_speeds_slice(loop_segment_speeds.begin() + start_idx,
                                                                  loop_segment_speeds.begin() + end_idx);
      all_segment_speeds.push_back(segment_speeds_slice);

      std::vector<bool> loop_acceleration_slice(loop_acceleration_segments.begin() + start_idx,
                                                loop_acceleration_segments.begin() + end_idx);
      all_acceleration_segments.push_back(loop_acceleration_slice);

      std::vector<double> loop_acceleration_v_slice(loop_acceleration_values.begin() + start_idx,
                                                    loop_acceleration_values.begin() + end_idx);
      all_acceleration_values.push_back(loop_acceleration_v_slice);

      std::vector<double> loop_segment_distances_slice(loop_segment_distances.begin() + start_idx,
                                                       loop_segment_distances.begin() + end_idx);
      all_segment_distances.push_back(loop_segment_distances_slice);
    };

    /** @brief Create loop block with the newly created loop */
    auto create_loop_block = [&]() -> size_t {
      const size_t num_loops_in_block = std::min(static_cast<size_t>(num_repetitions), num_loops - loop_idx);

      logger("Creating loop block " + std::to_string(block_idx) + " with " +
              std::to_string(num_loops_in_block) + " loops");

      logger("\n////CREATED LOOP////");
      logger(get_loop_readable());

      // --------------- Create the first loop of the block ---------------
      logger("Creating loop 1");
      // For loops in a block greater than the first one, they will begin with segments that travel from
      // the last corner of the route to the first corner. Add these segments to the last loop of the previous block.
      // For example, the beginning of a typical loop past the first block looks something like this:
      // +-----------+---------------+-----------------------+----------------
      // ; Segments  ; Speeds (m/s)  ; Acceleration (m/s^2)  ; Distances (m) ;
      // +-----------+---------------+-----------------------+----------------
      // | [573,585] |     [  6, 12] |                  0.500|          117.4|
      // | [585,591] |     [ 12, 12] |                      0|          56.05|
      // | [591, 19] |     [ 12,  7] |                  -0.10|          469.7|
      // | [ 19, 54] |     [  7,  7] |                      0|          261.1|
      // | [ 54, 58] |     [  7,  4] |                     -1|          25.69|
      // FIGURE A

      // Recall that for each corner, we create the segments coming from the last corner to the current
      // corner. In this case, we want to place [573,585],[585,591] at the end of the last loop in the
      // previous block. For crossover segments e.g. [591, 19], we break them up into two sub-segments.
      // The first will travel from the segment beginning to index 0 and the second will travel from index 0
      // to the end of the original segment

      // For the very first block, these segments will not exist, since we're starting at index 0
      num_added_wrap_around_segments.push(0);
      while (block_idx != 0 && loop_segments[0].second != 0 && loop_segments[0].first < loop_segments[0].second) {
        all_segments.back().push_back(loop_segments[0]);
        all_segment_speeds.back().push_back(loop_segment_speeds[0]);
        all_acceleration_segments.back().push_back(loop_acceleration_segments[0]);
        all_acceleration_values.back().push_back(loop_acceleration_values[0]);
        all_segment_distances.back().push_back(loop_segment_distances[0]);
        num_added_wrap_around_segments.top() = num_added_wrap_around_segments.top() + 1;

        logger("Moving segment [" + std::to_string(loop_segments[0].first) + "," +
              std::to_string(loop_segments[0].second) + "] to the last loop");

        // Remove the first element of loop vectors - this is why we don't have to keep track of an index variable
        loop_segments.erase(loop_segments.begin());
        loop_segment_speeds.erase(loop_segment_speeds.begin());
        loop_acceleration_segments.erase(loop_acceleration_segments.begin());
        loop_acceleration_values.erase(loop_acceleration_values.begin());
        loop_segment_distances.erase(loop_segment_distances.begin());
      }

      // Crossover segment detected. Break into two sub-segments
      if (loop_segments[0].first > loop_segments[0].second) {
        std::pair<size_t, size_t> first_segment = {loop_segments[0].first, 0};
        const double first_segment_distance = route_distances.get_value(first_segment.first, 0);
        const double first_segment_ending_speed = calc_final_speed_a(loop_segment_speeds[0].first,
                                                                     loop_acceleration_values[0],
                                                                     first_segment_distance);
        std::pair<double, double> first_segment_speeds = {loop_segment_speeds[0].first,
                                                          first_segment_ending_speed};

        all_segments.back().push_back(first_segment);
        all_segment_speeds.back().push_back(first_segment_speeds);
        all_acceleration_segments.back().push_back(loop_acceleration_segments[0]);
        all_acceleration_values.back().push_back(loop_acceleration_values[0]);
        all_segment_distances.back().push_back(first_segment_distance);

        std::pair<size_t, size_t> second_segment = {0, loop_segments[0].second};
        const double second_segment_distance = route_distances.get_value(0, second_segment.second);
        const double second_segment_ending_speed = loop_segment_speeds[0].second;
        std::pair<double, double> second_segment_speeds = {first_segment_ending_speed,
                                                          second_segment_ending_speed};
        loop_segments[0] = second_segment;
        loop_segment_speeds[0] = second_segment_speeds;
        loop_segment_distances[0] = second_segment_distance;
      }

      // For num_repetitions > 1, there will be a wrap-around corner. Therefore, the end of the loop
      // will look something like this:
      // ...
      // | [560,573] |     [  6,  6] |                      0|          112.9|
      // | [573,574] |     [  6,  7] |                  1.552|          8.895|
      // | [574,615] |     [  7,  7] |                      0|          393.5|
      // | [615, 19] |     [  7,  2] |                  -0.09|          240.7|
      // | [ 19, 54] |     [  2,  2] |                      0|          261.1|
      // FIGURE B

      // We want to cut off at index 0, meaning we remove the [19,54] segment and break the crossover segment
      // into two sub-segments as in the above
      // Record the wrap around segments which needed to be added to the beginning of the loop later
      std::vector<std::pair<size_t, size_t>> wrap_around_segments;
      std::vector<std::pair<double, double>> wrap_around_speeds;
      std::vector<bool> wrap_around_acceleration;
      std::vector<double> wrap_around_acceleration_values;
      std::vector<double> wrap_around_distances;
      while (num_repetitions > 1 && loop_segments.back().second > loop_segments.back().first &&
            loop_segments.back().second != 0) {
        wrap_around_segments.insert(wrap_around_segments.begin(), loop_segments.back());
        wrap_around_speeds.insert(wrap_around_speeds.begin(), loop_segment_speeds.back());
        wrap_around_acceleration.insert(wrap_around_acceleration.begin(),
                                        loop_acceleration_segments.back());
        wrap_around_acceleration_values.insert(wrap_around_acceleration_values.begin(),
                                               loop_acceleration_values.back());
        wrap_around_distances.insert(wrap_around_distances.begin(),
                                     loop_segment_distances.back());

        loop_segments.pop_back();
        loop_segment_speeds.pop_back();
        loop_acceleration_segments.pop_back();
        loop_acceleration_values.pop_back();
        loop_segment_distances.pop_back();
      }

      // Crossover segment detected on last segment. We will only insert the first sub-segment into
      // the current loop, but we also need to preserve the second segment when we create the loops > 1st one
      // of the block
      if (loop_segments.back().second < loop_segments.back().first) {
        std::pair<size_t, size_t> first_segment = {loop_segments.back().first, 0};
        const double first_segment_distance = route_distances.get_value(first_segment.first, 0);
        const double first_segment_ending_speed = calc_final_speed_a(loop_segment_speeds.back().first,
                                                                     loop_acceleration_values.back(),
                                                                     first_segment_distance);
        std::pair<double, double> first_segment_speeds = {loop_segment_speeds.back().first,
                                                          first_segment_ending_speed};
        const std::pair<size_t, size_t> second_segment = {0, loop_segments.back().second};
        const double second_segment_distance = route_distances.get_value(0, second_segment.second);
        const double second_segment_ending_speed = loop_segment_speeds.back().second;
        const std::pair<double, double> second_segment_speeds = {first_segment_ending_speed,
                                                                  second_segment_ending_speed};

        wrap_around_segments.insert(wrap_around_segments.begin(), second_segment);
        wrap_around_speeds.insert(wrap_around_speeds.begin(), second_segment_speeds);
        wrap_around_acceleration.insert(wrap_around_acceleration.begin(), loop_acceleration_segments.back());
        wrap_around_acceleration_values.insert(wrap_around_acceleration_values.begin(),
                                               loop_acceleration_values.back());
        wrap_around_distances.insert(wrap_around_distances.begin(), second_segment_distance);

        loop_segments.back() = first_segment;
        loop_segment_speeds.back() = first_segment_speeds;
        loop_segment_distances.back() = first_segment_distance;
      }
      // Add the first loop
      if (num_loops_in_block > 1) {
        logger("Loop 1 Modified:");
        logger(get_loop_readable());
        add_loop(0, loop_segments.size());
      }
      // --------------------------------------------------------------------

      // --------------- Create the loops after the first one ---------------
      // This is only applicable if the number of loops in a block is greater than 1
      if (num_loops_in_block > 1) {
        logger("Modifying loop construct for the second... loop in the block");
        // Refer to figure A, we first want to delete the segments that extend up to and including
        // the first corner as they will be replaced by the wrap-around segment
        while (loop_segments[0].second <= cornering_segment_bounds[0].second) {
          loop_segments.erase(loop_segments.begin());
          loop_segment_speeds.erase(loop_segment_speeds.begin());
          loop_acceleration_segments.erase(loop_acceleration_segments.begin());
          loop_acceleration_values.erase(loop_acceleration_values.begin());
          loop_segment_distances.erase(loop_segment_distances.begin());
        }

        // Move the recorded wrap around segments to the front
        loop_segments.insert(loop_segments.begin(), wrap_around_segments.begin(), wrap_around_segments.end());
        loop_segment_speeds.insert(loop_segment_speeds.begin(), wrap_around_speeds.begin(), wrap_around_speeds.end());
        loop_acceleration_segments.insert(loop_acceleration_segments.begin(), wrap_around_acceleration.begin(),
                                          wrap_around_acceleration.end());
        loop_acceleration_values.insert(loop_acceleration_values.begin(), wrap_around_acceleration_values.begin(),
                                        wrap_around_acceleration_values.end());
        loop_segment_distances.insert(loop_segment_distances.begin(), wrap_around_distances.begin(),
                                      wrap_around_distances.end());

        for (size_t i=1; i < num_loops_in_block - 1; i++) {
          add_loop(0, loop_segments.size());
        }
        logger("LOOP X MODIFIED");
        logger(get_loop_readable());
      }

      // --------------------------------------------------------------------

      // --------------- Add the last loop of the block ---------------
      size_t last_corner_ending_idx = 0;
      for (; last_corner_ending_idx < loop_segments.size(); last_corner_ending_idx++) {
        if (loop_segments[last_corner_ending_idx].second == cornering_segment_bounds.back().second) {
          break;
        }
      }
      if (block_idx == num_blocks - 1) {
        // Remove all the segments past the last corner and create a constant segment until index 0
        logger("HERE");
        const size_t num_segments_to_remove = loop_segments.size() - last_corner_ending_idx;

        loop_segments.erase(loop_segments.end() - num_segments_to_remove, loop_segments.end());
        loop_segment_speeds.erase(loop_segment_speeds.end() - num_segments_to_remove,
                                  loop_segment_speeds.end());
        loop_acceleration_segments.erase(loop_acceleration_segments.end() - num_segments_to_remove,
                                          loop_acceleration_segments.end());
        loop_acceleration_values.erase(loop_acceleration_values.end() - num_segments_to_remove,
                                        loop_acceleration_values.end());
        loop_segment_distances.erase(loop_segment_distances.end() - num_segments_to_remove,
                                      loop_segment_distances.end());

        const size_t corner_end_idx = loop_segments.back().second;
        const int corner_speed = loop_segment_speeds.back().second;
        segment = {corner_end_idx, 0};
        segment_speed = {corner_speed, corner_speed};
        acceleration_value = 0.0;
        acceleration = false;
        segment_distance = route_distances.get_value(segment.first, segment.second);

        add_segment();
        add_loop(0, loop_segments.size());
      } else {
        logger("Last corner ending index: " + std::to_string(last_corner_ending_idx));
        logger("Block counter: " + std::to_string(block_idx));
        // If this is the last loop of the block, then we stop at the end of the last corner
        // since we want the next loop to decide how to travel from the last corner to the first
        add_loop(0, last_corner_ending_idx + 1);
      }
      return num_loops_in_block;
    };

    // Clear all temp data holders
    loop_segments.clear();
    loop_segment_speeds.clear();
    loop_acceleration_segments.clear();
    loop_acceleration_values.clear();
    loop_segment_distances.clear();
    loop_segments.clear();
    loop_segment_speeds.clear();
    loop_acceleration_segments.clear();
    loop_acceleration_values.clear();
    loop_segment_distances.clear();

    // Labmda to remove the latest corner segments added to the loop
    auto remove_last_corner_segments = [&]() {
      const size_t curr_loop_num_segments = loop_segments.size();
      const size_t num_segments_of_last_corner = static_cast<size_t>(num_segments_added.top());
      logger("Number of segments in current loop: " + std::to_string(curr_loop_num_segments));
      const size_t num_remove_from_curr_loop = std::min(curr_loop_num_segments, num_segments_of_last_corner);
      if (num_segments_of_last_corner == 0) {
        return;
      }
      logger("Removing " + std::to_string(num_segments_of_last_corner) + " segments");

      for (size_t idx = 0; idx < num_remove_from_curr_loop; idx++) {
        loop_segments.pop_back();
        loop_segment_speeds.pop_back();
        loop_acceleration_segments.pop_back();
        loop_acceleration_values.pop_back();
        loop_segment_distances.pop_back();
      }

      const size_t num_remove_from_last_loop = num_segments_of_last_corner > curr_loop_num_segments ?
                                               num_segments_of_last_corner - curr_loop_num_segments : 0;
      logger("Removed " + std::to_string(num_remove_from_curr_loop) + " segments from current loop and " +
             std::to_string(num_remove_from_last_loop) + " from last loop");
      if (all_segments.size() > 0 && num_remove_from_last_loop > 0) {
        RUNTIME_EXCEPTION(num_remove_from_last_loop <= loop_segments_stack.top().size(),
                          "Last loop has too few segments");
      }
      for (size_t idx = 0; idx < num_remove_from_last_loop; idx++) {
        loop_segments_stack.top().pop_back();
        loop_speeds_stack.top().pop_back();
        loop_acceleration_stack.top().pop_back();
        loop_acceleration_values_stack.top().pop_back();
        loop_distances_stack.top().pop_back();
      }
    };

    // Lambda to convert segment information into a string (debugging + logging purposes)
    auto get_segment_string = [&]() {
      std::stringstream ss;
      ss << "Segment: [" << segment.first << "," << segment.second << "]\n";
      ss << "Segment Distance: " << segment_distance << "m\n";
      ss << "Segment Speeds: [" << segment_speed.first << "," << segment_speed.second << "]\n";
      ss << "Acceleration: " << (acceleration ? "True\n" : "False\n");
      ss << "Acceleration Value: " << acceleration_value << "\n";
      return ss.str();
    };

    // We construct the segments between the previous corner to the current corner.
    // We consider a complete loop as one that wraps back around to the first corner
    // of the route.
    size_t corner_idx = 0;
    while (corner_idx < num_corners_to_create) {
      segment_counter = 0;
      auto create_segments = [&]() -> bool {
        // Wrap-around corner is index 0
        logger("--------------CREATING SEGMENTS FOR CORNER " + std::to_string(corner_idx) +
              " OF LOOP BLOCK " + std::to_string(block_idx), false);
        if (corner_idx == num_corners) {
          logger(" [WRAP-AROUND CORNER]--------------");
        } else {
          logger("--------------");
        }
        // Get attributes for current corner
        const size_t corner_start = cornering_segment_bounds[corner_idx % num_corners].first;
        const size_t corner_end = cornering_segment_bounds[corner_idx % num_corners].second;
        const double max_corner_speed = cornering_speed_bounds[corner_idx % num_corners];
        logger("Corner maximum speed is " + std::to_string(max_corner_speed) + "m/s");
        logger("Corner start is index " + std::to_string(corner_start));
        logger("Corner end is index " + std::to_string(corner_end));

        // If the maximum cornering speed is greater than the route speed, then we can treat this as a
        // no acceleration zone and move onto the next corner
        if (max_corner_speed > this->max_route_speed) {
          logger("This corner has a speed greater than the route speed. "
                "Skip segment generation and treat as no-acceleration zone\n");
          return true;
        }

        // Get attributes for previous corner
        if (corner_idx > 0) {  // Note this includes the wrap-around corner e.g. corner_idx = num_corners
          prev_corner_end = cornering_segment_bounds[corner_idx-1].second;
          prev_corner_max_speed = cornering_speed_bounds[corner_idx-1];
        } else {  // Last corner
          prev_corner_end = cornering_segment_bounds[num_corners-1].second;
          prev_corner_max_speed = cornering_speed_bounds[num_corners-1];
        }

        // Get attributes for next corner
        if (corner_idx < num_corners - 1) {
          next_corner_start = cornering_segment_bounds[corner_idx+1].first;
          next_corner_max_speed = cornering_speed_bounds[corner_idx+1];
        } else if (corner_idx == num_corners) {  // First corner
          next_corner_start = cornering_segment_bounds[1].first;
          next_corner_max_speed = cornering_speed_bounds[1];
        } else {
          next_corner_start = cornering_segment_bounds[0].first;
          next_corner_max_speed = cornering_speed_bounds[0];
        }

        // Get the length of the straight between the last real corner and the current corner
        const size_t last_real_corner_idx = real_corner_indices.top();
        const double last_real_corner_speed = real_corner_speeds.top();
        last_real_corner_end = is_first_segment ? 0 : cornering_segment_bounds[last_real_corner_idx].second;
        logger("Last real corner index was " + std::to_string(last_real_corner_idx));

        const double straight_distance = route_distances.get_value(last_real_corner_end, corner_start);
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
          // Spending a quarter the straight accelerating seems wasteful. Maybe change later
          max_acceleration_distance = straight_distance * 0.25;
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

          // Route index of the ending point for acceleration. This is selected
          // by sampling an ending speed for the acceleration
          size_t acceleration_end_idx;

          // Route index of the starting point for deceleration. This must be in the range
          // of [last_real_corner_end, corner_start-1]
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
              return false;
            }

            // Create a distribution of speeds that can sustain travel at 0.1 m/s^2 acceleration and
            // whose distance(current_speed, speed) <= max_acceleration_distance
            double upper_bound_speed = last_real_corner_speed - 0.1;
            double acceleration_distance = 0.0;
            do {
              upper_bound_speed += 0.1;
              acceleration_distance = calc_distance_a(last_real_corner_speed, upper_bound_speed, min_acceleration);
            } while (min_acceleration * car_mass * last_real_corner_speed < acceleration_power_allowance &&
                    acceleration_distance < max_acceleration_distance);

            if (static_cast<int>(upper_bound_speed) <= last_real_corner_speed + 1) {
              // If the upper bound speed is the same as the corner speed, then that means
              // the maximum acceleration distance is too low or the last corner speed was too high
              // for the motor
              normal_straight = true;
              break;
            }

            if (average_speed > upper_bound_speed) {
              speed_mean = (upper_bound_speed + last_real_corner_speed + 1) / 2.0;
              speed_dev = speed_mean / 6.0;
            } else {
              speed_mean = average_speed;
              speed_dev = average_speed / 6.0;
            }

            speed_dist = std::normal_distribution<double>(speed_mean, speed_dev);
            logger("Created acceleration ending speed gaussian distribution with lower bound " +
                   std::to_string(last_real_corner_speed + 1) + ", upper bound " +
                   std::to_string(static_cast<int>(upper_bound_speed)) + ", mean of " +
                   std::to_string(speed_mean) + ", deviation of " + std::to_string(speed_dev));
            acceleration_end_speed = bounded_gaussian<int>(speed_dist, speed_rng, last_real_corner_speed + 1,
                                                           upper_bound_speed, logger);
            proposed_acceleration = calc_acceleration(last_real_corner_speed, acceleration_end_speed,
                                                      acceleration_distance);
            logger("Trying acceleration ending speed of " + std::to_string(acceleration_end_speed) +
                   "m/s with required acceleration of " + std::to_string(proposed_acceleration) + "m/s^2");

            // Ensure that the acceleration power budget is not exceeded
            instataneous_motor_power = proposed_acceleration * car_mass * last_real_corner_speed;
            if (instataneous_motor_power > acceleration_power_allowance) {
              logger(std::to_string(instataneous_motor_power) + " W exceeds maximum acceleration power budget. "
                    "Trying again");
              continue;
            }

            // Locate the smallest route index such that [last_real_corner_end, idx] is greater
            // than the acceleration distance
            acceleration_end_idx = last_real_corner_end;
            while (route_distances.get_value(last_real_corner_end, acceleration_end_idx) < acceleration_distance) {
              acceleration_end_idx = acceleration_end_idx == this->num_points - 1 ? 0 : acceleration_end_idx + 1;
            }

            // Select corner speed - if wrap-around, then the corner speed is fixed further below
            // We create a distribution that is biased by the desired average speed for the route
            lower_bound_speed = std::max<int>(1, static_cast<int>(max_corner_speed * corner_speed_min));
            upper_bound_speed = static_cast<int>(max_corner_speed * corner_speed_max);

            if (average_speed > upper_bound_speed) {
              speed_mean = (upper_bound_speed + lower_bound_speed) / 2.0;
              speed_dev = speed_mean / 6.0;
            } else {
              speed_mean = average_speed;
              speed_dev = average_speed / 6.0;
            }
            speed_dist = std::normal_distribution<double>(speed_mean, speed_dev);
            logger("Created corner speed gaussian distribution with lower bound " +
                   std::to_string(lower_bound_speed) + "m/s, upper bound " +
                   std::to_string(upper_bound_speed) + "m/s, mean of " +
                   std::to_string(speed_mean) + ", and deviation of " +
                   std::to_string(speed_dev));

            // Loop until we find a valid corner speed
            int count2 = 0;
            bool valid_corner_speed = false;
            while (!valid_corner_speed) {
              count2++;
              if (count2 == max_iters) {
                return false;
              }

              if (corner_idx == num_corners) {
                proposed_corner_speed = first_corner_speeds.top();
              } else {
                proposed_corner_speed = bounded_gaussian<int>(speed_dist, speed_rng, lower_bound_speed,
                                                              upper_bound_speed, logger);
              }
              logger("Trying corner speed " + std::to_string(proposed_corner_speed) + "m/s");

              // Select a location to start decelerating
              if (corner_start - 1 < acceleration_end_idx) {
                // Crossover from one loop to the next
                idx_dist = std::uniform_int_distribution<size_t>(acceleration_end_idx + 1,
                                                                this->num_points + corner_start - 1);
                logger("Created deceleration index distribution with lower bound " +
                      std::to_string(acceleration_end_idx + 1) +
                      " and upper bound " + std::to_string(this->num_points + corner_start - 1) +
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
              if (proposed_deceleration > 0.0 && proposed_deceleration < max_acceleration &&
                  proposed_deceleration * car_mass * proposed_corner_speed <= acceleration_power_allowance) {
                valid_corner_speed = true;
              } else if (proposed_deceleration < 0.0 && proposed_deceleration > max_deceleration) {
                valid_corner_speed = true;
              } else {
                logger("Necessary deceleration to reach next corner exceeds maximum acceleration/deceleration "
                       "or exceeds maximum acceleration power allowance");
              }

              // Check that the selected corner speed allows the car to reach the next corner's range of speeds in half
              // the straight distance
              distance_to_next_corner = route_distances.get_value(corner_end, next_corner_start);
              logger("Distance to next corner is " + std::to_string(distance_to_next_corner));
              const std::pair<double, double> next_corner_range = {next_corner_max_speed * corner_speed_min,
                                                                  next_corner_max_speed * corner_speed_max};
              if (!can_reach_speeds(proposed_corner_speed, acceleration_power_allowance, preferred_acceleration,
                                    preferred_deceleration, next_corner_range, distance_to_next_corner, car_mass)) {
                logger("Corner speed is too high to reach the next corner");
                valid_corner_speed = false;
                continue;
              }
            }

            if (!normal_straight) {
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
            }
            valid = true;
          }
        } else {
          normal_straight = true;
        }

        // Create a normal straight: Pick a corner speed, accelerate/decelerate to the corner speed
        // in one shot and maintain it
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

          // Center the gaussian distribution at the desired average speed
          lower_bound_speed = std::max<int>(1, static_cast<int>(max_corner_speed * corner_speed_min));
          upper_bound_speed = static_cast<int>(max_corner_speed * corner_speed_max);
          if (average_speed > upper_bound_speed) {
            speed_mean = (upper_bound_speed + lower_bound_speed) / 2.0;
            speed_dev = speed_mean / 6.0;
          } else {
            speed_mean = average_speed;
            speed_dev = speed_mean / 6.0;
          }
          speed_dist = std::normal_distribution<double>(speed_mean, speed_dev);
          logger("Created corner speed gaussian distribution with lower bound " + std::to_string(lower_bound_speed) +
                "m/s, upper bound " + std::to_string(upper_bound_speed) + "m/s, mean of " +
                std::to_string(speed_mean) + " and deviation of " + std::to_string(speed_dev));

          valid = false;
          count = 0;
          sampled_speeds.clear();
          while (!valid) {
            count = count + 1;
            if (count == max_iters) {
              return false;
            }
            // Pick a corner speed
            if (corner_idx == num_corners) {
              proposed_corner_speed = first_corner_speeds.top();
              // If this isn't the first time that we tried reaching the corner speed, unsupport
              // and rollback
              if (sampled_speeds.find(proposed_corner_speed) != sampled_speeds.end()) {
                return false;
              }
              sampled_speeds.insert(proposed_corner_speed);
            } else {
              proposed_corner_speed = bounded_gaussian<int>(speed_dist, speed_rng, lower_bound_speed,
                                                            upper_bound_speed, logger);
              // If all speeds have been attempted, return false and rollback to the last corner
              if (all_speeds_sampled()) {
                return false;
              } else if (sampled_speeds.find(proposed_corner_speed) != sampled_speeds.end()) {
                // If the sampled speed was already tried, sample another speed
                continue;
              } else {
                sampled_speeds.insert(proposed_corner_speed);
              }
            }

            distance_to_next_corner = route_distances.get_value(corner_end, next_corner_start);
            const std::pair<double, double> next_corner_speed_range = {next_corner_max_speed * corner_speed_min,
                                                                      next_corner_max_speed * corner_speed_max};
            if (proposed_corner_speed > last_real_corner_speed) {
              // Need to accelerate to the current corner
              // Parameters to search for
              double acceleration_distance;
              double acceleration_ending_speed = proposed_corner_speed;
              size_t acceleration_end_idx;

              logger("Selected corner speed is " + std::to_string(proposed_corner_speed) +
                    " m/s which is greater than the last corner speed of " +
                    std::to_string(last_real_corner_speed) + "m/s");

              // Find the acceleration that can support the corner speed
              double proposed_acceleration = 0.0;
              do {
                proposed_acceleration += 0.1;
                acceleration_distance = calc_distance_a(last_real_corner_speed, proposed_corner_speed,
                                                        preferred_acceleration);
              } while (proposed_acceleration * car_mass * proposed_corner_speed < acceleration_power_allowance &&
                       acceleration_distance < distance_to_next_corner);
              if (proposed_acceleration == 0.0) {
                logger("Chosen corner speed demands an acceleration that is too high for the motor");
                continue;
              }

              // Use preferred acceleration
              // acceleration_distance = calc_distance_a(last_real_corner_speed,
              //                                         proposed_corner_speed,
              //                                         preferred_acceleration);
              // acceleration_ending_speed = calc_final_speed_a(last_real_corner_speed,
              //                                               preferred_acceleration,
              //                                               max_acceleration_distance);
              // instataneous_motor_power = car_mass * preferred_acceleration * proposed_corner_speed;
              // if (instataneous_motor_power >= acceleration_power_allowance) {
              //   logger("Instataneous motor power " + std::to_string(instataneous_motor_power) +
              //         " W exceeds maximum acceleration power budget");
              //   continue;
              // }

              // See if the car can reach the speed range of the next corner
              if (!can_reach_speeds(proposed_corner_speed, acceleration_power_allowance, 0.2,
                                    preferred_deceleration, next_corner_speed_range, distance_to_next_corner,
                                    car_mass)) {
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

              valid = true;
            } else if (proposed_corner_speed < last_real_corner_speed) {
              // Decelerate to corner speed
              // Parameters to search for
              size_t deceleration_end_idx;
              double deceleration_distance;

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

              // See if the car can reach the speed range of the next corner
              if (!can_reach_speeds(proposed_corner_speed, acceleration_power_allowance,
                                    preferred_acceleration, preferred_deceleration,
                                    next_corner_speed_range, max_acceleration_distance,
                                    car_mass)) {
                logger("The corner speed is too low/high to reach the next corner under the preferred "
                       "acceleration or preferred deceleration");
                continue;
              }

              deceleration_end_idx = last_real_corner_end;
              deceleration_distance = calc_distance_a(last_real_corner_speed,
                                                      proposed_corner_speed,
                                                      preferred_deceleration);
              while (route_distances.get_value(last_real_corner_end, deceleration_end_idx) < deceleration_distance) {
                deceleration_end_idx++;
              }
              logger("Deceleration distance is " + std::to_string(deceleration_distance) + "m");

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
            } else {
              // Maintain same corner speed
              logger("Proposed corner speed is the same as the current speed of the car. Adding one segment");

              acceleration = false;
              acceleration_value = 0.0;
              segment_speed = {proposed_corner_speed, proposed_corner_speed};
              segment = {last_real_corner_end,  corner_end};
              segment_distance = route_distances.get_value(last_real_corner_end, corner_end);
              add_segment();

              logger("\nCORNER SEGMENT");
              logger(get_segment_string());
              valid = true;
            }
          }
        }
        return true;
      };
      if (!create_segments()) {
        logger("Segments could NOT be created. Rolling back to the last segment");
        size_t last_real_corner_idx = real_corner_indices.top();
        logger("Re-creating segments for corner " + std::to_string(last_real_corner_idx) +
               "of loop block " + std::to_string(block_idx));

        remove_last_corner_segments();
        // Remove the last corner indices and speeds since they will be re-found
        real_corner_indices.pop();
        real_corner_speeds.pop();
        num_segments_added.pop();  // This must be called after remove_last_corner_segments()

        // If we failed on corner index 0, we need to rollback the entire last loop block
        if (corner_idx == 0) {
          RUNTIME_EXCEPTION(block_idx > 0, "Failed on the first segment");
          logger("Rolling back to the last corner of the previous loop block");
          loop_idx = loop_idx - num_repetitions;
          block_idx = block_idx - 1;

          // Restore the loop vectors
          loop_segments = loop_segments_stack.top();
          loop_segment_speeds = loop_speeds_stack.top();
          loop_segment_distances = loop_distances_stack.top();
          loop_acceleration_segments = loop_acceleration_stack.top();
          loop_acceleration_values = loop_acceleration_values_stack.top();

          loop_segments_stack.pop();
          loop_speeds_stack.pop();
          loop_distances_stack.pop();
          loop_acceleration_stack.pop();
          loop_acceleration_values_stack.pop();

          RUNTIME_EXCEPTION(all_segments.size() > 0, "Segment plan has no vectors");

          // Remove the last loop block
          all_segments.erase(all_segments.end() - num_repetitions, all_segments.end());
          all_segment_speeds.erase(all_segment_speeds.end() - num_repetitions, all_segment_speeds.end());
          all_segment_distances.erase(all_segment_distances.end() - num_repetitions, all_segment_distances.end());
          all_acceleration_segments.erase(all_acceleration_segments.end() - num_repetitions,
                                          all_acceleration_segments.end());
          all_acceleration_values.erase(all_acceleration_values.end() - num_repetitions,
                                        all_acceleration_values.end());

          // Remove the number of wrap-around segments added to the block prior
          const size_t num_added_segments = num_added_wrap_around_segments.top();
          all_segments.back().erase(all_segments.back().end() - num_added_segments - 1, all_segments.back().end());
          all_segment_speeds.back().erase(all_segment_speeds.back().end() - num_added_segments - 1,
                                          all_segment_speeds.back().end());
          all_segment_distances.back().erase(all_segment_distances.back().end() - num_added_segments - 1,
                                            all_segment_distances.back().end());
          all_acceleration_segments.back().erase(all_acceleration_segments.back().end() - num_added_segments - 1,
                                                 all_acceleration_segments.back().end());
          all_acceleration_values.back().erase(all_acceleration_values.back().end() - num_added_segments - 1,
                                        all_acceleration_values.back().end());
          num_added_wrap_around_segments.pop();
        }
        corner_idx = last_real_corner_idx;

        // If we're rolling back to the first corner of the first loop, set is_first_segment signal
        if (corner_idx == 0 && loop_idx == 0) {
          is_first_segment = true;
        }
      } else {
        // Segment creation was successful
        is_first_segment = false;
        // Update stack structures

        // We don't add no. of wrap-around corners for rolling back to specific corners since it doesn't make sense
        // to ever rollback to the wrap-around corner
        if (segment_counter > 0 && corner_idx != num_corners) {
          num_segments_added.push(segment_counter);
          real_corner_indices.push(corner_idx);
          real_corner_speeds.push(proposed_corner_speed);
          // If we completed the first corner, push the speeds and the ending segment index
          if (corner_idx == 0) {
            first_corner_speeds.push(static_cast<uint64_t>(proposed_corner_speed));
          }
        }

        // In the case of the wrap around corner, we need to add the number of segments so that if we ever
        // rollback to the last corner of the route, we remove the wrap-around as well
        if (segment_counter > 0 && corner_idx == num_corners) {
          num_segments_added.top() += segment_counter;
        }
        corner_idx = corner_idx + 1;
      }
    }

    // Add the created loop num_repetitions times to form a single block
    loop_segments_stack.push(loop_segments);
    loop_speeds_stack.push(loop_segment_speeds);
    loop_acceleration_stack.push(loop_acceleration_segments);
    loop_acceleration_values_stack.push(loop_acceleration_values);
    loop_distances_stack.push(loop_segment_distances);
    const size_t num_loops_created = create_loop_block();
    loop_idx = loop_idx + num_loops_created;
    block_idx = block_idx + 1;
  }

  return RacePlan(all_segments, all_segment_speeds, all_acceleration_segments,
                  all_acceleration_values, all_segment_distances);
}
