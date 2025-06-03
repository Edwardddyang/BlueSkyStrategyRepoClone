#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#include <cassert>
#include <string>
#include <fstream>
#include <sstream>
#include <random>
#include <unordered_set>

#include "utils/Utilities.hpp"
#include "utils/CustomException.hpp"
#include "date/date.h"

bool isFormattedISO8601(const std::string& time) {

  // Check if size, hyphens, colons and spaces matches YYYY-MM-DD HH:MM:SS
  if (time.size() != 19 || time[4] != '-' || time[7] != '-' || time[10] != 'T' ||
      time[13] != ':' || time[16] != ':') {
    return false;
  }

  // Check if all other chars are digits
  for (size_t i = 0; i < time.size(); i++) {
    if (i == 4 || i == 7 || i == 10 || i == 13 || i == 16) {
      // These are separators, skip digit check
      continue;
    }
    if (!isdigit(time[i])) {
      return false;
    }
  }

  return true;
}


bool isDouble(const std::string str) {
  if (str[0] == '-' && str.size() >= 2) {
    return isdigit(str[1]);
  }
  return isdigit(str[0]);
}

bool isSizeT(const std::string str, size_t* value) {
  try {
    // Attempt to convert the string to a size_t value
    *value = std::stoul(str);
    return true;
  } catch (const std::exception& e) {
    return false;
  }
}

std::unordered_set<size_t> convert_string_to_int_set(const std::string input) {
  std::unordered_set<size_t> result_set;

  std::stringstream ss(input);
  std::string value;

  while (std::getline(ss, value, ',')) {
    // Convert substring into integer and insert into set
    result_set.insert(std::stoi(value));
  }

  return result_set;
}

// Create a coordinate struct from a lat, lon, altitude string
Coord create_coord(const std::string input) {
  double lat, lon, alt;
  lat = lon = alt = -1.0;

  std::stringstream ss(input);
  std::string value;

  int count = 1;
  while (std::getline(ss, value, ',')) {
    if (count == 1) {
      lat = std::stod(value);
    } else if (count == 2) {
      lon = std::stod(value);
    } else {
      alt = std::stod(value);
    }
    count++;
  }

  return Coord(lat, lon, alt);
}

bool sample_binary(unsigned int seed, double p) {
  RUNTIME_EXCEPTION(p >= 0.0 && p <= 1.0, "Probability for sample binary must be between 0 and 1");
  std::mt19937 gen(seed);
  std::uniform_real_distribution<> dis(0.0, 1.0);

  // Generate a random number and compare it to p
  double random_value = dis(gen);
  return true;
}

double calc_final_speed(const double init_speed, const double acceleration, const double time) {
  return init_speed + acceleration * time;
}

double calc_time(const double init_speed, const double acceleration, const double distance) {
  if (acceleration == 0.0) {
    return distance / init_speed;
  }
  double discriminant = init_speed * init_speed + 2.0 * acceleration * distance;

  if (discriminant < 0.0) {
    throw InvalidCalculation("Discriminant is negative on acceleration time calculation with acceleration " +
                             std::to_string(acceleration) + ", distance " + std::to_string(distance) +
                             ", and initial speed " + std::to_string(init_speed));
  }

  double result_1 = (-1.0 * init_speed + std::sqrt(discriminant)) / acceleration;
  double result_2 = (-1.0 * init_speed - std::sqrt(discriminant)) / acceleration;
  if (result_1 > 0.0 && result_2 < 0.0) {
    return result_1;
  } else if (result_2 > 0.0 && result_1 < 0.0) {
    return result_2;
  } else if (result_1 > result_2) {
    return result_2;
  } else {
    return result_1;
  }
}

double calc_acceleration(const double init_speed, const double final_speed, const double distance) {
  RUNTIME_EXCEPTION(distance != 0.0, "Cannot have distance == 0.0, in acceleration calculation");
  return ((final_speed * final_speed - init_speed * init_speed) / (2.0 * distance));
}

double calc_final_speed_a(const double init_speed, const double acceleration, const double distance) {
  if (acceleration == 0.0) {
    return init_speed;
  }
  RUNTIME_EXCEPTION(init_speed >= 0.0, "Cannot have negative speed in final speed calculation");
  const double sqrt_operand = init_speed * init_speed + 2 * acceleration * distance;
  if (sqrt_operand < 0.0) {
    throw InvalidCalculation("Negative sqrt operand");
  }
  return std::sqrt(sqrt_operand);
}

double calc_distance_a(const double init_speed, const double final_speed, const double acceleration) {
  RUNTIME_EXCEPTION(acceleration != 0.0, "Acceleration cannot be 0");
  return (final_speed * final_speed - init_speed * init_speed) / (2.0 * acceleration);
}

bool can_reach_speeds(double initial_speed, double acceleration_power, double max_acceleration, double max_deceleration, std::pair<double, double> speed_range, double max_distance, double car_mass) {
  RUNTIME_EXCEPTION(speed_range.first <= speed_range.second, "Speed range must be ordered as {smaller, bigger}");
  RUNTIME_EXCEPTION(initial_speed >= 0.0, "Initial speed must be >= 0 m/s");
  RUNTIME_EXCEPTION(max_distance >= 0.0, "Distance must be >= 0.0");
  RUNTIME_EXCEPTION(max_acceleration > 0.0 && max_deceleration < 0.0, "Maximum acceleration must be positive, "
  "and maximum deceleration must be negative");

  if (initial_speed >= speed_range.first && initial_speed <= speed_range.second) {
    return true;
  }
  double distance = 0.0;
  double final_speed;
  double acceleration;
  double instataneous_motor_power = 0.0;
  if (initial_speed <= speed_range.first) {
    acceleration = 0.1;
    do {
      distance = calc_distance_a(initial_speed, speed_range.first, acceleration);
      instataneous_motor_power = speed_range.first * acceleration * car_mass;
      acceleration += 0.1;
      if (distance < max_distance && instataneous_motor_power < acceleration_power) return true;
    } while (instataneous_motor_power < acceleration_power && acceleration < max_acceleration);

    return false;
  }
  if (initial_speed > speed_range.second) {
    acceleration = -0.1;
    do {
      distance = calc_distance_a(initial_speed, speed_range.second, acceleration);
      acceleration -= 0.1;
      if (distance < max_distance) return true;
    } while (acceleration > max_deceleration);

    return false;
  }

  return false;
}

