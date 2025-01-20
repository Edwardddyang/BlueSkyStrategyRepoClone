#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#include <cassert>
#include <string>
#include <fstream>
#include <sstream>
#include <unordered_set>

#include "utils/Utilities.hpp"
#include "utils/CustomException.hpp"
#include "date/date.h"

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

double calc_final_speed(const double init_speed, const double acceleration, const double time) {
  return init_speed + acceleration * time;
}

double calc_time(const double init_speed, const double acceleration, const double distance) {
  if (acceleration == 0.0) {
    return distance / init_speed;
  }
  double discriminant = init_speed * init_speed + 2.0 * acceleration * distance;

  if (discriminant < 0.0) {
    throw InvalidCalculation("Discriminant is negative");
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
