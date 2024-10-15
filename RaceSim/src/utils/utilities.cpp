#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#include <cassert>
#include <string>
#include <fstream>
#include <sstream>
#include <unordered_set>

#include "utils/Utilities.hpp"
#include "date/date.h"

bool isDouble(const std::string str) {
  if (str[0] == '-' && str.size() >= 2) {
    return isdigit(str[1]);
  }
  return isdigit(str[0]);
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
