/* A variety of general purpose utility functions */

#pragma once

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <unordered_set>

#include "Units.hpp"
#include "CustomTime.hpp"

/* Determine if a string can be represented by a double */
bool isDouble(const std::string str);

/** Convert a comma seperate string of doubles into a set
 * example: convert_string_to_set("2, 3, 4, 5") --> std::set({1,2,3,4,5})
 * 
 * @param input comma seperated string of integers
*/
std::unordered_set<size_t> convert_string_to_int_set(const std::string input);

/** Convert a comma seperated string into a coordinate 
 * @param input: comma seperated string of doubles in <lat>, <lon>, <alt> format
 * example: create_coord("2.0, 3.0, 4.9") --> Coord({lat: 2.0, lon: 3.0, alt: 4.9})
*/
Coord create_coord(const std::string input);
