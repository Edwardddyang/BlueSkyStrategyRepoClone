/* A variety of general purpose utility functions */

#pragma once

#include <stdlib.h>
#include <stdbool.h>
#include <string>
#include <unordered_set>

#include "utils/Units.hpp"
#include "utils/CustomTime.hpp"

/* Determine if a string can be represented by a double */
bool isDouble(const std::string str);

/** @brief Determine if a string can be converted by a size_t
 * 
 * @param str String with the value to check
 * @param value Pointer to size_t value if conversion is valid
 * @return true if conversion was successful
 */
bool isSizeT(const std::string str, size_t* value);

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

///////////////////////////
// Kinematics Equations
///////////////////////////

/** @brief Calculate final velocity given initial velocity, acceleration and time
 * @param init_speed Initial speed in m/s
 * @param acceleration acceleration in m/s^2
 * @param time Time in seconds
*/
double calc_final_speed(const double init_speed, const double acceleration, const double time);

/** @brief Calculate time given initial velocity, acceleration, and distance
 * @param init_speed Initial speed in m/s
 * @param acceleration acceleration in m/s^2
 * @param distance distance in m
 */
double calc_time(const double init_speed, const double acceleration, const double distance);

/** @brief Calculate acceleration given initial velocity, ending velocity, and distance
 * @param init_speed Initial speed in m/s
 * @param final_speed Final speed in m/s
 * @param distance Total distance in m
 */
double calc_acceleration(const double init_speed, const double final_speed, const double distance);

/** @brief Calculate final velocity given initial velocity, acceleration and distance
 * @param init_speed Initial speed in m/s
 * @param acceleration Acceleration in m/s^2
 * @param distance Distance to cover
*/
double calc_final_speed_a(const double init_speed, const double acceleration, const double distance);
