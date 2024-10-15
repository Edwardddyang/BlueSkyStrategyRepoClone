/* Functions for geographical related calculations */

#pragma once

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include "utils/Units.hpp"
#include "utils/CustomTime.hpp"

/* Calculate haversine distance between two lat/lon/alt points */
/** @brief Calculate haversine distance between two lat/lon/alt points
 * point 1 -> point 2
 *
 * @param src_coord: point 1
 * @param dst_coord: point 2
 */
double get_distance(Coord src_coord, Coord dst_coord);

/** @brief Calculate haversine distance between two lat/lon points
 * point 1 -> point 2
 *
 * @param src_coord: point 1
 * @param dst_coord: point 2
 */
double get_forecast_coord_distance(ForecastCoord src_coord, ForecastCoord dst_coord);

/** @brief Calculate bearing for a car at src_coord pointing towards dst_coord.
 * Units are in degrees cw from true north
 * 
 * @param src_coord: Location of the tail of the car
 * @param dst_coord: Direction of the nose of the car
*/
double get_bearing(Coord src_coord, Coord dst_coord);

/** @brief Get azimuth and elevation angles of the sun relative to the car
 * 
 * @param bearing: The bearing of the car nose in degrees. Convention is clockwise from north
 * @param coord: The coordinates of the car
 * @param time: Time of day
 */
SolarAngle get_az_el_from_bearing(double bearing, Coord coord, const Time* time);

/** @brief Get car speed relative to wind speed
 * 
 * Note: Only headwind considered
 * 
 * @param car_speed: Speed of the car in m/s
 * @param car_bearing: Bearing of the car nose in degrees. Convention is clockwise from north
 * @param wind: Wind speed (m/s) and direction (degrees cw from north)
 */
double get_speed_relative_to_wind(double car_speed, double car_bearing, Wind wind);

/** @brief Get julian day from a utc time
 * 
 * Note: time_t is the number of seconds since epoch beginning
 * 
 * @param utc_time_point: UTC time relative to epoch
 */
double julian_day(time_t utc_time_point);

/*
Copyright(c) 2010, Darin Koblick
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met :

*Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED.IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
void get_az_el(time_t utc_time_point, double Lat, double Lon, double Alt, double* Az, double* El);
