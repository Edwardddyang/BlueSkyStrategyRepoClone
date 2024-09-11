/* Class for a sun position LUT
Row Format: |Azimuth(double)|Elevation(double)|Irradiance(double)|Time(string)|
 */

#pragma once

#include <stdlib.h>
#include <string>
#include <vector>
#include <fstream>
#include <filesystem>
#include <iostream>
#include "Utilities.hpp"
#include "Globals.h"

/* Holds a single coordinate */
struct Coord {
	double lat;
	double lon;
	double alt;

	Coord(double latitude, double longitude, double altitude) : lat(latitude), lon(longitude), alt(altitude) {}
	Coord() : lat(0), lon(0), alt(0) {}
};

/* Parse and store information from a 3 column Route CSV |latitude(deg)|longitude(deg)|altitude(m)|*/
class RouteLUT {
private:
    // Stores all points in the csv. The length of this vector should be equal to the number of rows
    // in the csv
    std::vector<Coord> route_points;

    // Set equal to route_points.size()
    size_t num_points;
public:
    inline std::vector<Coord> get_coords() const {return route_points;}
    
    /** @brief Parse a route csv file
     * @param path: Path object holding the absolute path to the route csv
     */
    RouteLUT(const std::filesystem::path& path) {
        std::ifstream lut(path.string());
        RUNTIME_ASSERT(lut.is_open(), "Route LUT could not be opened");

        std::string line;
        num_points = 0;
        while (std::getline(lut, line)) {
            if (line.empty()) continue;
            std::stringstream linestream(line);
            std::string cell;
            double lat, lon, alt;

            std::getline(linestream, cell, ',');
            RUNTIME_ASSERT(isDouble(cell), "Value in Route LUT is not a number " + cell);
            lat = std::stod(cell);
            
            std::getline(linestream, cell, ',');
            RUNTIME_ASSERT(isDouble(cell), "Value in Route LUT is not a number " + cell);
            lon = std::stod(cell);

            std::getline(linestream, cell, ',');
            RUNTIME_ASSERT(isDouble(cell), "Value in Route LUT is not a number " + cell);
            alt = std::stod(cell);

            Coord new_coord(lat, lon, alt);
            route_points.emplace_back(new_coord);
            num_points++;
        }
        RUNTIME_ASSERT(num_points == route_points.size(), "Route LUT parsing has gone terribly wrong");
    }

    inline size_t get_num_points() const {return num_points;}

    Coord get_coord(size_t idx) const {
        RUNTIME_ASSERT(0 <= idx < num_points, "Illegal access on coordinate in RouteLUT object");
        return route_points[idx];
    }

};

/* Parse and store data from a 4 column csv describing the path of the sun through the sky
    |azimuth (deg)|elevation (deg)|irradiance (W/m^2)|time (24 hour)|
 */
class SunPositionLUT {
protected:
    // Stores all data from the sun position csv. Each vector's length should equal
    // the number of rows in the csv
    std::vector<double> azimuth;
    std::vector<double> elevation;
    std::vector<double> irradiance;
    std::vector<std::string> time;

    size_t num_rows;
public:
    /** @brief Parse a Sun Position csv file
     * @param path: Path object holding the absolute path to the sun position csv
     */
    SunPositionLUT(const std::filesystem::path& path) {
        std::ifstream lut(path.string());
        RUNTIME_ASSERT(lut.is_open(), "Sun Position LUT could not be opened");

        std::string line;
        num_rows = 0;
        while (std::getline(lut, line)) {
            if (line.empty()) continue;
            std::stringstream linestream(line);
            std::string cell;

            std::getline(linestream, cell, ',');
            
            RUNTIME_ASSERT(isDouble(cell), "Value in Sun Position LUT is not a number: " + cell);
            double azimuthValue = std::stod(cell);
            RUNTIME_ASSERT(0.0 <= azimuthValue <= 360.0,
                           "Azimuth value in Sun Position LUT is not in range [0.0, 360.0]");
            azimuth.emplace_back(azimuthValue);

            std::getline(linestream, cell, ',');
            RUNTIME_ASSERT(isDouble(cell), "Value in Sun Position LUT is not a number: " + cell);
            double elevationValue = std::stod(cell);
            RUNTIME_ASSERT(0.0 <= elevationValue <= 360.0,
                           "Elevation value in Sun Position LUT is not in range [0.0, 360.0]");
            elevation.emplace_back(elevationValue);

            std::getline(linestream, cell, ',');
            RUNTIME_ASSERT(isDouble(cell), "Value in Sun Position LUT is not a number: " + cell);
            double irradianceValue = std::stod(cell);
            RUNTIME_ASSERT(irradianceValue >= 0.0,
                           "Irradiance value in Sun Position LUT must be greater than or equal to 0");
            irradiance.emplace_back(irradianceValue);

            std::getline(linestream, cell, ',');
            time.emplace_back(cell);
            num_rows++;
        }

        RUNTIME_ASSERT(num_rows == azimuth.size() &&
                       num_rows == elevation.size() &&
                       num_rows == irradiance.size() &&
                       num_rows == time.size(),
                        "SunPosition LUT parsing has gone terribly wrong");
    }

    inline size_t get_num_rows() const {return num_rows;}

    double get_azimuth_value(size_t idx) const {
        RUNTIME_ASSERT(0 <= idx < num_rows, "Illegal access on azimuth values in Sun Position LUT");
        return azimuth[idx];
    }

    double get_elevation_value(size_t idx) const {
        RUNTIME_ASSERT(0 <= idx < num_rows, "Illegal access on elevation values in Sun Position LUT");
        return elevation[idx];
    }

    double get_irradiance_value(size_t idx) const {
        RUNTIME_ASSERT(0 <= idx < num_rows, "Illegal access on irradiance values in Sun Position LUT");
        return irradiance[idx];
    }

    std::string get_time(size_t idx) const {
        RUNTIME_ASSERT(0 <= idx < num_rows, "Illegal access on time values in Sun Position LUT");
        return time[idx];
    }
};
