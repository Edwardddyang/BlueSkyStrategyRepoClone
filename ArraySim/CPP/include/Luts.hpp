#pragma once

#include <stdlib.h>
#include <string>
#include <vector>
#include <fstream>
#include <filesystem>
#include <iostream>
#include "Utilities.hpp"
#include "time.hpp"
#include "Globals.h"

/* Represent a (latitude, longitude, altitude) coordinate */
struct Coord {
	double lat;
	double lon;
	double alt;

	Coord(double latitude, double longitude, double altitude) : lat(latitude), lon(longitude), alt(altitude) {}
	Coord() : lat(0), lon(0), alt(0) {}
};

/* Parse and store information from a 3 column Route CSV |latitude(deg)|longitude(deg)|altitude(m)|
    as a vector of Coord objects
*/
class RouteLUT {
private:
    // Stores all points in the csv. The length of this vector should be equal to the number of rows
    // in the csv
    std::vector<Coord> route_points;

    // Set equal to route_points.size()
    size_t num_points;
public:
    /** @brief Parse a route csv file
     * @param path: Path object holding the absolute path to the route csv
     */
    RouteLUT(const std::filesystem::path& path);

    inline std::vector<Coord> get_coords() const {return route_points;}
    inline size_t get_num_points() const {return num_points;}
    inline Coord get_coord_value(const size_t& idx) const {
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
    std::vector<Time> times;
    std::vector<std::string> time;

    size_t num_rows;
public:
    /** @brief Parse a Sun Position csv file
     * @param path: Path object holding the absolute path to the sun position csv
     */
    SunPositionLUT(const std::filesystem::path& path);

    // Safe retrieval functions
    inline double get_azimuth_value(const size_t& idx) const {
        RUNTIME_ASSERT(0 <= idx < num_rows, "Illegal access on azimuth values in Sun Position LUT");
        return azimuth[idx];
    }
    inline double get_elevation_value(const size_t& idx) const {
        RUNTIME_ASSERT(0 <= idx < num_rows, "Illegal access on elevation values in Sun Position LUT");
        return elevation[idx];
    }
    inline double get_irradiance_value(const size_t& idx) const {
        RUNTIME_ASSERT(0 <= idx < num_rows, "Illegal access on irradiance values in Sun Position LUT");
        return irradiance[idx];
    }
    inline Time get_time_value(const size_t& idx) const {
        RUNTIME_ASSERT(0 <= idx < num_rows, "Illegal access on time values in Sun Position LUT");
        return times[idx];
    }

    inline size_t get_num_rows() const {return num_rows;}
    inline std::vector<Time> get_times() const {return times;}
    inline std::vector<double> get_azimuths() const {return azimuth;}
    inline std::vector<double> get_elevations() const {return elevation;}
    inline std::vector<double> get_irradiances() const {return irradiance;}
};

/** Holds data for the irradiance csv from the cell irradiance simulation
 * A value at cell (i,j) is the irradiance in W/m^2 for the j-th cell
 * at the i-th sun position
 * 
 * Holds data for the metadata csv from a dynamic cell irradiance simulation
 * |bearing(deg)|latitude(deg)|longitude(deg)|altitude(m)|time strings|sun position idx|
 */
class CellIrradianceCsv {
private:
    // Storage for irradiance csv data
    std::vector<std::vector<double>> irradiance_values;
    size_t num_irr_rows = 0;
    size_t num_irr_cols = 0;

    // Storage for metadata csv data that is created for a dynamic cell irradiance simulation
    std::vector<double> bearings;
    std::vector<Coord> coordinates;
    std::vector<Time> times;
    std::vector<size_t> sun_position_caches;  // The "row" in the sun position csv used for each
                                                // position of the car
    std::vector<std::string> time_strings;

    // Maximum and minimum data value
    std::pair<double, double> irradiance_limits;

public:
    /** @brief Read and parse a created irradiance csv and its corresponding metadata csv
     * if a dynamic simulation was run
     * @param csv_path: Absolute path to the irradiance csv
     */
    CellIrradianceCsv(const std::filesystem::path irr_csv_path,
                      const std::filesystem::path metadata_csv_path = std::filesystem::path{});

    /** @brief Constructor with irradiance csv data and metadata csv data (if present)
     * @param data: Irradiance csv data
     */
    CellIrradianceCsv(std::vector<std::vector<double>> irr_data,
                      std::vector<double> bearing_data = {},
                      std::vector<Coord> coordinate_data = {},
                      std::vector<Time> time_data = {},
                      std::vector<size_t> sun_position_cache_data = {},
                      std::vector<std::string> time_string_data = {});

    /** @brief Write the contents of irradiance_values to a csv file
     * @param csv_path: Absolute path to the irradiance csv file to create
     */
    void write_irr_csv(const std::filesystem::path csv_path) const;

    /** @brief Write the contents of the metada values to a csv file
     * @param csv_path: Absolute path to the metadata csv file to create
     */
    void write_metadata_csv(const std::filesystem::path csv_path) const;

    /** @brief Safe retrieval functions */
    inline double get_irr_value(const size_t& row_idx, const size_t& col_idx) const {
        RUNTIME_ASSERT(0 <= row_idx < num_irr_rows && 0 <= col_idx < num_irr_cols,
                        "Illegal access on irradiance csv");
        return irradiance_values[row_idx][col_idx];
    }
    inline double get_bearing_value(const size_t& idx) const {
        RUNTIME_ASSERT(0 <= idx < num_irr_rows, "Illegal bearing access from metadata csv");
        return bearings[idx];
    }
    inline Coord get_coord_value(const size_t& idx) const {
        RUNTIME_ASSERT(0 <= idx < num_irr_rows, "Illegal coordinate access from metadata csv");
        return coordinates[idx];
    }
    inline std::string get_time_string_value(const size_t& idx) const {
        RUNTIME_ASSERT(0 <= idx < num_irr_rows, "Illegal time string access from metadata csv");
        return time_strings[idx];
    }
    inline size_t get_sun_position_cache_value(const size_t& idx) const {
        RUNTIME_ASSERT(0 <= idx < num_irr_rows, "Illegal sun position cache access from metadata csv");
        return sun_position_caches[idx];
    }
    inline std::vector<double> get_csv_row(const size_t& idx) const {
        RUNTIME_ASSERT(0 <= idx < num_irr_rows, "Illegal row access from irradiance csv");
        return irradiance_values[idx];
    }
    inline size_t get_num_irr_rows() const {return num_irr_rows;}
    inline size_t get_num_irr_cols() const {return num_irr_cols;}

    inline std::pair<double, double> get_irradiance_limits() const {return irradiance_limits;}
    inline std::vector<std::vector<double>> get_csv_data() const {return irradiance_values;}
};
