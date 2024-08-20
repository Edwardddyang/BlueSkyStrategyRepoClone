/* Class for a sun position LUT
Row Format: |Azimuth(double)|Elevation(double)|Irradiance(double)|Time(string)|
 */

#pragma once

#include <stdlib.h>
#include <string>
#include <vector>
#include <fstream>
#include <cassert>
#include <filesystem>
#include <iostream>
#include "Utilities.hpp"

class SunPositionLUT {
protected:
    /* Absolute path to LUT */
    std::string lut_path;

    /* LUT Values */
    std::vector<double> azimuth;
    std::vector<double> elevation;
    std::vector<double> irradiance;
    std::vector<std::string> time;

    /* Dimensions of the LUT */
    size_t num_rows;
    size_t num_cols;
public:
    SunPositionLUT(const std::filesystem::path& path) {
        std::ifstream lut(path.string()); // Use ifstream for reading
        assert(lut.is_open() && "File not found...");

        std::string line;
        while (std::getline(lut, line)) {
            if (line.empty()) continue;
            std::stringstream linestream(line);
            std::string cell;

            std::getline(linestream, cell, ',');
            assert(isDouble(cell) && "Value is not a number.");
            double azimuthValue = std::stod(cell);
            assert(azimuthValue >= 0.0 && azimuthValue <= 360.0);
            azimuth.emplace_back(azimuthValue);

            std::getline(linestream, cell, ',');
            assert(isDouble(cell) && "Value is not a number.");
            double elevationValue = std::stod(cell);
            assert(elevationValue >= 0.0 && elevationValue <= 90.0);
            elevation.emplace_back(elevationValue);

            std::getline(linestream, cell, ',');
            assert(isDouble(cell) && "Value is not a number.");
            double irradianceValue = std::stod(cell);
            assert(irradianceValue >= 0.0);
            irradiance.emplace_back(irradianceValue);

            std::getline(linestream, cell, ',');
            time.emplace_back(cell); // Assume time is valid without isDouble check
        }

        this->num_rows = azimuth.size();
        this->num_cols = 4;
    }

    size_t get_num_rows() {return num_rows;}
    size_t get_num_cols() {return num_cols;}

    double get_azimuth_value(size_t idx) {
        if (idx < azimuth.size() && idx >= 0) {
            return azimuth[idx];
        }
        std::cout << "Index out of bounds on azimuth access, returning azimuth's last value" << std::endl;
        return azimuth[num_rows-1];
    }

    double get_elevation_value(size_t idx) {
        if (idx < elevation.size() && idx >= 0) {
            return elevation[idx];
        } 
        std::cout << "Index out of bounds on elevation access, returning azimuth's last value" << std::endl;      
        return elevation[num_rows-1];
    }

    double get_irradiance_value(size_t idx) {
        if (idx < irradiance.size() && idx >= 0) {
            return irradiance[idx];
        }
        std::cout << "Index out of bounds on irradiance access, returning irradiance's last value" << std::endl;
        return irradiance[num_rows-1];
    }

    std::string get_time(size_t idx) {
        if (idx < time.size() && idx >= 0) {
            return time[idx];
        }

        std::cout << "Index out of bounds on time access, returning time last value" << std::endl;
        return time[num_rows-1];
    }
};
