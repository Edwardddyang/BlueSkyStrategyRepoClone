/* Class for a sun position LUT
Row Format: |Azimuth(double)|Elevation(double)|Irradiance(double)|Time(string)|
 */

#ifndef LUTS_H
#define LUTS_H

#include <stdlib.h>
#include <string>
#include <vector>
#include <fstream>
#include <cassert>
#include <filesystem>

bool isDouble(std::string str) {
	if (str[0] == '-' && str.size() >= 2) {
		return isdigit(str[1]);
	}
	return isdigit(str[0]);
}

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
            azimuth.emplace_back(azimuthValue);

            std::getline(linestream, cell, ',');
            assert(isDouble(cell) && "Value is not a number.");
            double elevationValue = std::stod(cell);
            elevation.emplace_back(elevationValue);

            std::getline(linestream, cell, ',');
            assert(isDouble(cell) && "Value is not a number.");
            double irradianceValue = std::stod(cell);
            irradiance.emplace_back(irradianceValue);

            std::getline(linestream, cell, ',');
            time.emplace_back(cell); // Assume time is valid without isDouble check
        }

        this->num_rows = azimuth.size();
        this->num_cols = 4;
    }
};

#endif
