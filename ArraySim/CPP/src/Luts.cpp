#include "Luts.hpp"

RouteLUT::RouteLUT(const std::filesystem::path& path) {
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
    RUNTIME_ASSERT(num_points > 0, "Route LUT has no data " + path.string());
    RUNTIME_ASSERT(num_points == route_points.size(), "Route LUT parsing has gone terribly wrong");
}

SunPositionLUT::SunPositionLUT(const std::filesystem::path& path)  {
    std::ifstream lut(path.string());
    RUNTIME_ASSERT(lut.is_open(), "Sun Position LUT could not be opened " + path.string());

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
    RUNTIME_ASSERT(num_rows > 0, "SunPosition LUT has no data " + path.string());

    RUNTIME_ASSERT(num_rows == azimuth.size() &&
                    num_rows == elevation.size() &&
                    num_rows == irradiance.size() &&
                    num_rows == time.size(),
                    "SunPosition LUT parsing has gone terribly wrong");
}

CellIrradianceCsv::CellIrradianceCsv(const std::filesystem::path irr_csv_path,
                                     const std::filesystem::path metadata_csv_path) {
    std::string line;
    std::ifstream irr_csv(irr_csv_path.string());

    RUNTIME_ASSERT(irr_csv.is_open(), "Irradiance CSV could not be opened " + irr_csv_path.string());
    double max_irradiance_value = std::numeric_limits<double>::lowest();
    double min_irradiance_value = std::numeric_limits<double>::max();
    num_irr_rows = 0;
    num_irr_cols = 0;

    while (std::getline(irr_csv, line)) {
        std::stringstream ss(line);
        std::vector<double> row;
        std::string cell;
        while (std::getline(ss, cell, ',')) {
            try {
                double value = std::stod(cell);
                max_irradiance_value = value > max_irradiance_value ? value : max_irradiance_value;
                min_irradiance_value = value < min_irradiance_value ? value : min_irradiance_value;
                row.push_back(value);
            } catch (const std::invalid_argument& e) {
                RUNTIME_ASSERT(false, "Irradiance CSV has invalid value: " + cell);
            }
        }
        irradiance_values.push_back(row);

        if (num_irr_cols == 0) {
            num_irr_cols = row.size();
        }
        RUNTIME_ASSERT(num_irr_cols == row.size(), "Irradiance CSV doesn't have uniform row lengths " + irr_csv_path.string());
        num_irr_rows = num_irr_rows + 1;
    }
    RUNTIME_ASSERT(num_irr_rows > 0, "Irradiance CSV is empty " + irr_csv_path.string());

    irradiance_limits = {min_irradiance_value, max_irradiance_value};

    if (!metadata_csv_path.empty()) {
        std::ifstream metadata_csv(metadata_csv_path.string());
        RUNTIME_ASSERT(metadata_csv.is_open(), "Metadata CSV could not be opened " + metadata_csv_path.string());
        while(std::getline(metadata_csv, line)) {
            if (line.empty()) continue;
            std::stringstream linestream(line);
            std::string cell;

            double lat, lon, alt, bearing;
            int sun_position_cache;
            std::string time_s;

            std::getline(linestream, cell, ',');
            RUNTIME_ASSERT(isDouble(cell), "Value in metadata csv is not a number " + cell);
            bearing = std::stod(cell);
            bearings.emplace_back(bearing);
            
            std::getline(linestream, cell, ',');
            RUNTIME_ASSERT(isDouble(cell), "Value in metadata csv is not a number " + cell);
            lat = std::stod(cell);

            std::getline(linestream, cell, ',');
            RUNTIME_ASSERT(isDouble(cell), "Value in metadata csv is not a number " + cell);
            lon = std::stod(cell);

            std::getline(linestream, cell, ',');
            RUNTIME_ASSERT(isDouble(cell), "Value in metadata csv is not a number " + cell);
            alt = std::stod(cell);

            Coord new_coord(lat, lon, alt);
            coordinates.emplace_back(new_coord);

            std::getline(linestream, cell, ',');
            time_s = cell;
            time_strings.emplace_back(time_s);

            std::getline(linestream, cell, ',');
            sun_position_cache = std::stoi(cell);
            sun_position_caches.emplace_back(sun_position_cache);
        }
        RUNTIME_ASSERT(bearings.size() == num_irr_rows &&
                        coordinates.size() == num_irr_rows &&
                        times.size() == num_irr_rows &&
                        sun_position_caches.size() == num_irr_rows &&
                        time_strings.size() == num_irr_rows,
                        "Metadata csv data do not have uniform lengths");
    }
}

CellIrradianceCsv::CellIrradianceCsv(std::vector<std::vector<double>> irr_data,
                                    std::vector<double> bearing_data,
                                    std::vector<Coord> coordinate_data,
                                    std::vector<Time> time_data,
                                    std::vector<size_t> sun_position_cache_data,
                                    std::vector<std::string> time_string_data) {
    RUNTIME_ASSERT(irr_data.size() > 0, "Irradiance csv constructor received no data");
    irradiance_values = irr_data;

    num_irr_rows = irr_data.size();
    num_irr_cols = 0;
    double max_irradiance_value = std::numeric_limits<double>::lowest();
    double min_irradiance_value = std::numeric_limits<double>::max();
    for (size_t i = 0; i < num_irr_rows; i++) {
        if (num_irr_cols == 0) {
            num_irr_cols = irr_data[i].size();
        }
        RUNTIME_ASSERT(num_irr_cols == irr_data[i].size(), "Irradiance CSV constructor data doesn't have uniform row lengths");
        for (size_t j = 0; j < num_irr_cols; j++) {
            double value = irr_data[i][j];
            max_irradiance_value = value > max_irradiance_value ? value : max_irradiance_value;
            min_irradiance_value = value < min_irradiance_value ? value : min_irradiance_value;
        }
    }
    irradiance_limits = {min_irradiance_value, max_irradiance_value};

    if (bearing_data.size() > 0) {
        RUNTIME_ASSERT(bearing_data.size() == num_irr_rows &&
                        coordinate_data.size() == num_irr_rows &&
                        time_data.size() == num_irr_rows &&
                        sun_position_cache_data.size() == num_irr_rows &&
                        time_string_data.size() == num_irr_rows,
                        "Metadata csv data do not have uniform lengths");

        bearings = bearing_data;
        coordinates = coordinate_data;
        times = time_data;
        sun_position_caches = sun_position_cache_data;
        time_strings = time_string_data;
    }
}

void CellIrradianceCsv::write_irr_csv(const std::filesystem::path csv_path) const {
    std::ofstream lut(csv_path.string());
    RUNTIME_ASSERT(lut.is_open(), "Irradiance CSV could not be opened for writing " + csv_path.string());

    lut << std::fixed << std::setprecision(8);

    for (const std::vector<double> row : irradiance_values) {
        size_t num_cells = row.size();
        for (size_t i=0; i<num_cells; i++) {
            lut << row[i];
            lut << ",";
        }

        lut << "\n";
    }

    lut.close();
}

void CellIrradianceCsv::write_metadata_csv(const std::filesystem::path csv_path) const {
    std::ofstream metadata_csv(csv_path.string());
    RUNTIME_ASSERT(metadata_csv.is_open(), "Metadata CSV could not be opened for writing " + csv_path.string());
    RUNTIME_ASSERT(bearings.size() == num_irr_rows &&
                    coordinates.size() == num_irr_rows &&
                    times.size() == num_irr_rows &&
                    time_strings.size() == num_irr_rows &&
                    sun_position_caches.size() == num_irr_rows,
                    "Metadata vectors are empty or have non-uniform sizes");

    metadata_csv << std::fixed << std::setprecision(8);
    size_t idx = 0;
    for (const Coord coord : coordinates) {
        metadata_csv << bearings[idx] << ','
                    << coord.lat << ',' << coord.lon << ',' << coord.alt << ','
                    << times[idx].get_local_readable_time() << ',' << sun_position_caches[idx] << ",\n";
        idx++;
    }
    metadata_csv.close();
}
