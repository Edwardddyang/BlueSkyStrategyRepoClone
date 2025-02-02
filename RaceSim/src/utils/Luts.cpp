#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include <sstream>
#include <string>
#include <limits>
#include <filesystem>
#include <vector>

#include "spdlog/spdlog.h"
#include "utils/Luts.hpp"
#include "utils/Defines.hpp"

template <typename T>
BaseLut<T>::BaseLut(const std::filesystem::path path) {
  lut_path = path;
}

void BasicLut::load_LUT() {
  std::fstream lut(this->lut_path);
  RUNTIME_EXCEPTION(lut.is_open(), "File {} not found", lut_path.string());
  std::string line;
  std::string cell;

  while (!lut.eof()) {
    std::getline(lut, line);
    if (line.empty()) continue;
    std::stringstream linestream(line);

    values.emplace_back(std::vector<double>());
    while (!linestream.eof()) {
      std::getline(linestream, cell, ',');
      RUNTIME_EXCEPTION(isDouble(cell), "Value {} is not a number", cell);
      double val = std::stod(cell);
      values.back().emplace_back(val);
    }
  }

  this->num_rows = values.size();
  this->num_cols = values[0].size();
  spdlog::info("Loaded LUT: {}", lut_path.string());
}

double BasicLut::get_value(size_t row_idx, size_t col_idx) const {
  RUNTIME_EXCEPTION(0 <= row_idx && row_idx < num_rows && col_idx < num_cols && num_cols >= 0,
                    "Invalid access in BasicLut {}", lut_path.string());
  return values[row_idx][col_idx];
}

BasicLut::BasicLut(const std::filesystem::path lut_path) : BaseLut<double>(lut_path) {
  load_LUT();
}

BasicLut::BasicLut(std::vector<std::vector<double>> data) {
  values = data;
  num_rows = data.size();
  RUNTIME_EXCEPTION(num_rows > 0, "BasicLut loaded with no data");
  num_cols = data[0].size();

  for (size_t i=0; i < num_rows; i++) {
    RUNTIME_EXCEPTION(num_cols == data[i].size(), "BasicLut have unequal row sizes");
  }
}

void EffLut::load_LUT() {
  std::fstream lut(this->lut_path);
  std::string line;

  // Read first row values
  std::getline(lut, line);
  std::stringstream linestream(line);
  std::string cell;

  std::getline(linestream, cell, ',');  // Remove empty cell at top left of file.

  while (!linestream.eof()) {
    std::getline(linestream, cell, ',');
    RUNTIME_EXCEPTION(isDouble(cell), "Value {} is not a number in Efficiency LUT {}", cell, lut_path.string());
    double column_val = std::stod(cell);
    column_values.push_back(column_val);
  }

  while (!lut.eof()) {
    std::getline(lut, line);
    if (line.empty()) continue;
    std::stringstream linestream(line);

    std::getline(linestream, cell, ',');
    RUNTIME_EXCEPTION(isDouble(cell), "Value {} is not a number in Efficiency LUT {}", cell, lut_path.string());
    double row_val = std::stod(cell);
    row_values.push_back(row_val);

    this->values.emplace_back(std::vector<double>());
    while (!linestream.eof()) {
      std::getline(linestream, cell, ',');
      RUNTIME_EXCEPTION(isDouble(cell), "Value {} is not a number in Efficiency LUT {}", cell, lut_path.string());
      double val = std::stod(cell);
      this->values.back().emplace_back(val);
    }
  }

  this->num_cols = column_values.size();
  this->num_rows = row_values.size();
  spdlog::info("Loaded LUT: {}", lut_path.string());
}

EffLut::EffLut(const std::filesystem::path lut_path) : BaseLut<double>(lut_path) {
  load_LUT();
}

double EffLut::get_value(double row_value, double column_value) {
  size_t row = 0;
  size_t col = 0;

  // Find the closest row index
  for (; row < num_rows; ++row) {
    if (row_values[row] >= row_value) {
      break;
    }
  }
  if (row == num_rows) {
    row--;
  }
  // Adjust row index if necessary
  if (std::abs(row_values[row - 1] - row_value) < std::abs(row_values[row] - row_value)) {
    --row;
  }

  // Find the closest column index
  for (; col < num_cols; ++col) {
    if (column_values[col] >= column_value) {
      break;
    }
  }
  if (col == num_cols) {
    col--;
  }
  // Adjust column index if necessary
  if (std::abs(column_values[col - 1] - column_value) < std::abs(column_values[col] - column_value)) {
    --col;
  }

  RUNTIME_EXCEPTION(row >= 0 && row < num_rows && col >= 0 && col < num_cols,
                    "Invalid access in efficiency LUT {}", lut_path.string());
  return values[row][col];
}

ForecastLut::ForecastLut(const std::filesystem::path lut_path) : BaseLut<double>(lut_path) {
  load_LUT();
}

void ForecastLut::load_LUT() {
  std::fstream file(lut_path);
  std::string times_line;
  file >> times_line;
  std::stringstream times_stream(times_line);

  // Remove 'latitude' and 'longitude' from first 2 cols of csv input.
  std::string time;
  std::getline(times_stream, time, ',');
  std::getline(times_stream, time, ',');

  /* Create an array of the time keys */
  while (!times_stream.eof()) {
    std::getline(times_stream, time, ',');
    RUNTIME_EXCEPTION(isDouble(time), "Time {} is not a number in ForecastLUT {}", time, lut_path.string());
    uint64_t temp_time = std::stoull(time);
    int seconds = temp_time % 100;
    temp_time /= 100;
    int minutes = temp_time % 100;
    temp_time /= 100;
    int hours = temp_time % 100;
    temp_time /= 100;
    int days = temp_time % 100;
    temp_time /= 100;
    int month = (temp_time % 100);
    temp_time /= 100;
    int year = temp_time;

    /* Construct YYYY-MM-DD HH:MM:SS string */
    std::string forecast_time = "20" + std::to_string(year) + "-" + std::to_string(month) + "-"
      + std::to_string(days) + " " + std::to_string(hours) + ":" + std::to_string(minutes) + ":"
      + std::to_string(seconds);

    std::istringstream iss(forecast_time);
    date::sys_time<std::chrono::seconds> epoch_time;
    iss >> date::parse("%F %T", epoch_time);
    time_t local_time_t = std::chrono::system_clock::to_time_t(epoch_time);

    forecast_times.push_back(local_time_t);
  }

  int row_counter = 0;
  while (!file.eof()) {
    std::string file_line;
    file >> file_line;
    std::stringstream file_linestream(file_line);
    if (file_linestream.str().empty()) break;

    std::string cell;
    ForecastCoord coord{};
    std::getline(file_linestream, cell, ',');
    RUNTIME_EXCEPTION(isDouble(cell), "Value {} is not a number in Forecast LUT {}", cell, lut_path.string());
    coord.lat = std::stod(cell);

    std::getline(file_linestream, cell, ',');
    RUNTIME_EXCEPTION(isDouble(cell), "Value {} is not a number in Efficiency LUT {}", cell, lut_path.string());
    coord.lon = std::stod(cell);

    forecast_coords.emplace_back(coord);

    std::getline(file_linestream, cell, ',');
    RUNTIME_EXCEPTION(isDouble(cell), "Value {} is not a number in Efficiency LUT {}", cell, lut_path.string());
    double value = std::stod(cell);
    std::vector<double> inner_vector;
    inner_vector.emplace_back(value);
    this->values.push_back(inner_vector);

    int column_counter = 0;
    while (!file_linestream.eof()) {
      std::getline(file_linestream, cell, ',');
      RUNTIME_EXCEPTION(isDouble(cell), "Value {} is not a number in Efficiency LUT {}", cell, lut_path.string());
      this->values[row_counter].push_back(std::stod(cell));
      column_counter++;
    }

    row_counter++;
  }

  this->num_rows = forecast_coords.size();
  this->num_cols = forecast_times.size();

  row_cache = 0;
  column_cache = 0;
  spdlog::info("Loaded LUT: {}", lut_path.string());
}

double ForecastLut::get_value(ForecastCoord coord, time_t time) {
  double row_key;
  double col_key;

  double min_distance = std::numeric_limits<double>::max();
  for (size_t row=0; row < num_rows; row++) {
    ForecastCoord forecast_coord = forecast_coords[row];
    double distance = get_forecast_coord_distance(coord, forecast_coord);
    if (distance < min_distance) {
      min_distance = distance;
      row_key = row;
    }
  }

  double min_time = std::numeric_limits<double>::max();
  for (size_t col=0; col < num_cols; col++) {
    uint64_t forecast_time = forecast_times[col];
    int time_diff = time - forecast_time;
    if (std::abs(static_cast<double>(time_diff)) < min_time) {
      min_time = std::abs(static_cast<double>(time_diff));
      col_key = col;
    }
  }

  RUNTIME_EXCEPTION(row_key >= 0 && row_key < num_rows && col_key >= 0 && col_key < num_cols,
                    "Out of bounds access in Forecast LUT {}", lut_path.string());
  return this->values[row_key][col_key];
}

void ForecastLut::initialize_caches(ForecastCoord coord, time_t time) {
  /* Initialize row cache */
  Coord forecast_coord_as_coord = Coord(coord);
  double min_distance = std::numeric_limits<double>::max();
  for (size_t i = 0; i < num_rows; i++) {
    Coord forecast_coord = Coord(forecast_coords[i]);
    double distance = get_distance(forecast_coord, forecast_coord_as_coord);
    if (distance < min_distance) {
      min_distance = distance;
      row_cache = i;
    }
  }

  /* Initialize column cache */
  double min_time = std::numeric_limits<double>::max();
  for (size_t i=0; i < num_cols; i++) {
    time_t forecast_time = forecast_times[i];
    int time_diff = time - forecast_time;
    if (std::abs(static_cast<double>(time_diff)) < min_time) {
      min_time = std::abs(static_cast<double>(time_diff));
      column_cache = i;
    }
  }
}

void ForecastLut::initialize_caches(Coord coord, time_t time) {
  /* Initialize row cache */
  double min_distance = std::numeric_limits<double>::max();
  for (size_t i = 0; i < num_rows; i++) {
    Coord forecast_coord = Coord(forecast_coords[i]);
    double distance = get_distance(forecast_coord, coord);
    if (distance < min_distance) {
      min_distance = distance;
      row_cache = i;
    }
  }

  /* Initialize column cache */
  double min_time = std::numeric_limits<double>::max();
  for (size_t i=0; i < num_cols; i++) {
    time_t forecast_time = forecast_times[i];
    int time_diff = time - forecast_time;
    if (std::abs(static_cast<double>(time_diff)) < min_time) {
      min_time = std::abs(static_cast<double>(time_diff));
      column_cache = i;
    }
  }
}

/* Begins searching from the specified indices */
void ForecastLut::update_index_cache(ForecastCoord coord, time_t time) {
  if (row_cache < num_rows-1) {
    ForecastCoord next_coord = forecast_coords[row_cache+1];
    ForecastCoord current_coord = forecast_coords[row_cache];

    double dist_from_current_coord = get_forecast_coord_distance(coord, current_coord);
    double dist_from_next_coord = get_forecast_coord_distance(coord, next_coord);

    row_cache = dist_from_current_coord <= dist_from_next_coord ? row_cache : row_cache+1;
  }
  if (column_cache < num_cols-1) {
    uint64_t current_time = forecast_times[column_cache];
    uint64_t next_time = forecast_times[column_cache+1];

    uint64_t diff_time_from_current = abs(static_cast<double>(time - current_time));
    uint64_t diff_time_from_next = abs(static_cast<double>(time - next_time));

    column_cache = diff_time_from_current <= diff_time_from_next ? column_cache : column_cache+1;
  }
}

double ForecastLut::get_value_with_cache() {
  return this->values[row_cache][column_cache];
}

ResultsLut::ResultsLut(const std::filesystem::path lut_path) : BaseLut<double>(lut_path) {
  load_LUT();
}

void ResultsLut::load_LUT() {
  std::fstream file(this->lut_path);

  // Remove first row (header)
  std::string header_line;
  std::getline(file, header_line);

  while (!file.eof()) {
    std::string file_line;
    std::getline(file, file_line);
    std::stringstream file_linestream(file_line);
    if (file_linestream.str().empty()) break;

    std::string cell;
    std::getline(file_linestream, cell, ',');
    RUNTIME_EXCEPTION(isDouble(cell), "Value {} is not a number in Results csv {}", cell, lut_path.string());
    battery_energy.emplace_back(std::stod(cell));

    std::getline(file_linestream, cell, ',');
    RUNTIME_EXCEPTION(isDouble(cell), "Value {} is not a number in Results csv {}", cell, lut_path.string());
    accumulated_distance.emplace_back(std::stod(cell));

    std::getline(file_linestream, cell, ',');
    time.emplace_back(cell);

    std::getline(file_linestream, cell, ',');
    RUNTIME_EXCEPTION(isDouble(cell), "Value {} is not a number in Results csv {}", cell, lut_path.string());
    azimuth.emplace_back(std::stod(cell));

    std::getline(file_linestream, cell, ',');
    RUNTIME_EXCEPTION(isDouble(cell), "Value {} is not a number in Results csv {}", cell, lut_path.string());
    elevation.emplace_back(std::stod(cell));

    std::getline(file_linestream, cell, ',');
    RUNTIME_EXCEPTION(isDouble(cell), "Value {} is not a number in Results csv {}", cell, lut_path.string());
    bearing.emplace_back(std::stod(cell));

    std::getline(file_linestream, cell, ',');
    RUNTIME_EXCEPTION(isDouble(cell), "Value {} is not a number in Results csv {}", cell, lut_path.string());
    latitude.emplace_back(std::stod(cell));

    std::getline(file_linestream, cell, ',');
    RUNTIME_EXCEPTION(isDouble(cell), "Value {} is not a number in Results csv {}", cell, lut_path.string());
    longitude.emplace_back(std::stod(cell));

    std::getline(file_linestream, cell, ',');
    RUNTIME_EXCEPTION(isDouble(cell), "Value {} is not a number in Results csv {}", cell, lut_path.string());
    altitude.emplace_back(std::stod(cell));

    std::getline(file_linestream, cell, ',');
    RUNTIME_EXCEPTION(isDouble(cell), "Value {} is not a number in Results csv {}", cell, lut_path.string());
    speed.emplace_back(std::stod(cell));

    std::getline(file_linestream, cell, ',');
    RUNTIME_EXCEPTION(isDouble(cell), "Value {} is not a number in Results csv {}", cell, lut_path.string());
    acceleration.emplace_back(std::stod(cell));

    std::getline(file_linestream, cell, ',');
    RUNTIME_EXCEPTION(isDouble(cell), "Value {} is not a number in Results csv {}", cell, lut_path.string());
    array_power.emplace_back(std::stod(cell));

    std::getline(file_linestream, cell, ',');
    RUNTIME_EXCEPTION(isDouble(cell), "Value {} is not a number in Results csv {}", cell, lut_path.string());
    array_energy.emplace_back(std::stod(cell));

    std::getline(file_linestream, cell, ',');
    RUNTIME_EXCEPTION(isDouble(cell), "Value {} is not a number in Results csv {}", cell, lut_path.string());
    motor_power.emplace_back(std::stod(cell));

    std::getline(file_linestream, cell, ',');
    RUNTIME_EXCEPTION(isDouble(cell), "Value {} is not a number in Results csv {}", cell, lut_path.string());
    motor_energy.emplace_back(std::stod(cell));

    std::getline(file_linestream, cell, ',');
    RUNTIME_EXCEPTION(isDouble(cell), "Value {} is not a number in Results csv {}", cell, lut_path.string());
    aero_power.emplace_back(std::stod(cell));

    std::getline(file_linestream, cell, ',');
    RUNTIME_EXCEPTION(isDouble(cell), "Value {} is not a number in Results csv {}", cell, lut_path.string());
    aero_energy.emplace_back(std::stod(cell));

    std::getline(file_linestream, cell, ',');
    RUNTIME_EXCEPTION(isDouble(cell), "Value {} is not a number in Results csv {}", cell, lut_path.string());
    rolling_power.emplace_back(std::stod(cell));

    std::getline(file_linestream, cell, ',');
    RUNTIME_EXCEPTION(isDouble(cell), "Value {} is not a number in Results csv {}", cell, lut_path.string());
    rolling_energy.emplace_back(std::stod(cell));

    std::getline(file_linestream, cell, ',');
    RUNTIME_EXCEPTION(isDouble(cell), "Value {} is not a number in Results csv {}", cell, lut_path.string());
    gravitational_power.emplace_back(std::stod(cell));

    std::getline(file_linestream, cell, ',');
    RUNTIME_EXCEPTION(isDouble(cell), "Value {} is not a number in Results csv {}", cell, lut_path.string());
    gravitational_energy.emplace_back(std::stod(cell));

    std::getline(file_linestream, cell, ',');
    RUNTIME_EXCEPTION(isDouble(cell), "Value {} is not a number in Results csv {}", cell, lut_path.string());
    electric_energy.emplace_back(std::stod(cell));

    std::getline(file_linestream, cell, ',');
    RUNTIME_EXCEPTION(isDouble(cell), "Value {} is not a number in Results csv {}", cell, lut_path.string());
    delta_energy.emplace_back(std::stod(cell));
  }
}

void ResultsLut::reset_logs() {
  battery_energy.clear();
  accumulated_distance.clear();
  time.clear();
  azimuth.clear();
  elevation.clear();
  bearing.clear();
  latitude.clear();
  longitude.clear();
  altitude.clear();
  speed.clear();
  array_energy.clear();
  array_power.clear();
  motor_power.clear();
  motor_energy.clear();
  aero_power.clear();
  aero_energy.clear();
  rolling_power.clear();
  rolling_energy.clear();
  gravitational_power.clear();
  gravitational_energy.clear();
  electric_energy.clear();
  delta_energy.clear();
}

void ResultsLut::write_logs(const std::string lut_path) const {
  std::ofstream output_csv(lut_path);
  RUNTIME_EXCEPTION(output_csv.is_open(), "Unable to open csv file path for writing: {}", lut_path);

  output_csv  << "Battery Charge(kWh),"
              << "Accumulated Distance(m),"
              << "DateTime,"
              << "Azimuth(Degrees),"
              << "Elevation(Degrees),"
              << "Bearing(Degrees),"
              << "Latitude,"
              << "Longitude,"
              << "Altitude(m),"
              << "Speed(m/s),"
              << "Acceleration(m/s^2),"
              << "Array Power(W),"
              << "Array Energy(kWh),"
              << "Motor Power(W),"
              << "Motor Energy(kWh),"
              << "Aero Power(W),"
              << "Aero Energy(kWh),"
              << "Rolling Power(W),"
              << "Rolling Energy(kWh),"
              << "Gravitational Power(W),"
              << "Gravitational Energy(kWh),"
              << "Electric Energy(W),"
              << "Delta Battery(kWh),\n";

  size_t num_points = battery_energy.size();
  for (size_t i=0; i < num_points; i++) {
      output_csv << std::to_string(battery_energy[i]) + ",";
      output_csv << std::to_string(accumulated_distance[i]) + ",";
      output_csv << (std::string(time[i]) + ",");
      output_csv << std::to_string(azimuth[i]) + ",";
      output_csv << std::to_string(elevation[i]) + ",";
      output_csv << std::to_string(bearing[i]) + ",";
      output_csv << std::to_string(latitude[i]) + ",";
      output_csv << std::to_string(longitude[i]) + ",";
      output_csv << std::to_string(altitude[i]) + ",";
      output_csv << std::to_string(speed[i]) + ",";
      output_csv << std::to_string(acceleration[i]) + ",";
      output_csv << std::to_string(array_power[i]) + ",";
      output_csv << std::to_string(array_energy[i]) + ",";
      output_csv << std::to_string(motor_power[i]) + ",";
      output_csv << std::to_string(motor_energy[i]) + ",";
      output_csv << std::to_string(aero_power[i]) + ",";
      output_csv << std::to_string(aero_energy[i]) + ",";
      output_csv << std::to_string(rolling_power[i]) + ",";
      output_csv << std::to_string(rolling_energy[i]) + ",";
      output_csv << std::to_string(gravitational_power[i]) + ",";
      output_csv << std::to_string(gravitational_energy[i]) + ",";
      output_csv << std::to_string(electric_energy[i]) + ",";
      output_csv << std::to_string(delta_energy[i]) + ",\n";
  }
}

void ResultsLut::update_logs(const CarUpdate update, double battery, double d_energy,
                             double distance, Coord next_coord, double curr_speed, Time curr_time,
                             double accel) {
  battery_energy.push_back(battery);
  delta_energy.push_back(d_energy);
  accumulated_distance.push_back(distance);
  azimuth.push_back(update.az_el.Az);
  elevation.push_back(update.az_el.El);
  bearing.push_back(update.bearing);
  latitude.push_back(next_coord.lat);
  longitude.push_back(next_coord.lon);
  altitude.push_back(next_coord.alt);
  array_energy.push_back(update.array.energy);
  array_power.push_back(update.array.power);
  speed.push_back(curr_speed);
  acceleration.push_back(accel);
  motor_power.push_back(update.motor_power);
  motor_energy.push_back(update.motor_energy);
  aero_power.push_back(update.aero.power);
  aero_energy.push_back(update.aero.energy);
  rolling_power.push_back(update.rolling.power);
  rolling_energy.push_back(update.rolling.energy);
  gravitational_power.push_back(update.gravitational.power);
  gravitational_energy.push_back(update.gravitational.energy);
  electric_energy.push_back(update.electric);
  time.push_back(curr_time.get_local_readable_time());
}
