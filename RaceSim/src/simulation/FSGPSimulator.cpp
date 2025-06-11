#include <memory>
#include <iostream>
#include <limits>
#include <vector>
#include <string>
#include <utility>
#include "sim/FSGPSimulator.hpp"
#include "utils/CustomException.hpp"

std::vector<double> FSGPSimulator::run_sim(std::vector<Coord> telem_coords, std::vector<Time> telem_times,
            std::shared_ptr<ResultsLut> results_lut){

  // Reset results lut logs
  results_lut->reset_logs();

// Ensure telemetry vectors are the same size
RUNTIME_EXCEPTION(telem_coords.size() == telem_times.size(),
    "Mismatch in number of telemetry coordinates and timestamps");

const size_t num_points = telem_coords.size();
    if (num_points < 2) {
        spdlog::warn("Not enough telemetry points to simulate.");
        return {};
    }

  // Initialize simulation state variables
  double accumulated_distance = 0.0;  // In meters
  double driving_time = 0.0;  // In seconds
  Time curr_time = this->sim_start_time;
  double battery_energy = this->sim_start_soc;
  Coord starting_coord = this->sim_start_coord;
  double delta_energy;
  double curr_speed = 0.0;
  bool is_accelerating;
  double acceleration = 0.0;
                
  // Initialize index caches for forecast lut lookups
  std::pair<size_t, size_t> wind_speed_cache = wind_speed_lut->initialize_caches(starting_coord,
                                                                                 curr_time.get_utc_time_point());
  std::pair<size_t, size_t> wind_dir_cache = wind_dir_lut->initialize_caches(starting_coord,
                                                                             curr_time.get_utc_time_point());
  std::pair<size_t, size_t> dni_cache = dni_lut->initialize_caches(starting_coord,
                                                                   curr_time.get_utc_time_point());
  std::pair<size_t, size_t> dhi_cache = dhi_lut->initialize_caches(starting_coord,
                                                                   curr_time.get_utc_time_point());

/* Get telemetry data */
// const size_t num_points = telem_coords.size();
RUNTIME_EXCEPTION(num_points >= 2, "Telemetry must contain at least two coordinates.");

/* Get starting position in telemetry data */
// Coord starting_coord = this->sim_start_coord;
size_t starting_index = 0;
double min_distance = std::numeric_limits<double>::max();

for (size_t i = 0; i < num_points; ++i) {
    double distance = get_distance(telem_coords[i], starting_coord);
    if (distance < min_distance) {
        min_distance = distance;
        starting_index = i;
    }
}

/* (Optional) Iterate through segments of the telemetry path */
for (size_t i = starting_index; i < num_points - 1; ++i) {
    const Coord& start = telem_coords[i];
    const Coord& end = telem_coords[i + 1];

    // Example: calculate segment distance
    double segment_distance = get_distance(start, end);

    // You can also calculate estimated speed, acceleration, etc. here if needed
}

spdlog::debug("Starting SOC: {}", max_soc);


  // bool is_first_day = curr_time >= day_one_start_time && curr_time < day_one_end_time;

  // // // Keep track of the end time of the current day and the start time of the next day
  // // Time current_day_end(curr_time);
  // // Time next_day_start(curr_time);
  // // if (is_first_day) {
  // //   current_day_end.copy_hh_mm_ss(day_one_end_time);
  // // } else {
  // //   current_day_end.copy_hh_mm_ss(day_end_time);
  // // }
  // // next_day_start.copy_hh_mm_ss(day_start_time);
  // // // Advance the timestamp by 24 hours => 24 * 3600 seconds / hour = 86400 seconds
  // next_day_start.update_time_seconds(SECONDS_IN_DAY);


CarUpdate car_update;
// Instead of looping over segments, loop directly over the route points (coords or route_points)
curr_time = telem_times[starting_index];

RUNTIME_EXCEPTION(num_points >= 2, "Telemetry must contain at least two coordinates.");

// Optionally, write starting condition to the result CSV
results_lut->update_logs(car_update, battery_energy, 0.0, 0.0,
                         telem_coords[starting_index], 0.0, curr_time, 0.0);

// Main simulation loop: iterate over each segment (pair of points)
for (size_t i = starting_index; i < num_points - 1; ++i) {
    const Coord& current_coord = telem_coords[i];
    const Coord& next_coord = telem_coords[i + 1];
    const Time& current_time = telem_times[i];
    const Time& next_time = telem_times[i + 1];
    curr_time = current_time;

    double segment_distance = get_distance(current_coord, next_coord);
    double segment_duration = next_time.get_utc_time_point() - current_time.get_utc_time_point();

    acceleration = 0.0;
    delta_energy = 0.0;

    // Update forecast LUT index caches and get forecast data at the src coordinate
    ForecastCoord coord_one_forecast(current_coord.lat, current_coord.lon);

    wind_speed_lut->update_index_cache(&wind_speed_cache, coord_one_forecast, curr_time.get_utc_time_point());
    double wind_speed = wind_speed_lut->get_value(wind_speed_cache);

    wind_dir_lut->update_index_cache(&wind_dir_cache, coord_one_forecast, curr_time.get_utc_time_point());
    double wind_dir = wind_dir_lut->get_value(wind_dir_cache);

    dni_lut->update_index_cache(&dni_cache, coord_one_forecast, curr_time.get_utc_time_point());
    double dni = dni_lut->get_value(dni_cache);

    dhi_lut->update_index_cache(&dhi_cache, coord_one_forecast, curr_time.get_utc_time_point());
    double dhi = dhi_lut->get_value(dhi_cache);

    Wind wind = Wind(wind_dir, wind_speed);
    Irradiance irr = Irradiance(dni, dhi);

    // Compute state update of the car for this segment
    // double constant_distance = get_distance(current_coord, next_coord);
    try {
        car_update = car->compute_travel_update(
            current_coord, next_coord, curr_speed, curr_speed,
            acceleration, &curr_time, wind, irr, 0.0, segment_distance
        );
    } catch (const InvalidCalculation& e) {
        spdlog::error("Invalid calculation during simulation: {}", e.what());
        return {};
    }

    // Update the running state of the simulation
    delta_energy += car_update.delta_energy;
    accumulated_distance += car_update.delta_distance;
    curr_time.update_time_seconds(car_update.delta_time);
    driving_time += car_update.delta_time;
    curr_speed = calc_final_speed(curr_speed, acceleration, car_update.delta_time);

    // Make sure the battery doesn't exceed the maximum bound
    if (battery_energy + delta_energy > max_soc) {
        battery_energy = max_soc;
    } else {
        battery_energy += delta_energy;
    }
    spdlog::debug("Battery Energy: {}", battery_energy);

    // Update the logs
    results_lut->update_logs(car_update, battery_energy, delta_energy, accumulated_distance,
                            next_coord, curr_speed, curr_time, acceleration);

    // Invalid simulation if battery goes below 0 or if the end of the race has been reached
    if (battery_energy < 0.0) {
        // Handle out-of-battery case (e.g., log or break out)
        return {};
    }
}

return {accumulated_distance, driving_time, accumulated_distance / driving_time};

// Optionally, set summary statistics or return results as needed
    }
//   }
//   race_plan->set_driving_time(driving_time);
//   race_plan->set_accumulated_distance(accumulated_distance);
//   race_plan->set_average_speed(accumulated_distance / driving_time);
//   race_plan->set_viability(true);
// }

FSGPSimulator::FSGPSimulator(std::shared_ptr<Car> model) :
  charging_coord(Config::get_instance()->get_overnight_charging_location()),
  impounding_start_time(Config::get_instance()->get_impounding_start_time()),
  impounding_release_time(Config::get_instance()->get_impounding_release_time()),
  car(std::dynamic_pointer_cast<V2Car>(model)),
  WSCSimulator(model) {}
