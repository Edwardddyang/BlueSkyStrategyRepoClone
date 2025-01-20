#include <memory>
#include <iostream>
#include <limits>
#include <vector>
#include <string>
#include <utility>
#include "sim/FSGPSimulator.hpp"
#include "utils/CustomException.hpp"

void FSGPSimulator::run_sim(const std::shared_ptr<Route>& route, RacePlan* race_plan) {
  RUNTIME_EXCEPTION(route != nullptr, "Route pointer is null");
  RUNTIME_EXCEPTION(results_lut != nullptr, "Results lut is null");
  RUNTIME_EXCEPTION(race_plan != nullptr, "Race plan is null");
  RUNTIME_EXCEPTION(race_plan->validate_members(route->get_route_points()), "Race Plan is improperly created");

  /* Reset variables */
  reset_vars();
  results_lut->reset_logs();

  /* Get route data */
  const size_t num_points = route->get_num_points();
  const std::vector<Coord> route_points = route->get_route_points();

  /* Get race plan data */
  const std::vector<std::vector<std::pair<size_t, size_t>>> segments = race_plan->get_segments();
  const std::vector<std::vector<std::pair<double, double>>> segment_speeds = race_plan->get_speed_profile();
  const std::vector<std::vector<bool>> acceleration_segments = race_plan->get_acceleration_segments();
  const std::vector<std::vector<double>> acceleration_values = race_plan->get_acceleration_values();

  /* Get starting position in the route */
  size_t starting_route_index = 0;
  double min_distance = std::numeric_limits<double>::max();
  for (size_t i = 0; i < num_points; i++) {
    Coord route_coord = route_points[i];
    double distance = get_distance(route_coord, starting_coord);
    if (distance < min_distance) {
      min_distance = distance;
      starting_route_index = i;
    }
  }
  spdlog::debug("Starting SOC: {}", max_soc);

  bool is_first_day = curr_time >= day_one_start_time && curr_time < day_one_end_time;

  // Keep track of the start and end time of the current day
  Time current_day_end(curr_time);
  Time next_day_start(curr_time);
  if (is_first_day) {
    current_day_end.copy_hh_mm_ss(day_one_end_time);
  } else {
    current_day_end.copy_hh_mm_ss(day_end_time);
  }
  next_day_start.copy_hh_mm_ss(day_start_time);
  // Advance the timestamp by 24 hours => 24 * 3600 seconds / hour = 86400 seconds
  next_day_start.update_time_seconds(86400);
  CarUpdate car_update;

  const size_t num_loops = segments.size();
  for (size_t loop_idx = 0; loop_idx < num_loops; loop_idx++) {
    const std::vector<std::pair<size_t, size_t>>& loop_segments = segments[loop_idx];
    const std::vector<std::pair<double, double>>& loop_segment_speeds = segment_speeds[loop_idx];
    const std::vector<bool>& loop_segment_acceleration = acceleration_segments[loop_idx];
    const std::vector<double>& loop_segment_acceleration_values = acceleration_values[loop_idx];

    size_t segment_counter = 0;
    std::pair<size_t, size_t> current_segment = loop_segments[segment_counter];
    std::pair<double, double> segment_speeds = loop_segment_speeds[segment_counter];
    is_accelerating = loop_segment_acceleration[segment_counter];
    acceleration = loop_segment_acceleration_values[segment_counter];
    curr_speed = segment_speeds.first;

    for (size_t idx=starting_route_index; idx < num_points; idx++) {
      const size_t coord_one = idx;
      const size_t coord_two = idx == num_points - 1 ? 0 : idx + 1;
      current_coord = route_points[coord_one];
      next_coord = route_points[coord_two];
      delta_energy = 0.0;

      // Update segment counter. The only time when the ending index is 0 i.e. current_segment.second == 0
      // is the last segment when the loop wraps around
      if (idx > current_segment.second && current_segment.second != 0) {
        segment_counter++;
        current_segment = loop_segments[segment_counter];
        segment_speeds = loop_segment_speeds[segment_counter];
        is_accelerating = loop_segment_acceleration[segment_counter];
        acceleration = loop_segment_acceleration_values[segment_counter];
        curr_speed = segment_speeds.first;
      }

      /* Get forecasting data - wind and irradiance */
      ForecastCoord coord_one_forecast(current_coord.lat, current_coord.lon);

      wind_speed_lut->update_index_cache(coord_one_forecast, curr_time.get_utc_time_point());
      double wind_speed = wind_speed_lut->get_value_with_cache();

      wind_dir_lut->update_index_cache(coord_one_forecast, curr_time.get_utc_time_point());
      double wind_dir = wind_dir_lut->get_value_with_cache();

      dni_lut->update_index_cache(coord_one_forecast, curr_time.get_utc_time_point());
      double dni = dni_lut->get_value_with_cache();

      dhi_lut->update_index_cache(coord_one_forecast, curr_time.get_utc_time_point());
      double dhi = dhi_lut->get_value_with_cache();

      Wind wind = Wind(wind_dir, wind_speed);
      Irradiance irr = Irradiance(dni, dhi);

      /** @brief Lambda function for charging between the start and end time
       * @param start_time: The starting time of the charging period. This will be modified
       * to eventually match the timestamp of end_time
       * @param end_time: The ending time of the charging period
       */
      auto charge_in_time_interval = [&](Time& start_time, const Time& end_time) {
          while (start_time < end_time) {
            /* Step in 30-second intervals */
            SolarAngle sun = SolarAngle();
            get_az_el(start_time.get_utc_time_point(), charging_coord.lat, charging_coord.lon, charging_coord.alt,
                      &sun.Az, &sun.El);

            double step_size = (start_time + static_cast<double>(CHARGING_STEP_SIZE)) >= end_time ?
                                end_time - start_time : static_cast<double>(CHARGING_STEP_SIZE);
            if (sun.El > 0) {
                delta_energy += car->compute_static_energy(current_coord, &start_time, step_size, irr);

                start_time.update_time_seconds(step_size);
                dni_lut->update_index_cache(coord_one_forecast, start_time.get_utc_time_point());
                irr.dni = dni_lut->get_value_with_cache();

                dhi_lut->update_index_cache(coord_one_forecast, start_time.get_utc_time_point());
                irr.dhi = dhi_lut->get_value_with_cache();
            } else {
                start_time.update_time_seconds(step_size);
            }
          }
      };
      /* Overnight stop */
      if (curr_time >= day_end_time) {
        // Charge from EoD to impounding start time
        charge_in_time_interval(curr_time, impounding_start_time);

        // Move curr_time to the next day's impounding release time
        curr_time = next_day_start;
        curr_time.copy_hh_mm_ss(impounding_release_time);

        // Charge from impounding release time to the race start
        charge_in_time_interval(curr_time, next_day_start);

        // Update the EoD and next day start timestamps
        current_day_end = curr_time;
        current_day_end.copy_hh_mm_ss(day_end_time);

        next_day_start = curr_time;
        next_day_start.copy_hh_mm_ss(day_start_time);
        next_day_start.update_time_seconds(86400);
      }

      /* Compute state update of the car */
      try {
        car_update = car->compute_travel_update(current_coord, next_coord, curr_speed, acceleration,
                                                      &curr_time, wind, irr);
      } catch (const InvalidCalculation& e) {
        // Discriminant is negative. Some deceleration/acceleration cannot be completed and this race plan
        // should be thrown out
        race_plan->set_viability(false);
        race_plan->set_inviability_reason(std::string(e.what()));
        return;
      }

      /* Update the running state of the simulation */
      delta_energy += car_update.delta_energy;
      accumulated_distance += car_update.delta_distance;
      curr_time.update_time_seconds(car_update.delta_time);
      curr_speed = calc_final_speed(curr_speed, acceleration, car_update.delta_time);

      /* Make sure the battery doesn't exceed the maximum bound */
      if (battery_energy + delta_energy > max_soc) {
        battery_energy = max_soc;
      } else {
        battery_energy += delta_energy;
      }
      spdlog::debug("Battery Energy: {}", battery_energy);

      /* Update the logs */
      results_lut->update_logs(car_update, battery_energy, delta_energy, accumulated_distance,
                              next_coord, curr_speed, curr_time, acceleration);

      /* Invalid simulation if battery goes below 0 or if the end of the race has been reached */
      if (battery_energy < 0.0) {
        race_plan->set_viability(false);
        std::cout << "No battery charge" << std::endl;
        race_plan->set_inviability_reason("Out of battery charge");
        return;
      }
    }
  }
  race_plan->set_viability(true);
}

void FSGPSimulator::reset_vars() {
  max_soc = Config::get_instance()->get_max_soc();
  starting_coord = Config::get_instance()->get_gps_coordinates();
  battery_energy = Config::get_instance()->get_current_soc();
  curr_time = Config::get_instance()->get_current_date_time();
  wind_speed_lut->initialize_caches(starting_coord, curr_time.get_utc_time_point());
  wind_dir_lut->initialize_caches(starting_coord, curr_time.get_utc_time_point());
  dni_lut->initialize_caches(starting_coord, curr_time.get_utc_time_point());
  dhi_lut->initialize_caches(starting_coord, curr_time.get_utc_time_point());
  accumulated_distance = 0.0;
  impounding_start_time = Config::get_instance()->get_impounding_start_time();
  impounding_release_time = Config::get_instance()->get_impounding_release_time();
}

FSGPSimulator::FSGPSimulator(std::shared_ptr<Car> model) :
  charging_coord(Config::get_instance()->get_overnight_charging_location()),
  impounding_start_time(Config::get_instance()->get_impounding_start_time()),
  impounding_release_time(Config::get_instance()->get_impounding_release_time()),
  WSCSimulator(model) {}
