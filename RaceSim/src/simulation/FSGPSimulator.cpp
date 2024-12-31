#include <memory>
#include <iostream>
#include "sim/FSGPSimulator.hpp"

void FSGPSimulator::run_sim(const std::shared_ptr<Route>& route, RacePlan* race_plan) {
  std::cout << "Running FSGP Simulation" << std::endl;
  // RUNTIME_EXCEPTION(route != nullptr, "Route pointer is null");
  // RUNTIME_EXCEPTION(results_lut != nullptr, "Results lut is null");
  // RUNTIME_EXCEPTION(race_plan != nullptr, "Race plan is null");

  // /* Reset variables */
  // reset_vars();
  // results_lut->reset_logs();

  // /* Get route data */
  // const size_t num_points = route->get_num_points();
  // const std::vector<Coord> route_points = route->get_route_points();
  // const std::unordered_set<size_t> control_stops = route->get_control_stops();

  // /* Get race plan data */
  // RUNTIME_EXCEPTION(race_plan->validate_members(), "Race Plan is invalid");
  // const std::vector<std::vector<std::pair<size_t, size_t>>> segments = race_plan->get_segments();
  // const std::vector<std::vector<std::pair<double, double>>> segment_speeds = race_plan->get_speed_profile();
  // const std::vector<std::vector<bool>> acceleration_segments = race_plan->get_acceleration_segments();
  // const std::vector<std::vector<double>> acceleration_values = race_plan->get_acceleration_values();

  // /* Get starting position in the route */
  // size_t starting_route_index = 0;
  // double min_distance = std::numeric_limits<double>::max();
  // for (size_t i = 0; i < num_points; i++) {
  //   Coord route_coord = route_points[i];
  //   double distance = get_distance(route_coord, starting_coord);
  //   if (distance < min_distance) {
  //     min_distance = distance;
  //     starting_route_index = i;
  //   }
  // }
  // spdlog::debug("Starting SOC: {}", max_soc);

  // Time sim_start_time(curr_time);  // Remember starting time to calculate total elapsed time
  // bool is_first_day = sim_start_time >= race_day_one_start_time && sim_start_time < race_day_one_end_time;

  // const size_t num_loops = segments.size();
  // for (size_t loop_idx = 0; loop_idx < num_loops; loop_idx++) {
  //   const std::vector<std::pair<size_t, size_t>>& loop_segments = segments[loop_idx];
  //   const std::vector<std::pair<double, double>>& loop_segment_speeds = segment_speeds[loop_idx];
  //   const std::vector<bool>& loop_segment_acceleration = acceleration_segments[loop_idx];
  //   const std::vector<double>& loop_segment_acceleration_values = accleeration_values[loop_idx];
  //   size_t segment_counter = 0;
  //   std::pair<size_t, size_t> current_segment = loop_segments[segment_counter];
  //   std::pair<double, double> segment_speeds = loop_segment_speeds[segment_counter];
  //   is_accelerating = loop_segment_acceleration[segment_counter];
  //   acceleration = loop_segment_acceleration_values[segment_counter];

  //   for (size_t idx=starting_route_index; idx < num_points-1; idx++) {
  //     current_coord = route_points[idx];
  //     next_coord = route_points[idx+1];
  //     delta_energy = 0.0;

  //     // Update segment counter
  //     if (idx > current_segment.second) {
  //       segment_counter++;
  //       current_segment = loop_segments[segment_counter];
  //       segment_speeds = loop_segment_speeds[segment_counter];
  //       is_accelerating = loop_segment_acceleration[segment_counter];
  //       acceleration = loop_segment_acceleration_values[segment_counter];
  //     }

  //     /* Get forecasting data - wind and irradiance */
  //     ForecastCoord coord_one_forecast(current_coord.lat, current_coord.lon);

  //     wind_speed_lut->update_index_cache(coord_one_forecast, curr_time.get_utc_time_point());
  //     double wind_speed = wind_speed_lut->get_value_with_cache();

  //     wind_dir_lut->update_index_cache(coord_one_forecast, curr_time.get_utc_time_point());
  //     double wind_dir = wind_dir_lut->get_value_with_cache();

  //     dni_lut->update_index_cache(coord_one_forecast, curr_time.get_utc_time_point());
  //     double dni = dni_lut->get_value_with_cache();

  //     dhi_lut->update_index_cache(coord_one_forecast, curr_time.get_utc_time_point());
  //     double dhi = dhi_lut->get_value_with_cache();

  //     Wind wind = Wind(wind_dir, wind_speed);
  //     Irradiance irr = Irradiance(dni, dhi);

  //     /* Overnight stop */
  //     if ((is_first_day && curr_time > day_one_end_time)) {
  //       is_first_day = false;
  //       // Charge from EoD to impounding start time
  //       while (curr_time < impounding_start_time) {
  //         /* Step in 30 second intervals */
  //         SolarAngle sun = SolarAngle();
  //         get_az_el(curr_time.get_utc_time_point(), charging_coord.lat, charging_coord.lon, charging_coord.alt,
  //                   &sun.Az, &sun.El);

  //         if (sun.El > 0) {
  //           delta_energy += car->compute_static_energy(charging_coord, &curr_time, OVERNIGHT_STEP_SIZE, irr);

  //           curr_time.update_time_seconds(OVERNIGHT_STEP_SIZE);
  //           dni_lut->update_index_cache(coord_one_forecast, curr_time.get_utc_time_point());
  //           irr.dni = dni_lut->get_value_with_cache();

  //           dhi_lut->update_index_cache(coord_one_forecast, curr_time.get_utc_time_point());
  //           irr.dhi = dhi_lut->get_value_with_cache();
  //         } else {
  //           curr_time.update_time_seconds(OVERNIGHT_STEP_SIZE);
  //         }
  //       }

  //       // Advance time to the next day's battery impounding release time
  //       curr_time.update_time_seconds()

  //       // Charge from impounding release time to start of day
  //       while
  //     }
  //     while ((is_first_day && curr_time > race_day_one_end_time && curr_time < race_day_start) ||
  //            (curr_time > race_day_end && curr_time < race_day_start)) {
  //       is_first_day = false;
  //       /* Step in 30 second intervals */
  //       SolarAngle sun = SolarAngle();
  //       get_az_el(curr_time.get_utc_time_point(), current_coord.lat, current_coord.lon,
  //                 current_coord.alt, &sun.Az, &sun.El);

  //       if (sun.El > 0) {
  //         /* Charging at dawn/dusk */
  //         delta_energy += car->compute_static_energy(current_coord, &curr_time, OVERNIGHT_STEP_SIZE, irr);
  //         curr_time.update_time_seconds(OVERNIGHT_STEP_SIZE);
  //         dni_lut->update_index_cache(coord_one_forecast, curr_time.get_utc_time_point());
  //         double dni = dni_lut->get_value_with_cache();

  //         dhi_lut->update_index_cache(coord_one_forecast, curr_time.get_utc_time_point());
  //         double dhi = dhi_lut->get_value_with_cache();
  //         irr = Irradiance(dni, dhi);
  //       } else {
  //         curr_time.update_time_seconds(OVERNIGHT_STEP_SIZE);
  //       }
  //     }

  //     /* Control Stop */
  //     if (control_stops.find(idx) != control_stops.end()) {
  //       delta_energy += car->compute_static_energy(current_coord, &curr_time, control_stop_charge_time, irr);
  //       curr_time.update_time_seconds(control_stop_charge_time);
  //     }

  //     /* Move from point one to point two */
  //     if (idx > current_segment.second) {
  //       segment_counter++;
  //       current_segment = segments[segment_counter];
  //       curr_speed = kph2mps(speed_profile_kph[segment_counter]);
  //     }

  //     /* Compute state update of the car */
  //     CarUpdate update = car->compute_travel_update(current_coord, next_coord, curr_speed, acceleration,
  //                                                   &curr_time, wind, irr);

  //     /* Update the running state of the simulation */
  //     delta_energy += update.delta_energy;
  //     accumulated_distance += update.delta_distance;
  //     curr_time.update_time_seconds(update.delta_time);

  //     /* Make sure the battery doesn't exceed the maximum bound */
  //     if (battery_energy + delta_energy > max_soc) {
  //       battery_energy = max_soc;
  //     } else {
  //       battery_energy += delta_energy;
  //     }
  //     spdlog::debug("Battery Energy: {}", battery_energy);

  //     /* Update the logs */
  //     results_lut->update_logs(update, battery_energy, delta_energy, accumulated_distance,
  //                             next_coord, curr_speed, curr_time, acceleration);

  //     /* Invalid simulation if battery goes below 0 or if the end of the race has been reached */
  //     if (battery_energy < 0.0 || curr_time > race_end_time) {
  //       race_plan->set_viability(false);
  //       return;
  //     }
  //   }
  // }
  // race_plan->set_viability(true);
  // race_plan->set_time_taken(curr_time.get_local_time_point() - race_start_time.get_local_time_point());
}

FSGPSimulator::FSGPSimulator(std::shared_ptr<Car> model) :
  charging_coord(Config::get_instance()->get_overnight_charging_location()),
  impounding_start_time(Config::get_instance()->get_impounding_start_time()),
  impounding_release_time(Config::get_instance()->get_impounding_release_time()),
  WSCSimulator(model) {}
