#include <stdlib.h>
#include <assert.h>
#include <unordered_set>
#include <string>
#include <iostream>
#include <fstream>
#include <memory>
#include <limits>
#include <vector>
#include <utility>

#include "spdlog/spdlog.h"
#include "sim/Simulator.hpp"
#include "utils/Geography.hpp"
#include "utils/Defines.hpp"

void Simulator::run_sim(const std::unique_ptr<Route>& route, RacePlan* race_plan) {
  RUNTIME_EXCEPTION(route != nullptr, "Route pointer is null");
  RUNTIME_EXCEPTION(results_lut != nullptr, "Results lut is null");

  /* Reset variables */
  reset_vars();
  results_lut->reset_logs();

  /* Get route data */
  const size_t num_points = route->get_num_points();
  const std::vector<Coord> route_points = route->get_route_points();
  const std::vector<std::pair<size_t, size_t>> segments = race_plan->get_segments();
  const std::unordered_set<size_t> control_stops = route->get_control_stops();
  const std::vector<double> speed_profile_kph = race_plan->get_speed_profile();

  size_t segment_counter = 0;
  std::pair<size_t, size_t> current_segment = segments[segment_counter];
  curr_speed = kph2mps(speed_profile_kph[segment_counter]);

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

  bool is_first_day = Config::get_instance()->get_first_day();
  Time race_start_time(curr_time);
  for (size_t idx=starting_route_index; idx < num_points-1; idx++) {
    current_coord = route_points[idx];
    next_coord = route_points[idx+1];
    delta_energy = 0.0;

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

    /* Overnight stop */
    while (curr_time > race_end || (!is_first_day && curr_time < race_start)) {
      is_first_day = false;
      /* Step in 30 second intervals */
      SolarAngle sun = SolarAngle();
      get_az_el(curr_time.get_utc_time_point(), current_coord.lat, current_coord.lon,
                current_coord.alt, &sun.Az, &sun.El);

      if (sun.El > 0) {
        /* Charging at dawn/dusk */
        delta_energy += car->compute_static_energy(current_coord, &curr_time, OVERNIGHT_STEP_SIZE, irr);
        curr_time.update_time_seconds(OVERNIGHT_STEP_SIZE);
        dni_lut->update_index_cache(coord_one_forecast, curr_time.get_utc_time_point());
        double dni = dni_lut->get_value_with_cache();

        dhi_lut->update_index_cache(coord_one_forecast, curr_time.get_utc_time_point());
        double dhi = dhi_lut->get_value_with_cache();
        irr = Irradiance(dni, dhi);
      } else {
        curr_time.update_time_seconds(OVERNIGHT_STEP_SIZE);
      }
    }

    /* Control Stop */
    if (control_stops.find(idx) != control_stops.end()) {
      delta_energy += car->compute_static_energy(current_coord, &curr_time, control_stop_charge_time, irr);
      curr_time.update_time_seconds(control_stop_charge_time);
    }

    /* Move from point one to point two */
    if (idx > current_segment.second) {
      segment_counter++;
      current_segment = segments[segment_counter];
      curr_speed = kph2mps(speed_profile_kph[segment_counter]);
    }

    /* Compute state update of the car */
    CarUpdate update = car->compute_travel_update(current_coord, next_coord, curr_speed, &curr_time, wind, irr);

    /* Update the running state of the simulation */
    delta_energy += update.delta_energy;
    accumulated_distance += update.delta_distance;
    curr_time.update_time_seconds(update.delta_time);

    /* Make sure the battery doesn't exceed the maximum bound */
    if (battery_energy + delta_energy > max_soc) {
      battery_energy = max_soc;
    } else {
      battery_energy += delta_energy;
    }
    spdlog::debug("Battery Energy: {}", battery_energy);

    /* Update the logs */
    results_lut->update_logs(update, battery_energy, delta_energy, accumulated_distance,
                             next_coord, curr_speed, curr_time);

    /* Invalid simulation if battery goes below 0 or if the end of the race has been reached */
    if (battery_energy < 0.0 || curr_time > race_end_time) {
      race_plan->set_viability(false);
      return;
    }
  }
  race_plan->set_viability(true);
  race_plan->set_time_taken(curr_time.get_local_time_point() - race_start_time.get_local_time_point());
}

void Simulator::reset_vars() {
  max_soc = Config::get_instance()->get_max_soc();
  starting_coord = Config::get_instance()->get_gps_coordinates();
  battery_energy = max_soc;
  curr_time = Config::get_instance()->get_current_date_time();
  wind_speed_lut->initialize_caches(starting_coord, curr_time.get_utc_time_point());
  wind_dir_lut->initialize_caches(starting_coord, curr_time.get_utc_time_point());
  dni_lut->initialize_caches(starting_coord, curr_time.get_utc_time_point());
  dhi_lut->initialize_caches(starting_coord, curr_time.get_utc_time_point());
  accumulated_distance = 0.0;
}

void Simulator::write_result(std::string csv_path) {
  results_lut->write_logs(csv_path);
  spdlog::info("Simulation data saved");
}

ResultsLut Simulator::get_results_lut() const {
  if (!results_lut) return ResultsLut();
  return *(results_lut.get());
}

Simulator::Simulator(std::unique_ptr<Car> model) :
  car(std::move(model)),
  results_lut(std::make_unique<ResultsLut>()),
  wind_speed_lut(std::make_unique<ForecastLut>(Config::get_instance()->get_wind_speed_path())),
  wind_dir_lut(std::make_unique<ForecastLut>(Config::get_instance()->get_wind_direction_path())),
  dni_lut(std::make_unique<ForecastLut>(Config::get_instance()->get_dni_path())),
  dhi_lut(std::make_unique<ForecastLut>(Config::get_instance()->get_dhi_path())),
  control_stop_charge_time(mins2secs(Config::get_instance()->get_control_stop_charge_time())),
  race_start(Config::get_instance()->get_day_start_time()),
  race_end(Config::get_instance()->get_day_end_time()),
  starting_coord(Config::get_instance()->get_gps_coordinates()),
  curr_time(Config::get_instance()->get_current_date_time()),
  race_end_time(Config::get_instance()->get_race_end_time()) {}
