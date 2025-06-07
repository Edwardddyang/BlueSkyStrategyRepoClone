#include <memory>
#include <vector>
#include <unordered_set>
#include <limits>
#include <utility>

#include "sim/WSCSimulator.hpp"
#include "config/Config.hpp"

void WSCSimulator::run_sim(const std::shared_ptr<Route>& route, RacePlan* race_plan,
                           std::shared_ptr<ResultsLut> results_lut) {
  RUNTIME_EXCEPTION(route != nullptr, "Route pointer is null");
  RUNTIME_EXCEPTION(results_lut != nullptr, "Results lut is null");
  RUNTIME_EXCEPTION(race_plan != nullptr, "Race plan is null");
  RUNTIME_EXCEPTION(race_plan->validate_members(route->get_route_points()), "Race Plan is improperly created");
  RUNTIME_EXCEPTION(race_plan->get_segments().size() == 1, "Race Plan should have only one loop for WSC simulator");
  RUNTIME_EXCEPTION(control_stop_charge_time % CHARGING_STEP_SIZE == 0, "Control stop charge time must be divisible by "
                    "charging step size");
  RUNTIME_EXCEPTION(wind_speed_lut != nullptr, "Wind speed lut must be loaded");
  RUNTIME_EXCEPTION(wind_dir_lut != nullptr, "Wind direction lut must be loaded");
  RUNTIME_EXCEPTION(dni_lut != nullptr, "DNI lut must be loaded");
  RUNTIME_EXCEPTION(dhi_lut != nullptr, "DHI Lut must be loaded");

  // Reset results lut logs
  results_lut->reset_logs();

  // Initialize simulation state variables
  double accumulated_distance = 0.0;
  Time curr_time = this->sim_start_time;
  double battery_energy = this->sim_start_soc;
  Coord starting_coord = this->sim_start_coord;
  // Initialize index caches for forecast lut lookups
  std::pair<size_t, size_t> wind_speed_cache = wind_speed_lut->initialize_caches(starting_coord,
                                                                                 curr_time.get_utc_time_point());
  std::pair<size_t, size_t> wind_dir_cache = wind_dir_lut->initialize_caches(starting_coord,
                                                                             curr_time.get_utc_time_point());
  std::pair<size_t, size_t> dni_cache = dni_lut->initialize_caches(starting_coord,
                                                                   curr_time.get_utc_time_point());
  std::pair<size_t, size_t> dhi_cache = dhi_lut->initialize_caches(starting_coord,
                                                                   curr_time.get_utc_time_point());
  double delta_energy;
  double curr_speed;
  bool is_accelerating;
  double acceleration;

  /* Get route and race plan data */
  const size_t num_points = route->get_num_points();
  const std::vector<Coord>& route_points = route->get_route_points();
  const std::vector<RacePlan::SegmentData> segments = race_plan->get_segments()[0];
  const std::unordered_set<size_t> control_stops = route->get_control_stops();

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
  // Keep track of the end of day time
  Time current_day_end(curr_time);
  Time next_day_start(curr_time);
  if (is_first_day) {
    current_day_end.copy_hh_mm_ss(day_one_end_time);
  } else {
    current_day_end.copy_hh_mm_ss(day_end_time);
  }
  next_day_start.copy_hh_mm_ss(day_start_time);
  // Advance the timestamp by 24 hours => 24 * 3600 seconds / hour = 86400 seconds
  next_day_start.update_time_seconds(SECONDS_IN_DAY);

  Time race_start_time(curr_time);  // Save starting time to calculate total elapsed time
  size_t segment_counter = 0;
  RacePlan::SegmentData current_segment = segments[segment_counter];
  curr_speed = kph2mps(current_segment.start_speed);

  for (size_t idx=starting_route_index; idx < num_points-1; idx++) {
    const Coord& current_coord = route_points[idx];
    const Coord& next_coord = route_points[idx+1];
    delta_energy = 0.0;
    // Update route segment
    if (idx > current_segment.end_idx) {
      segment_counter++;
      current_segment = segments[segment_counter];
      curr_speed = kph2mps(current_segment.start_speed);
    }

    /* Update forecast lut index caches and get forecast data at coordinate 1 */
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
    SolarAngle sun = SolarAngle();

    // Control stop
    double overflowed_control_stop_time = 0.0;
    if (control_stops.find(idx) != control_stops.end()) {
      // If the control stop spills into the next day, we need to calculate the stop in two stages
      // 1. curr_time -> EoD
      // 2. Next day start -> time remaining
      // We first charge stage 1
      const double time_until_eod = current_day_end - curr_time;
      double time_charging = 0.0;
      while (curr_time < current_day_end && time_charging < control_stop_charge_time) {
        get_az_el(curr_time.get_utc_time_point(), current_coord.lat, current_coord.lon,
                  current_coord.alt, &sun.Az, &sun.El);
        double step_size = (curr_time + static_cast<double>(CHARGING_STEP_SIZE)) >= current_day_end ?
                            current_day_end - curr_time : static_cast<double>(CHARGING_STEP_SIZE);
        delta_energy += car->compute_static_energy(current_coord, &curr_time, step_size, irr);
        curr_time.update_time_seconds(step_size);

        dni_lut->update_index_cache(&dni_cache, coord_one_forecast, curr_time.get_utc_time_point());
        irr.dni = dni_lut->get_value(dni_cache);

        dhi_lut->update_index_cache(&dhi_cache, coord_one_forecast, curr_time.get_utc_time_point());
        irr.dhi = dhi_lut->get_value(dhi_cache);

        time_charging = time_charging + step_size;
      }

      overflowed_control_stop_time = control_stop_charge_time - time_charging;
    }

    /* Overnight stop */
    if (curr_time >= current_day_end) {
      while (curr_time < next_day_start) {
        /* Step in 30 second intervals */
        get_az_el(curr_time.get_utc_time_point(), current_coord.lat, current_coord.lon,
                  current_coord.alt, &sun.Az, &sun.El);
        double step_size = (curr_time + static_cast<double>(CHARGING_STEP_SIZE)) >= next_day_start ?
                            next_day_start - curr_time : static_cast<double>(CHARGING_STEP_SIZE);
        if (sun.El > 0) {
          /* Charging at dawn/dusk */
          delta_energy += car->compute_static_energy(current_coord, &curr_time, step_size, irr);
          curr_time.update_time_seconds(step_size);

          dni_lut->update_index_cache(&dni_cache, coord_one_forecast, curr_time.get_utc_time_point());
          irr.dni = dni_lut->get_value(dni_cache);

          dhi_lut->update_index_cache(&dhi_cache, coord_one_forecast, curr_time.get_utc_time_point());
          irr.dhi = dhi_lut->get_value(dhi_cache);
        } else {
          curr_time.update_time_seconds(step_size);
        }
      }
      current_day_end = curr_time;
      current_day_end.copy_hh_mm_ss(day_end_time);
      next_day_start = curr_time;
      next_day_start.copy_hh_mm_ss(day_start_time);
      next_day_start.update_time_seconds(SECONDS_IN_DAY);
    }

    /* Overflowed control stop */
    double time_charging = 0.0;
    while (time_charging < overflowed_control_stop_time) {
      double step_size = (time_charging + static_cast<double>(CHARGING_STEP_SIZE)) >
                              overflowed_control_stop_time ?
                              overflowed_control_stop_time - time_charging :
                              static_cast<double>(CHARGING_STEP_SIZE);
      delta_energy += car->compute_static_energy(current_coord, &curr_time, step_size, irr);
      curr_time.update_time_seconds(step_size);

      dni_lut->update_index_cache(&dni_cache, coord_one_forecast, curr_time.get_utc_time_point());
      irr.dni = dni_lut->get_value(dni_cache);

      dhi_lut->update_index_cache(&dhi_cache, coord_one_forecast, curr_time.get_utc_time_point());
      irr.dhi = dhi_lut->get_value(dhi_cache);
      time_charging += step_size;
    }

    /* Compute state update of the car */
    CarUpdate update = car->compute_travel_update(current_coord, next_coord, curr_speed,
                                                  acceleration, &curr_time, wind, irr);

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
                             next_coord, curr_speed, curr_time, acceleration);

    /* Invalid simulation if battery goes below 0 or if the end of the race has been reached */
    if (battery_energy < 0.0 || curr_time > race_end_time) {
      race_plan->set_viability(false);
      return;
    }

    /* Update the speed if the car is accelerating */
    curr_speed = calc_final_speed(curr_speed, acceleration, update.delta_time);
  }
  race_plan->set_viability(true);
  race_plan->set_time_taken(curr_time.get_local_time_point() - race_start_time.get_local_time_point());
}

WSCSimulator::WSCSimulator(std::shared_ptr<Car> model) :
  control_stop_charge_time(mins2secs(Config::get_instance()->get_control_stop_charge_time())),
  sim_start_time(Config::get_instance()->get_current_date_time()),
  sim_start_soc(Config::get_instance()->get_current_soc()),
  day_one_start_time(Config::get_instance()->get_day_one_start_time()),
  day_one_end_time(Config::get_instance()->get_day_one_end_time()),
  day_start_time(Config::get_instance()->get_day_start_time()),
  day_end_time(Config::get_instance()->get_day_end_time()),
  race_end_time(Config::get_instance()->get_race_end_time()),
  sim_start_coord(Config::get_instance()->get_gps_coordinates()),
  max_soc(Config::get_instance()->get_max_soc()),
  Simulator(model) {}
