//
// ASC Simulator
//

#include <memory>
#include <vector>
#include <unordered_set>
#include <limits>
#include <utility>

#include "sim/ASCSimulator.hpp"
#include "config/ConfigParser.hpp"

void ASCSimulator::run_sim_impl(const ASCRoute& route, ASCRacePlan* race_plan,
                                Luts::DataSet* results_lut) {
  RUNTIME_EXCEPTION(results_lut != nullptr, "Results lut is null");
  RUNTIME_EXCEPTION(race_plan != nullptr, "Race plan is null");
  RUNTIME_EXCEPTION(race_plan->validate_members(route), "Race Plan is improperly created");
  RUNTIME_EXCEPTION(!params.wind_speed_lut.is_empty(), "Wind speed lut must be loaded");
  RUNTIME_EXCEPTION(!params.wind_dir_lut.is_empty(), "Wind direction lut must be loaded");
  RUNTIME_EXCEPTION(!params.dni_lut.is_empty(), "DNI lut must be loaded");
  RUNTIME_EXCEPTION(!params.dhi_lut.is_empty(), "DHI Lut must be loaded");

  util::type::ForecastCoord sim_start_forecast_coord(params.sim_start_coord);
  std::pair<size_t, size_t> dni_cache = params.dni_lut.initialize_caches(sim_start_forecast_coord, params.sim_start_time);
  std::pair<size_t, size_t> dhi_cache = params.dhi_lut.initialize_caches(sim_start_forecast_coord, params.sim_start_time);
  std::pair<size_t, size_t> wind_speed_cache = params.wind_speed_lut.initialize_caches(sim_start_forecast_coord, params.sim_start_time);
  std::pair<size_t, size_t> wind_dir_cache = params.wind_dir_lut.initialize_caches(sim_start_forecast_coord, params.sim_start_time);

  util::type::Irradiance irr;
  util::type::Wind wind;
  util::type::SolarAngle sun;

  race_plan->set_start_time(params.sim_start_time);
  // Reset results lut logs
  // results_lut.reset_logs();

  // Initialize simulation state variables
  double accumulated_distance = 0.0;
  double driving_time = 0.0;
  double battery_energy = params.sim_start_soc;
  util::type::Time curr_time = params.sim_start_time;
  util::type::Coord starting_coord = params.sim_start_coord;

  /** Update the irradiance variable - passed in by reference */
  auto update_irradiance = [&](const util::type::ForecastCoord& coord, const util::type::Time& curr_time, util::type::Irradiance& irr) {
    irr.dni = params.dni_lut.get_value_and_update_cache(coord, curr_time, dni_cache.first, dni_cache.second);
    irr.dhi = params.dhi_lut.get_value_and_update_cache(coord, curr_time, dhi_cache.first, dhi_cache.second);
  };

  // 1. Iterate through base legs
  // 2. Travel through loop when encountered

  // /* Get route and race plan data */
  // const size_t num_points = route->get_num_points();
  // const std::vector<Coord>& route_points = route->get_route_points();
  // const RacePlan::LoopData segments = race_plan->get_segments()[0];
  // const std::unordered_set<size_t> control_stops = route->get_control_stops();

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

  // Time current_day_end(curr_time);
  // Time next_day_start(curr_time);
  // current_day_end.copy_hh_mm_ss(day_end_time);
  // next_day_start.copy_hh_mm_ss(day_start_time);
  // next_day_start.update_time_seconds(SECONDS_IN_DAY);

  // Time race_start_time(curr_time);  // Save starting time to calculate total elapsed time
  // size_t segment_counter = 0;
  // RacePlan::SegmentData current_segment = segments[segment_counter];
  // curr_speed = kph2mps(current_segment.start_speed);

  // /** Update the irradiance variable - passed in by reference */
  // auto update_irradiance = [&](const ForecastCoord& coord, const Time& curr_time, Irradiance& irr) {
  //   double dni = 0.0;
  //   double dhi = 0.0;
  //   double ghi = 0.0;
  //   if (use_ghi) {
  //     ghi_lut.update_index_cache(&ghi_cache, coord, curr_time.get_utc_time_point());
  //     irr.ghi = ghi_lut.get_value(ghi_cache);
  //   } else {
  //     params.dni_lut.update_index_cache(&dni_cache, coord, curr_time.get_utc_time_point());
  //     irr.dni = params.dni_lut.get_value(dni_cache);

  //     params.dhi_lut.update_index_cache(&dhi_cache, coord, curr_time.get_utc_time_point());
  //     irr.dhi = params.dhi_lut.get_value(dhi_cache);
  //   }
  // };

  // for (size_t idx=starting_route_index; idx < num_points-1; idx++) {
  //   const Coord& current_coord = route_points[idx];
  //   const Coord& next_coord = route_points[idx+1];

  //   if (route->is_loop_start(current_coord)) {
  //     auto loop_points = route->get_loop_points(current_coord);
  //     // todo: process loop in simulation
  //   }

  //   delta_energy = 0.0;
  //   // Update route segment
  //   if (idx > current_segment.end_idx) {
  //     segment_counter++;
  //     current_segment = segments[segment_counter];
  //     curr_speed = kph2mps(current_segment.start_speed);
  //   }

  //   /* Update forecast lut index caches and get forecast data at coordinate 1 */
  //   ForecastCoord coord_one_forecast(current_coord.lat, current_coord.lon);

  //   params.wind_speed_lut.update_index_cache(&wind_speed_cache, coord_one_forecast, curr_time.get_utc_time_point());
  //   double wind_speed = params.wind_speed_lut.get_value(wind_speed_cache);

  //   params.wind_dir_lut.update_index_cache(&wind_dir_cache, coord_one_forecast, curr_time.get_utc_time_point());
  //   double wind_dir = params.wind_dir_lut.get_value(wind_dir_cache);

  //   update_irradiance(coord_one_forecast, curr_time, irr);
  //   Wind wind = Wind(wind_dir, wind_speed);
  //   SolarAngle sun = SolarAngle();

  //   // Control stop
  //   double overflowed_control_stop_time = 0.0;
  //   if (control_stops.find(idx) != control_stops.end()) {
  //     // If the control stop spills into the next day, we need to calculate the stop in two stages
  //     // 1. curr_time -> EoD
  //     // 2. Next day start -> time remaining
  //     // We first charge stage 1
  //     const double time_until_eod = current_day_end - curr_time;
  //     double time_charging = 0.0;
  //     while (curr_time < current_day_end && time_charging < checkpoint_hold_time) {
  //       get_az_el(curr_time.get_utc_time_point(), current_coord.lat, current_coord.lon,
  //                 current_coord.alt, &sun.Az, &sun.El);
  //       double step_size = (curr_time + static_cast<double>(CHARGING_STEP_SIZE)) >= current_day_end ?
  //                           current_day_end - curr_time : static_cast<double>(CHARGING_STEP_SIZE);
  //       delta_energy += car->compute_static_energy(current_coord, &curr_time, step_size, irr, "wsc");
  //       curr_time.update_time_seconds(step_size);

  //       update_irradiance(coord_one_forecast, curr_time, irr);

  //       time_charging = time_charging + step_size;
  //     }

  //     // overflowed_control_stop_time = control_stop_charge_time - time_charging;
  //   }

  //   /* Overnight stop */
  //   if (curr_time >= current_day_end) {
  //     while (curr_time < next_day_start) {
  //       /* Step in 30 second intervals */
  //       get_az_el(curr_time.get_utc_time_point(), current_coord.lat, current_coord.lon,
  //                 current_coord.alt, &sun.Az, &sun.El);
  //       double step_size = (curr_time + static_cast<double>(CHARGING_STEP_SIZE)) >= next_day_start ?
  //                           next_day_start - curr_time : static_cast<double>(CHARGING_STEP_SIZE);
  //       if (sun.El > 0) {
  //         /* Charging at dawn/dusk */
  //         delta_energy += car->compute_static_energy(current_coord, &curr_time, step_size, irr, "wsc");
  //         curr_time.update_time_seconds(step_size);

  //         update_irradiance(coord_one_forecast, curr_time, irr);
  //       } else {
  //         curr_time.update_time_seconds(step_size);
  //       }
  //     }
  //     current_day_end = curr_time;
  //     current_day_end.copy_hh_mm_ss(day_end_time);
  //     next_day_start = curr_time;
  //     next_day_start.copy_hh_mm_ss(day_start_time);
  //     next_day_start.update_time_seconds(SECONDS_IN_DAY);
  //   }

  //   /* Overflowed control stop */
  //   double time_charging = 0.0;
  //   while (time_charging < overflowed_control_stop_time) {
  //     double step_size = (time_charging + static_cast<double>(CHARGING_STEP_SIZE)) >
  //                             overflowed_control_stop_time ?
  //                             overflowed_control_stop_time - time_charging :
  //                             static_cast<double>(CHARGING_STEP_SIZE);
  //     delta_energy += car->compute_static_energy(current_coord, &curr_time, step_size, irr, "wsc");
  //     curr_time.update_time_seconds(step_size);
  //     update_irradiance(coord_one_forecast, curr_time, irr);

  //     time_charging += step_size;
  //   }

  //   /* Compute state update of the car */
  //   CarUpdate update = car->compute_travel_update(current_coord, next_coord, curr_speed,
  //                                                 acceleration, &curr_time, wind, irr);

  //   /* Update the running state of the simulation */
  //   delta_energy += update.delta_energy;
  //   accumulated_distance += update.delta_distance;
  //   curr_time.update_time_seconds(update.delta_time);

  //   /* Make sure the battery doesn't exceed the maximum bound */
  //   if (battery_energy + delta_energy > max_soc) {
  //     battery_energy = max_soc;
  //   } else {
  //     battery_energy += delta_energy;
  //   }
  //   spdlog::debug("Battery Energy: {}", battery_energy);

  //   /* Update the logs */
  //   results_lut.update_logs(update, irr, battery_energy, delta_energy, accumulated_distance,
  //                            next_coord, curr_speed, curr_time, acceleration);

  //   /* Invalid simulation if battery goes below 0 or if the end of the race has been reached */
  //   if (battery_energy < 0.0 || curr_time > race_end_time) {
  //     race_plan->set_viability(false);
  //     return;
  //   }

  //   /* Update the speed if the car is accelerating */
  //   curr_speed = calc_final_speed(curr_speed, acceleration, update.delta_time);
  // }
  // race_plan->set_viability(true);
  // race_plan->set_time_taken(curr_time.get_local_time_point() - race_start_time.get_local_time_point());
}

ASCSimulator::ASCSimulator(ASCSimulatorParams params, Car model) :
  Simulator<ASCSimulator, ASCRacePlan, ASCRoute>(model),
  params(std::move(params)) {}