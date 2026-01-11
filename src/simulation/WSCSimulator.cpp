#include "sim/WSCSimulator.hpp"

#include <cstddef>
#include <limits>
#include <unordered_set>
#include <utility>

#include "SimUtils/CustomException.hpp"
#include "SimUtils/Defines.hpp"
#include "SimUtils/Geography.hpp"
#include "SimUtils/Luts.hpp"
#include "SimUtils/Types.hpp"
#include "model/Car.hpp"
#include "route/RacePlan.hpp"
#include "route/Route.hpp"
#include "sim/Simulator.hpp"
#include "spdlog/spdlog.h"  // IWYU pragma: keep

void WSCSimulator::run_sim_impl(const WSCRoute& route, WSCRacePlan* race_plan,
                                Luts::DataSet* results_lut) const {
  RUNTIME_EXCEPTION(results_lut != nullptr, "Results lut is null");
  RUNTIME_EXCEPTION(race_plan != nullptr, "Race plan is null");
  RUNTIME_EXCEPTION(race_plan->validate_members(route),
                    "Race Plan is improperly created");
  RUNTIME_EXCEPTION(!params.wind_speed_lut.is_empty(),
                    "Wind speed lut must be loaded");
  RUNTIME_EXCEPTION(!params.wind_dir_lut.is_empty(),
                    "Wind direction lut must be loaded");
  RUNTIME_EXCEPTION(!params.dni_lut.is_empty(), "DNI lut must be loaded");
  RUNTIME_EXCEPTION(!params.dhi_lut.is_empty(), "DHI Lut must be loaded");

  race_plan->set_start_time(params.sim_start_time);
  const util::type::ForecastCoord sim_start_forecast_coord(
      params.sim_start_coord);
  std::pair<size_t, size_t> dni_cache = params.dni_lut.initialize_caches(
      sim_start_forecast_coord, params.sim_start_time);
  std::pair<size_t, size_t> dhi_cache = params.dhi_lut.initialize_caches(
      sim_start_forecast_coord, params.sim_start_time);
  std::pair<size_t, size_t> wind_speed_cache =
      params.wind_speed_lut.initialize_caches(sim_start_forecast_coord,
                                              params.sim_start_time);
  std::pair<size_t, size_t> wind_dir_cache =
      params.wind_dir_lut.initialize_caches(sim_start_forecast_coord,
                                            params.sim_start_time);
  util::type::Irradiance irr;
  util::type::Wind wind;
  util::type::SolarAngle sun;

  // Initialize simulation state variables
  double accumulated_distance = 0.0;  // In meters
  double driving_time = 0.0;          // In seconds
  double battery_energy = params.sim_start_soc;
  const util::type::Coord starting_coord = params.sim_start_coord;
  util::type::Time curr_time = params.sim_start_time;
  double curr_speed = 0.0;
  LogMetrics metrics;

  /* Get route and race plan data */
  const size_t num_points = route.get_num_points();
  const CoordVec& route_points = route.get_route_points();
  const std::unordered_set<int>& control_stops = route.get_control_stops();
  const WSCRacePlan::SegmentsVec& segments = race_plan->get_segments();

  /* Get starting position in the route */
  size_t starting_route_index = 0;
  double min_distance = std::numeric_limits<double>::max();
  for (size_t i = 0; i < num_points; i++) {
    const util::type::Coord route_coord = route_points[i];
    const double distance =
        util::geo::get_distance(route_coord, starting_coord);
    if (distance < min_distance) {
      min_distance = distance;
      starting_route_index = i;
    }
  }
  spdlog::debug("Starting SOC: {}", car.get_max_soc());

  const bool is_first_day = curr_time >= params.day_one_start_time &&
                            curr_time < params.day_one_end_time;
  // End of day time
  util::type::Time current_day_end;
  if (is_first_day) {
    current_day_end = util::type::Time::combine_date_and_time(
        curr_time, params.day_one_end_time);
  } else {
    current_day_end =
        util::type::Time::combine_date_and_time(curr_time, params.day_end_time);
  }

  // Start of next day
  util::type::Time next_day_start =
      util::type::Time::combine_date_and_time(curr_time, params.day_start_time);
  next_day_start += SECONDS_IN_DAY;

  size_t segment_counter = 0;
  BaseSegment current_segment = segments[segment_counter];
  curr_speed = current_segment.start_speed;
  RUNTIME_EXCEPTION(current_segment.acceleration_value == 0.0,
                    "WSC Simulator does not currently simulate acceleration");

  /** Update the irradiance variable - passed in by reference */
  auto update_irradiance = [&](const util::type::ForecastCoord& coord,
                               const util::type::Time& curr_time,
                               util::type::Irradiance& irr) {
    irr.dni = params.dni_lut.get_value_and_update_cache(
        coord, curr_time, dni_cache.first, dni_cache.second);
    irr.dhi = params.dhi_lut.get_value_and_update_cache(
        coord, curr_time, dhi_cache.first, dhi_cache.second);
  };

  CarUpdate update;
  for (size_t idx = starting_route_index; idx < num_points - 1; idx++) {
    const util::type::Coord& current_coord = route_points[idx];
    const util::type::Coord& next_coord = route_points[idx + 1];
    double delta_energy = 0.0;

    // Update route segment
    if (idx > current_segment.end_idx) {
      segment_counter++;
      current_segment = segments[segment_counter];
      curr_speed = current_segment.start_speed;
      RUNTIME_EXCEPTION(
          current_segment.acceleration_value == 0.0,
          "WSC Simulator does not currently simulate acceleration");
    }

    /* Update forecast lut index caches and get forecast data at coordinate 1 */
    const util::type::ForecastCoord coord_one_forecast(current_coord.lat,
                                                       current_coord.lon);
    wind.speed = params.wind_speed_lut.get_value_and_update_cache(
        coord_one_forecast, curr_time, wind_speed_cache.first,
        wind_speed_cache.second);

    wind.bearing = params.wind_dir_lut.get_value_and_update_cache(
        coord_one_forecast, curr_time, wind_dir_cache.first,
        wind_dir_cache.second);
    update_irradiance(coord_one_forecast, curr_time, irr);

    // Check if the car arrived at a control stop
    double overflowed_control_stop_time = 0.0;
    if (control_stops.contains(static_cast<int>(idx))) {
      // If the control stop spills into the next day, we need to calculate the
      // stop in two stages
      // 1. curr_time -> EoD
      // 2. Next day start -> time remaining
      // We first charge stage 1
      double time_charging = 0.0;
      while (curr_time < current_day_end &&
             time_charging < params.control_stop_charge_time) {
        sun = util::geo::get_az_el(current_coord, curr_time);
        const double step_size =
            get_charging_step_size(curr_time, current_day_end);
        update_irradiance(coord_one_forecast, curr_time, irr);
        delta_energy += car.compute_static_energy(irr, sun, step_size);
        curr_time += step_size;

        time_charging = time_charging + step_size;
      }

      overflowed_control_stop_time =
          params.control_stop_charge_time - time_charging;
    }

    /* Overnight stop */
    if (curr_time >= current_day_end) {
      while (curr_time < next_day_start) {
        sun = util::geo::get_az_el(current_coord, curr_time);
        const double step_size =
            get_charging_step_size(curr_time, next_day_start);
        if (sun.El > 0) {
          /* Charging at dawn/dusk */
          update_irradiance(coord_one_forecast, curr_time, irr);

          delta_energy += car.compute_static_energy(irr, sun, step_size);
          curr_time += step_size;

        } else {
          curr_time += step_size;
        }
      }

      // Adjust new day ends and next day start
      current_day_end = util::type::Time::combine_date_and_time(
          curr_time, params.day_end_time);
      next_day_start = util::type::Time::combine_date_and_time(
          curr_time, params.day_start_time);
      next_day_start += SECONDS_IN_DAY;
    }

    /* Overflowed control stop */
    double time_charging = 0.0;
    while (time_charging < overflowed_control_stop_time) {
      const double step_size =
          (time_charging + static_cast<double>(CHARGING_STEP_SIZE)) >
                  overflowed_control_stop_time
              ? overflowed_control_stop_time - time_charging
              : static_cast<double>(CHARGING_STEP_SIZE);
      update_irradiance(coord_one_forecast, curr_time, irr);
      delta_energy += car.compute_static_energy(irr, sun, step_size);
      curr_time += step_size;

      time_charging += step_size;
    }

    /* Compute state update of the car */
    try {
      update = car.compute_constant_travel_update(
          current_coord, next_coord, curr_speed, curr_time, wind, irr);
    } catch (const util::error::InvalidCalculation&) {
      // Maximum motor power exceeded
      race_plan->set_viability(false);
      race_plan->set_inviability_reason("Maximum motor power exceeded");
      metrics.register_dataset(results_lut);
      return;
    }

    /* Update the running state of the simulation */
    delta_energy += update.delta_energy;
    accumulated_distance += update.delta_distance;
    curr_time += update.delta_time;
    driving_time += update.delta_time;

    /* Make sure the battery doesn't exceed the maximum bound */
    if (battery_energy + delta_energy > car.get_max_soc()) {
      battery_energy = car.get_max_soc();
    } else {
      battery_energy += delta_energy;
    }
    spdlog::debug("Battery Energy: {}", battery_energy);

    /* Update the logs */
    metrics.update_metrics(update, irr, battery_energy, delta_energy,
                           accumulated_distance, next_coord, curr_speed,
                           curr_time, 0.0 /* No acceleration */);

    /* Invalid simulation if battery goes below 0 or if the end of the race has
     * been reached */
    if (battery_energy < 0.0 || curr_time > params.race_end_time) {
      race_plan->set_viability(false);
      if (battery_energy < 0.0) {
        race_plan->set_inviability_reason("Out of battery");
      } else {
        race_plan->set_inviability_reason(
            "Did not make it to the finish line in time");
      }
      metrics.register_dataset(results_lut);
      return;
    }
  }
  race_plan->set_viability(true);
  race_plan->set_driving_time(driving_time);
  race_plan->set_average_speed(accumulated_distance / driving_time);
  race_plan->set_end_time(curr_time);
  metrics.register_dataset(results_lut);
}

WSCSimulator::WSCSimulator(WSCSimulatorParams params, Car model)
    : Simulator<WSCSimulator, WSCRacePlan, WSCRoute>(std::move(model)),
      params(std::move(params)) {}
