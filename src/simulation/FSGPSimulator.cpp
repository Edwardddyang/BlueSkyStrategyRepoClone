#include "sim/FSGPSimulator.hpp"

#include <cstddef>
#include <string>
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

void FSGPSimulator::run_sim_impl(const FSGPRoute& route,
                                 FSGPRacePlan* race_plan,
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

  const PointsLut& route_distances = route.get_precomputed_distances();
  // Don't get speed limits since validation should ensure that speed limits
  // are not violated
  RUNTIME_EXCEPTION(!route_distances.is_empty(),
                    "FSGP Simulator must use pre-computed distances,"
                    "but no data was loaded");

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

  race_plan->set_start_time(params.sim_start_time);

  const util::type::ForecastCoord charging_coord_forecast(
      params.charging_coord.lat, params.charging_coord.lon);

  // Initialize simulation state variables
  double accumulated_distance = 0.0;  // In meters
  double driving_time = 0.0;          // In seconds
  double battery_energy = params.sim_start_soc;
  util::type::Time curr_time = params.sim_start_time;
  LogMetrics metrics;

  /** Update the irradiance variable - passed in by reference */
  auto update_irradiance = [&](const util::type::ForecastCoord& coord,
                               const util::type::Time& curr_time,
                               util::type::Irradiance& irr) {
    irr.dni = params.dni_lut.get_value_and_update_cache(
        coord, curr_time, dni_cache.first, dni_cache.second);
    irr.dhi = params.dhi_lut.get_value_and_update_cache(
        coord, curr_time, dhi_cache.first, dhi_cache.second);
  };

  /* Get route and race plan data */
  const size_t num_points = route.get_num_points();
  const CoordVec& route_points = route.get_route_points();
  const FSGPRacePlan::PlanData segments = race_plan->get_segments();

  spdlog::debug("Starting SOC: {}", car.get_max_soc());

  const bool is_first_day = curr_time >= params.day_one_start_time &&
                            curr_time < params.day_one_end_time;
  // End of day time and impounding start time
  util::type::Time current_day_end;
  if (is_first_day) {
    current_day_end = util::type::Time::combine_date_and_time(
        curr_time, params.day_one_end_time);
  } else {
    current_day_end =
        util::type::Time::combine_date_and_time(curr_time, params.day_end_time);
  }
  const util::type::Time impound_start =
      util::type::Time::combine_date_and_time(curr_time,
                                              params.impounding_start_time);

  // Start of next day and impouding release time
  util::type::Time next_day_start =
      util::type::Time::combine_date_and_time(curr_time, params.day_start_time);
  next_day_start += SECONDS_IN_DAY;
  const util::type::Time impound_release =
      util::type::Time::combine_date_and_time(next_day_start,
                                              params.impounding_release_time);

  CarUpdate car_update;
  const size_t num_loops = race_plan->get_num_loops();
  for (size_t loop_idx = 0; loop_idx < num_loops; loop_idx++) {
    // Get plan for current loop and iterate through each segment
    const FSGPRacePlan::LoopData& loop_segments = segments[loop_idx];

    const size_t num_segments = loop_segments.size();
    if (loop_idx == 0) {
      // Write starting condition of the car to the result csv
      metrics.update_metrics(car_update, util::type::Irradiance(0, 0, 0),
                             battery_energy, 0.0, 0.0, route_points[0], 0.0,
                             curr_time, 0.0);
    }

    for (size_t segment_idx = 0; segment_idx < num_segments; segment_idx++) {
      // Get segment information
      const FSGPSegment& current_segment = loop_segments[segment_idx];
      const double curr_speed = current_segment.start_speed;
      const size_t starting_idx = current_segment.start_idx;
      const size_t ending_idx = current_segment.end_idx;
      const bool is_accelerating = current_segment.acceleration_value != 0.0;
      const bool is_dual_segment = current_segment.is_dual_segment;
      const bool is_crossover_segment = current_segment.is_crossover_segment();
      const double acceleration_distance =
          current_segment.acceleration_distance;
      const double constant_distance = current_segment.constant_distance;
      const double segment_distance = current_segment.distance;
      const double acceleration = current_segment.acceleration_value;

      // If accelerating, then we travel from segment start to end in one shot
      size_t num_segment_points = ending_idx - starting_idx;
      if (is_accelerating) {  // Acceleration or dual segment
        num_segment_points = 1;
      } else if (is_crossover_segment) {
        num_segment_points = num_points - starting_idx + ending_idx;
      }
      // Traverse the segment
      for (size_t cnt = 0; cnt < num_segment_points; cnt++) {
        size_t coord_one = 0;
        size_t coord_two = 0;
        if (!is_accelerating) {
          if (starting_idx + cnt >= num_points) {
            // Wrap-around between loops. Don't expand the brackets, all of
            // these data types are size_t
            coord_one = cnt - (num_points - starting_idx);
          } else {
            coord_one = starting_idx + cnt;
          }
          coord_two = starting_idx + cnt == num_points - 1 ? 0 : coord_one + 1;
        } else {
          coord_one = starting_idx;
          coord_two = ending_idx;
        }
        const util::type::Coord& current_coord = route_points[coord_one];
        const util::type::Coord& next_coord = route_points[coord_two];
        double delta_energy = 0.0;

        /** @brief Lambda function for static charging between the start and end
         * time
         * @param start_time: The starting time of the charging period. This is
         * passed by reference and will be modified to eventually match the
         * timestamp of end_time
         * @param end_time: The ending time of the charging period
         */
        auto charge_in_time_interval = [&](util::type::Time& start_time,
                                           const util::type::Time& end_time) {
          while (start_time < end_time) {
            /* Step in 30-second intervals */
            sun = util::geo::get_az_el(params.charging_coord, curr_time);
            update_irradiance(charging_coord_forecast, start_time, irr);

            const double step_size =
                get_charging_step_size(start_time, end_time);
            if (sun.El > 0) {
              delta_energy += car.compute_static_energy(irr, sun, step_size);
              start_time += step_size;
            } else {
              start_time += step_size;
            }
          }
        };

        /* Overnight stop */
        if (curr_time >= current_day_end) {
          // Charge from EoD to impounding start time
          charge_in_time_interval(curr_time, impound_start);

          // Move curr_time to the next day's impounding release time
          curr_time = impound_release;

          // Charge from impounding release time to the race start
          charge_in_time_interval(curr_time, next_day_start);

          // Update the EoD and next day start timestamps
          current_day_end = util::type::Time::combine_date_and_time(
              curr_time, params.day_end_time);

          next_day_start = util::type::Time::combine_date_and_time(
              curr_time, params.day_start_time);
          next_day_start += SECONDS_IN_DAY;
        }

        /* Update forecast lut index caches and get forecast data at the src
         * coordinate */
        const util::type::ForecastCoord coord_one_forecast(current_coord.lat,
                                                           current_coord.lon);
        wind.speed = params.wind_speed_lut.get_value_and_update_cache(
            coord_one_forecast, curr_time, wind_speed_cache.first,
            wind_speed_cache.second);

        wind.bearing = params.wind_dir_lut.get_value_and_update_cache(
            coord_one_forecast, curr_time, wind_dir_cache.first,
            wind_dir_cache.second);

        update_irradiance(coord_one_forecast, curr_time, irr);

        /* Compute state update of the car */
        try {
          if (is_dual_segment) {
            // Dual segments have a component of acceleration, so we complete
            // the segment in one shot from start_idx to end_idx
            car_update = car.compute_dual_travel_update(
                current_coord, next_coord, curr_speed, acceleration, curr_time,
                wind, irr, acceleration_distance, constant_distance);
          } else if (is_accelerating) {
            // When accelerating, we complete the segment in one shot from
            // start_idx to end_idx
            car_update = car.compute_acceleration_travel_update(
                current_coord, next_coord, curr_speed, acceleration, curr_time,
                wind, irr, segment_distance);
          } else {
            const double delta_distance =
                route_distances.get_value(coord_one, coord_two);
            car_update = car.compute_constant_travel_update(
                current_coord, next_coord, curr_speed, curr_time, wind, irr,
                delta_distance);
          }
        } catch (const util::error::InvalidCalculation& e) {
          // Discriminant is negative. Some deceleration/acceleration cannot be
          // completed and this race plan should be thrown out. Any good route
          // segmentation function should create RacePlans such that this
          // exception is never thrown
          race_plan->set_viability(false);
          race_plan->set_inviability_reason(std::string(e.what()));
          metrics.register_dataset(results_lut);
          return;
        }

        /* Update the running state of the simulation */
        delta_energy += car_update.delta_energy;
        accumulated_distance += car_update.delta_distance;
        curr_time += car_update.delta_time;
        driving_time += car_update.delta_time;

        /* Make sure the battery doesn't exceed the maximum bound */
        if (battery_energy + delta_energy > car.get_max_soc()) {
          battery_energy = car.get_max_soc();
        } else {
          battery_energy += delta_energy;
        }
        spdlog::debug("Battery Energy: {}", battery_energy);

        /* Update the logs */
        metrics.update_metrics(car_update, irr, battery_energy, delta_energy,
                               accumulated_distance, next_coord, curr_speed,
                               curr_time, acceleration);

        /* Invalid simulation if battery goes below 0 or if the end of the race
         * has been reached */
        if (battery_energy < 0.0 || curr_time > params.race_end_time) {
          race_plan->set_viability(false);
          race_plan->set_inviability_reason("Out of battery charge");
          metrics.register_dataset(results_lut);
          return;
        }
      }
    }
  }
  race_plan->set_driving_time(driving_time);
  race_plan->set_end_time(curr_time);
  race_plan->set_accumulated_distance(accumulated_distance);
  race_plan->set_average_speed(accumulated_distance / driving_time);
  race_plan->set_viability(true);
  metrics.register_dataset(results_lut);
}

FSGPSimulator::FSGPSimulator(FSGPSimulatorParams params, Car model)
    : Simulator<FSGPSimulator, FSGPRacePlan, FSGPRoute>(std::move(model)),
      params(std::move(params)) {}
