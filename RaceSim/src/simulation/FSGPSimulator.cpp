#include <memory>
#include <iostream>
#include <limits>
#include <vector>
#include <string>
#include <utility>
#include "sim/FSGPSimulator.hpp"
#include "utils/CustomException.hpp"

void FSGPSimulator::run_sim(const std::shared_ptr<Route>& route, RacePlan* race_plan,
                            std::shared_ptr<ResultsLut> results_lut) {
  RUNTIME_EXCEPTION(route != nullptr, "Route pointer is null");
  RUNTIME_EXCEPTION(results_lut != nullptr, "Results lut is null");
  RUNTIME_EXCEPTION(race_plan != nullptr, "Race plan is null");
  if (race_plan->is_empty()) {
    std::cout << "Empty race plan, reason: " << race_plan->get_inviability_reason() << std::endl;
    return;
  } else {
    RUNTIME_EXCEPTION(race_plan->validate_members(route), "Race Plan is improperly created");
  }
  race_plan->set_start_time(this->sim_start_time);

  const BasicLut& route_distances = route->get_precomputed_distances();
  const std::vector<double> cornering_speed_limits = route->get_cornering_speed_bounds();
  RUNTIME_EXCEPTION(!route_distances.is_empty(), "FSGP Simulator must use pre-computed distances,"
                                                 "but no data was loaded");
  RUNTIME_EXCEPTION(wind_speed_lut != nullptr, "Wind speed lut must be loaded");
  RUNTIME_EXCEPTION(wind_dir_lut != nullptr, "Wind direction lut must be loaded");
  std::pair<size_t, size_t> dni_cache, dhi_cache, ghi_cache;
  Irradiance irr;
  if (use_ghi) {
    RUNTIME_EXCEPTION(ghi_lut != nullptr, "GHI Lut must be loaded");
    ghi_cache = ghi_lut->initialize_caches(this->sim_start_coord, this->sim_start_time.get_utc_time_point());
  } else {
    RUNTIME_EXCEPTION(dni_lut != nullptr, "DNI lut must be loaded");
    RUNTIME_EXCEPTION(dhi_lut != nullptr, "DHI Lut must be loaded");
    dni_cache = dni_lut->initialize_caches(this->sim_start_coord, this->sim_start_time.get_utc_time_point());
    dhi_cache = dhi_lut->initialize_caches(this->sim_start_coord, this->sim_start_time.get_utc_time_point());
  }


  // Reset results lut logs
  results_lut->reset_logs();

  // Initialize simulation state variables
  double accumulated_distance = 0.0;  // In meters
  double driving_time = 0.0;  // In seconds
  Time curr_time = this->sim_start_time;
  double battery_energy = this->sim_start_soc;
  Coord starting_coord = this->sim_start_coord;
  double delta_energy;

  // Initialize index caches for forecast lut lookups
  std::pair<size_t, size_t> wind_speed_cache = wind_speed_lut->initialize_caches(starting_coord,
                                                                                 curr_time.get_utc_time_point());
  std::pair<size_t, size_t> wind_dir_cache = wind_dir_lut->initialize_caches(starting_coord,
                                                                             curr_time.get_utc_time_point());

  /** Update the irradiance variable - passed in by reference */
  auto update_irradiance = [&](const ForecastCoord& coord, const Time& curr_time, Irradiance& irr) {
    double dni = 0.0;
    double dhi = 0.0;
    double ghi = 0.0;
    if (use_ghi) {
      ghi_lut->update_index_cache(&ghi_cache, coord, curr_time.get_utc_time_point());
      irr.ghi = ghi_lut->get_value(ghi_cache);
    } else {
      dni_lut->update_index_cache(&dni_cache, coord, curr_time.get_utc_time_point());
      irr.dni = dni_lut->get_value(dni_cache);

      dhi_lut->update_index_cache(&dhi_cache, coord, curr_time.get_utc_time_point());
      irr.dhi = dhi_lut->get_value(dhi_cache);
    }
  };

  /* Get route data */
  const size_t num_points = route->get_num_points();
  const std::vector<Coord>& route_points = route->get_route_points();

  /* Get race plan data */
  const RacePlan::PlanData segments = race_plan->get_segments();

  /* Get starting position in the route */
  size_t starting_route_index = 0;
  double min_distance = std::numeric_limits<double>::max();
  for (size_t i = 0; i < num_points; i++) {
    const Coord& route_coord = route_points[i];
    double distance = get_distance(route_coord, starting_coord);
    if (distance < min_distance) {
      min_distance = distance;
      starting_route_index = i;
    }
  }
  spdlog::debug("Starting SOC: {}", max_soc);

  bool is_first_day = curr_time >= day_one_start_time && curr_time < day_one_end_time;

  // Keep track of the end time of the current day and the start time of the next day
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

  CarUpdate car_update;
  const size_t num_loops = segments.size();
  for (size_t loop_idx = 0; loop_idx < num_loops; loop_idx++) {
    // Get plan for current loop
    const RacePlan::LoopData loop_segments = segments[loop_idx];
    // Get plan for current segment
    const size_t num_segments = loop_segments.size();

    if (loop_idx == 0) {
      // Write starting condition of the car to the result csv
      results_lut->update_logs(car_update, Irradiance(0,0,0), battery_energy, 0.0, 0.0,
                               route_points[0], 0.0, curr_time, 0.0);
    }

    for (size_t segment_idx = 0; segment_idx < num_segments; segment_idx++) {
      // Get segment information
      RacePlan::SegmentData current_segment = loop_segments[segment_idx];
      double curr_speed = current_segment.start_speed;
      const double ending_speed = current_segment.end_speed;
      const size_t starting_idx = current_segment.start_idx;
      const size_t ending_idx = current_segment.end_idx;
      const bool is_accelerating = current_segment.acceleration_value != 0.0;
      const double acceleration = current_segment.acceleration_value;

      // If accelerating, then we travel from segment start to end in one shot
      size_t num_segment_points;
      if (is_accelerating) {
        num_segment_points = 1;
      } else if (ending_idx < starting_idx) {
        num_segment_points = num_points - starting_idx + ending_idx;
      } else {
        num_segment_points = ending_idx - starting_idx;
      }
      // Traverse the segment
      for (size_t cnt = 0; cnt < num_segment_points; cnt++) {
        size_t coord_one, coord_two;
        if (!is_accelerating) {
          if (starting_idx + cnt >= num_points) {
            // Wrap-around between loops. Don't expand the brackets, all of these data types
            // are size_t
            coord_one = cnt - (num_points - starting_idx);
          } else {
            coord_one = starting_idx + cnt;
          }
          coord_two = starting_idx + cnt == num_points - 1 ? 0 : coord_one + 1;
        } else {
          coord_one = starting_idx;
          coord_two = ending_idx;
        }
        const Coord& current_coord = route_points[coord_one];
        const Coord& next_coord = route_points[coord_two];
        delta_energy = 0.0;

        /* Update forecast lut index caches and get forecast data at the src coordinate */
        ForecastCoord coord_one_forecast(current_coord.lat, current_coord.lon);
        
        wind_speed_lut->update_index_cache(&wind_speed_cache, coord_one_forecast, curr_time.get_utc_time_point());
        double wind_speed = wind_speed_lut->get_value(wind_speed_cache);

        wind_dir_lut->update_index_cache(&wind_dir_cache, coord_one_forecast, curr_time.get_utc_time_point());
        double wind_dir = wind_dir_lut->get_value(wind_dir_cache);
        Wind wind = Wind(wind_dir, wind_speed);

        update_irradiance(coord_one_forecast, curr_time, irr);

        /** @brief Lambda function for static charging between the start and end time
         * @param start_time: The starting time of the charging period. This will be modified
         * to eventually match the timestamp of end_time
         * @param end_time: The ending time of the charging period
         * Note: Captures parameters by reference, start_time will be updated
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
              delta_energy += car->compute_static_energy(current_coord, &start_time, step_size, irr, "fsgp");

              start_time.update_time_seconds(step_size);
              update_irradiance(coord_one_forecast, start_time, irr);
            } else {
              start_time.update_time_seconds(step_size);
            }
          }
        };
        /* Overnight stop */
        if (curr_time >= current_day_end) {
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
          next_day_start.update_time_seconds(SECONDS_IN_DAY);
        }

        /* Compute state update of the car */
        try {
          double acceleration_distance;
          double constant_distance;
          if (is_accelerating) {
            // When accelerating, we complete the segment in one shot from start_idx to end_idx
            acceleration_distance = calc_distance_a(curr_speed, ending_speed, acceleration);
            constant_distance = route_distances.get_value(current_segment.start_idx, current_segment.end_idx)
                                - acceleration_distance;
            car_update = v2_car->compute_travel_update(route_points[current_segment.start_idx],
                                                    route_points[current_segment.end_idx],
                                                    curr_speed, ending_speed, acceleration,
                                                    &curr_time, wind, irr, acceleration_distance,
                                                    constant_distance);
          } else {
            acceleration_distance = 0.0;
            constant_distance = route_distances.get_value(coord_one, coord_two);
            car_update = v2_car->compute_travel_update(current_coord, next_coord, curr_speed, curr_speed,
                                                    acceleration, &curr_time, wind, irr, acceleration_distance,
                                                    constant_distance);
          }
        } catch (const InvalidCalculation& e) {
          // Discriminant is negative. Some deceleration/acceleration cannot be completed and this race plan
          // should be thrown out. Any good route segmentation function should create RacePlans such that this
          // exception is never thrown
          race_plan->set_viability(false);
          race_plan->set_inviability_reason(std::string(e.what()));
          return;
        }

        /* Update the running state of the simulation */
        delta_energy += car_update.delta_energy;
        accumulated_distance += car_update.delta_distance;
        curr_time.update_time_seconds(car_update.delta_time);
        driving_time += car_update.delta_time;
        curr_speed = calc_final_speed(curr_speed, acceleration, car_update.delta_time);

        /* Make sure the battery doesn't exceed the maximum bound */
        if (battery_energy + delta_energy > max_soc) {
          battery_energy = max_soc;
        } else {
          battery_energy += delta_energy;
        }
        spdlog::debug("Battery Energy: {}", battery_energy);

        /* Update the logs */
        if (!is_accelerating) {
          results_lut->update_logs(car_update, irr, battery_energy, delta_energy, accumulated_distance,
                                  next_coord, curr_speed, curr_time, acceleration);
        } else {
          results_lut->update_logs(car_update, irr, battery_energy, delta_energy, accumulated_distance,
                                  route_points[current_segment.end_idx], curr_speed, curr_time, acceleration);
        }

        /* Invalid simulation if battery goes below 0 or if the end of the race has been reached */
        if (battery_energy < 0.0 || curr_time > race_end_time) {
          race_plan->set_viability(false);
          race_plan->set_inviability_reason("Out of battery charge");
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
}

FSGPSimulator::FSGPSimulator(std::shared_ptr<Car> model) :
  charging_coord(Config::get_instance()->get_overnight_charging_location()),
  impounding_start_time(Config::get_instance()->get_impounding_start_time()),
  impounding_release_time(Config::get_instance()->get_impounding_release_time()),
  v2_car(std::dynamic_pointer_cast<V2Car>(model)),
  WSCSimulator(model) {}