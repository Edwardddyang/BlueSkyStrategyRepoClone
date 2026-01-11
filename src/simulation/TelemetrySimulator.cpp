#include "sim/TelemetrySimulator.hpp"

#include <cstddef>
#include <utility>
#include <vector>

#include "SimUtils/Defines.hpp"
#include "SimUtils/Geography.hpp"
#include "SimUtils/Luts.hpp"
#include "SimUtils/Types.hpp"
#include "SimUtils/Utilities.hpp"
#include "model/Car.hpp"
#include "route/Route.hpp"
#include "sim/Simulator.hpp"

void TelemetrySimulator::run_sim(const TelemRoute& route,
                                 Luts::DataSet* results_lut) {
  RUNTIME_EXCEPTION(results_lut != nullptr, "Results is null");

  const CoordVec& telem_coords = route.get_route_points();
  const std::vector<util::type::Time>& telem_times = route.get_timestamps();
  const std::vector<double>& telem_speeds = route.get_speeds();
  const size_t num_points = route.get_num_points();

  RUNTIME_EXCEPTION(num_points >= 2, "Not enough telemetry points");
  RUNTIME_EXCEPTION(
      num_points == telem_times.size(),
      "Timestamps has different number of points compared to route");
  RUNTIME_EXCEPTION(
      num_points == telem_speeds.size(),
      "Timestamps has different number of points compared to speeds");
  RUNTIME_EXCEPTION(!params.wind_speed_lut.is_empty(),
                    "Wind speed lut must be loaded");
  RUNTIME_EXCEPTION(!params.wind_dir_lut.is_empty(),
                    "Wind direction lut must be loaded");
  RUNTIME_EXCEPTION(!params.dni_lut.is_empty(), "DNI lut must be loaded");
  RUNTIME_EXCEPTION(!params.dhi_lut.is_empty(), "DHI Lut must be loaded");

  // Reset results lut logs
  // results_lut->reset_logs();

  // Initialize simulation state variables
  double accumulated_distance = 0.0;  // In meters
  util::type::Time curr_time = telem_times[0];
  double battery_energy = params.sim_start_soc;
  const util::type::Coord starting_coord = telem_coords[0];
  double delta_energy = 0.0;
  util::type::Irradiance irr;
  util::type::Wind wind;
  LogMetrics metrics;

  // Initialize index caches for forecast lut lookups
  const util::type::ForecastCoord sim_start_forecast_coord(starting_coord);
  std::pair<size_t, size_t> dni_cache =
      params.dni_lut.initialize_caches(sim_start_forecast_coord, curr_time);
  std::pair<size_t, size_t> dhi_cache =
      params.dhi_lut.initialize_caches(sim_start_forecast_coord, curr_time);
  std::pair<size_t, size_t> wind_speed_cache =
      params.wind_speed_lut.initialize_caches(sim_start_forecast_coord,
                                              curr_time);
  std::pair<size_t, size_t> wind_dir_cache =
      params.wind_dir_lut.initialize_caches(sim_start_forecast_coord,
                                            curr_time);

  CarUpdate car_update;
  for (size_t i = 0; i < num_points - 1; i++) {
    const util::type::Coord& start = telem_coords[i];
    const util::type::Coord& end = telem_coords[i + 1];
    const double init_speed = telem_speeds[i];
    const double end_speed = telem_speeds[i + 1];
    curr_time = telem_times[i];

    /* Update forecast lut index caches and get forecast data at coordinate 1 */
    const util::type::ForecastCoord coord_one_forecast(start.lat, start.lon);

    wind.bearing = params.wind_dir_lut.get_value_and_update_cache(
        coord_one_forecast, curr_time, wind_dir_cache.first,
        wind_dir_cache.second);
    wind.speed = params.wind_speed_lut.get_value_and_update_cache(
        coord_one_forecast, curr_time, wind_speed_cache.first,
        wind_speed_cache.second);
    irr.dni = params.dni_lut.get_value_and_update_cache(
        coord_one_forecast, curr_time, dni_cache.first, dni_cache.second);
    irr.dhi = params.dhi_lut.get_value_and_update_cache(
        coord_one_forecast, curr_time, dhi_cache.first, dhi_cache.second);

    /* Compute state update of the car */
    const double segment_distance = util::geo::get_distance(start, end);
    const double acceleration =
        util::calc_acceleration(init_speed, end_speed, segment_distance);
    if (acceleration != 0.0) {
      car_update = car.compute_acceleration_travel_update(
          start, end, init_speed, acceleration, curr_time, wind, irr,
          segment_distance);
    } else {
      car_update = car.compute_constant_travel_update(
          start, end, init_speed, curr_time, wind, irr, segment_distance);
    }

    /* Update the running state of the simulation */
    delta_energy = car_update.delta_energy;
    accumulated_distance += car_update.delta_distance;

    /* Make sure the battery doesn't exceed the maximum bound */
    if (battery_energy + delta_energy > car.get_max_soc()) {
      battery_energy = car.get_max_soc();
    } else {
      battery_energy += delta_energy;
    }

    /* Update the logs */
    metrics.update_metrics(car_update, irr, battery_energy, delta_energy,
                           accumulated_distance, end, end_speed, curr_time,
                           acceleration);
  }
  metrics.register_dataset(results_lut);
}

TelemetrySimulator::TelemetrySimulator(TelemetrySimulatorParams params,
                                       Car model)
    : car(std::move(model)), params(std::move(params)) {}
