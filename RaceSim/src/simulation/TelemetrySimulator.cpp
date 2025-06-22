#include <memory>
#include <iostream>
#include <limits>
#include <vector>
#include <string>
#include <utility>
#include "sim/TelemetrySimulator.hpp"
#include "utils/CustomException.hpp"

void TelemetrySimulator::run_sim(const std::shared_ptr<Route>& route, RacePlan* race_plan,
                                 std::shared_ptr<ResultsLut> result_lut) {
    RUNTIME_EXCEPTION(false, "TelemetrySimulator should never invoke the base run_sim() function");
    return;
}

void TelemetrySimulator::run_sim(const std::shared_ptr<Route>& route,
                                 std::shared_ptr<ResultsLut> results_lut){
    RUNTIME_EXCEPTION(route != nullptr, "Route is null");
    RUNTIME_EXCEPTION(results_lut != nullptr, "Results is null");

    const std::vector<Coord> telem_coords = route->get_route_points();
    const std::vector<Time> telem_times = route->get_timestamps();
    const std::vector<double> telem_speeds = route->get_speeds();
    const size_t num_points = telem_coords.size();

    RUNTIME_EXCEPTION(num_points >= 2, "Not enough telemetry points");
    RUNTIME_EXCEPTION(num_points == telem_times.size(), "Timestamps has different number of points compared to route");
    RUNTIME_EXCEPTION(num_points == telem_speeds.size(), "Timestamps has different number of points compared to speeds");
    RUNTIME_EXCEPTION(wind_speed_lut != nullptr, "Wind speed lut must be loaded");
    RUNTIME_EXCEPTION(wind_dir_lut != nullptr, "Wind direction lut must be loaded");
    RUNTIME_EXCEPTION(dni_lut != nullptr, "DNI lut must be loaded");
    RUNTIME_EXCEPTION(dhi_lut != nullptr, "DHI Lut must be loaded");
    RUNTIME_EXCEPTION(ghi_lut != nullptr, "GHI Lut must be loaded");

    // Reset results lut logs
    results_lut->reset_logs();

    // Initialize simulation state variables
    double accumulated_distance = 0.0;  // In meters
    double driving_time = 0.0;  // In seconds
    Time curr_time = telem_times[0];
    double battery_energy = this->sim_start_soc;
    Coord starting_coord = this->sim_start_coord;
    double delta_energy;
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
    std::pair<size_t, size_t> ghi_cache = ghi_lut->initialize_caches(starting_coord,
                                                                    curr_time.get_utc_time_point());                                                     


    /* Get starting position in telemetry data */
    size_t starting_index = 0;
    double min_distance = std::numeric_limits<double>::max();

    for (size_t i = 0; i < num_points; ++i) {
        double distance = get_distance(telem_coords[i], starting_coord);
        if (distance < min_distance) {
            min_distance = distance;
            starting_index = i;
        }
    }

    for (size_t i = starting_index; i < num_points - 1; i++) {
        const Coord& start = telem_coords[i];
        const Coord& end = telem_coords[i + 1];
        const double init_speed = telem_speeds[i];
        const double end_speed = telem_speeds[i+1];
        curr_time = telem_times[i];

        /* Update forecast lut index caches and get forecast data at coordinate 1 */
        ForecastCoord coord_one_forecast(start.lat, start.lon);

        wind_speed_lut->update_index_cache(&wind_speed_cache, coord_one_forecast, curr_time.get_utc_time_point());
        double wind_speed = wind_speed_lut->get_value(wind_speed_cache);

        wind_dir_lut->update_index_cache(&wind_dir_cache, coord_one_forecast, curr_time.get_utc_time_point());
        double wind_dir = wind_dir_lut->get_value(wind_dir_cache);

        dni_lut->update_index_cache(&dni_cache, coord_one_forecast, curr_time.get_utc_time_point());
        double dni = dni_lut->get_value(dni_cache);

        dhi_lut->update_index_cache(&dhi_cache, coord_one_forecast, curr_time.get_utc_time_point());
        double dhi = dhi_lut->get_value(dhi_cache);

        ghi_lut->update_index_cache(&ghi_cache, coord_one_forecast, curr_time.get_utc_time_point());
        double ghi = ghi_lut->get_value(ghi_cache);

        Wind wind = Wind(wind_dir, wind_speed);
        Irradiance irr = Irradiance(dni, dhi, ghi);
        SolarAngle sun = SolarAngle();

        double segment_distance = get_distance(start, end);

        RUNTIME_EXCEPTION(num_points >= 2, "Telemetry must contain at least two coordinates.");

        /* Compute state update of the car */
        const double acceleration = calc_acceleration(init_speed, end_speed, segment_distance);
        const double acceleration_distance = acceleration ? segment_distance : 0.0;
        const double constant_distance = acceleration ? 0.0 : segment_distance;
        CarUpdate update = car->compute_travel_update(start, end, init_speed,
                                                      end_speed, acceleration,
                                                      &curr_time, wind, irr,
                                                      acceleration_distance, constant_distance);

        /* Update the running state of the simulation */
        delta_energy = update.delta_energy;
        accumulated_distance += update.delta_distance;
        driving_time += update.delta_time;

        /* Make sure the battery doesn't exceed the maximum bound */
        if (battery_energy + delta_energy > max_soc) {
            battery_energy = max_soc;
        } else {
            battery_energy += delta_energy;
        }

        /* Update the logs */
        results_lut->update_logs(update, irr, battery_energy, delta_energy, accumulated_distance,
                                end, end_speed, curr_time, acceleration);
    }
}

TelemetrySimulator::TelemetrySimulator(std::shared_ptr<Car> model) :
    car(std::dynamic_pointer_cast<V2Car>(model)),
    WSCSimulator(model) {}
