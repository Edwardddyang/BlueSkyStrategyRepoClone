#include <stdlib.h>
#include <assert.h>
#include <unordered_set>
#include <string.h>
#include <iostream>
#include <fstream>

#include "Simulator.hpp"
#include "Units.hpp"
#include "Route.hpp"
#include "Config.hpp"
#include "Geography.hpp"
#include "spdlog/spdlog.h"
#include "Defines.hpp"

bool Simulator::run_sim(const std::unique_ptr<Route>& route, std::vector<double> speed_profile_kph) {
    RUNTIME_EXCEPTION(route != nullptr, "Route pointer is null");
    RUNTIME_EXCEPTION(speed_profile_kph.size() == route->get_num_segments(),
                      "Speed profile and route are deformed");

    /* Reset variables */
    reset_vars();
    reset_logs();

    /* Get route data */
    const size_t num_points = route->get_num_points();
    const std::vector<Coord> route_points = route->get_route_points();
    const std::vector<std::pair<size_t, size_t>> segments = route->get_segments();
    const std::unordered_set<size_t> control_stops = route->get_control_stops();

    RUNTIME_EXCEPTION(segments.size() == speed_profile_kph.size(),
                      "Route was not segmented properly");

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
    for (size_t idx=starting_route_index; idx < num_points-1; idx++) {
        current_coord = route_points[idx];
        next_coord = route_points[idx+1];
        delta_energy = 0.0;

        /* Get forecasting data - wind and irradiance */
        ForecastCoord coord_one_forecast(current_coord.lat, current_coord.lon);

        // curr_time.print_local_readable_time();

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
            get_az_el(curr_time.get_utc_time_point(), current_coord.lat, current_coord.lon, current_coord.alt, &sun.Az, &sun.El);

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
        CarUpdate update = car->compute_travel_update(
            current_coord, next_coord, curr_speed, &curr_time, wind, irr
        );

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
        update_logs(update);

        /* Invalid simulation if battery goes below 0 or if the end of the race has been reached */
        if (battery_energy < 0.0 || curr_time > race_end_time) {
            return false;
        }
    }
    return true;
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

void Simulator::reset_logs() {
    battery_energy_log.clear();
    accumulated_distance_log.clear();
    time_log.clear();
    azimuth_log.clear();
    elevation_log.clear();
    bearing_log.clear();
    latitude_log.clear();
    longitude_log.clear();
    altitude_log.clear();
    speed_log.clear();
    array_energy_log.clear();
    array_power_log.clear();
    motor_power_log.clear();
    motor_energy_log.clear();
    aero_power_log.clear();
    aero_energy_log.clear();
    rolling_power_log.clear();
    rolling_energy_log.clear();
    gravitational_power_log.clear();
    gravitational_energy_log.clear();
    electric_energy_log.clear();
    delta_energy_log.clear();
}

void Simulator::update_logs(CarUpdate update) {
    battery_energy_log.push_back(battery_energy);
    delta_energy_log.push_back(delta_energy);
    accumulated_distance_log.push_back(accumulated_distance);
    azimuth_log.push_back(update.az_el.Az);
    elevation_log.push_back(update.az_el.El);
    bearing_log.push_back(update.bearing);
    latitude_log.push_back(next_coord.lat);
    longitude_log.push_back(next_coord.lon);
    altitude_log.push_back(next_coord.alt);
    array_energy_log.push_back(update.array.energy);
    array_power_log.push_back(update.array.power);
    speed_log.push_back(curr_speed);
    motor_power_log.push_back(update.motor_power);
    motor_energy_log.push_back(update.motor_energy);
    aero_power_log.push_back(update.aero.power);
    aero_energy_log.push_back(update.aero.energy);
    rolling_power_log.push_back(update.rolling.power);
    rolling_energy_log.push_back(update.rolling.energy);
    gravitational_power_log.push_back(update.gravitational.power);
    gravitational_energy_log.push_back(update.gravitational.energy);
    electric_energy_log.push_back(update.electric);
    time_log.push_back(curr_time.get_local_readable_time());
}

void Simulator::write_result(std::string csv_path) {
    std::ofstream output_csv(csv_path);
    RUNTIME_EXCEPTION(output_csv.is_open(), "Unable to open csv file path: {}", csv_path);

    output_csv << "Battery Charge(kWh),"
                << "Accumulated Distance(m),"
                << "DateTime,"
                << "Azimuth(Degrees),"
                << "Elevation(Degrees),"
                << "Bearing(Degrees),"
                << "Latitude,"
                << "Longitude,"
                << "Altitude(m),"
                << "Speed(m/s),"
                << "Array Power(W),"
                << "Array Energy(kWh),"
                << "Motor Power(W),"
                << "Motor Energy(kWh),"
                << "Aero Power(W),"
                << "Aero Energy(kWh),"
                << "Rolling Power(W),"
                << "Rolling Energy(kWh),"
                << "Gravitational Power(W),"
                << "Gravitational Energy(kWh),"
                << "Electric Energy(W),"
                << "Delta Battery(kWh),\n";

    int num_points = battery_energy_log.size();
    for (int i=0; i<num_points; i++) {
        output_csv << std::to_string(battery_energy_log[i]) + ",";
        output_csv << std::to_string(accumulated_distance_log[i]) + ",";
        output_csv << (std::string(time_log[i]) + ",");
        output_csv << std::to_string(azimuth_log[i]) + ",";
        output_csv << std::to_string(elevation_log[i]) + ",";
        output_csv << std::to_string(bearing_log[i]) + ",";
        output_csv << std::to_string(latitude_log[i]) + ",";
        output_csv << std::to_string(longitude_log[i]) + ",";
        output_csv << std::to_string(altitude_log[i]) + ",";
        output_csv << std::to_string(speed_log[i]) + ",";
        output_csv << std::to_string(array_power_log[i]) + ",";
        output_csv << std::to_string(array_energy_log[i]) + ",";
        output_csv << std::to_string(motor_power_log[i]) + ",";
        output_csv << std::to_string(motor_energy_log[i]) + ",";
        output_csv << std::to_string(aero_power_log[i]) + ",";
        output_csv << std::to_string(aero_energy_log[i]) + ",";
        output_csv << std::to_string(rolling_power_log[i]) + ",";
        output_csv << std::to_string(rolling_energy_log[i]) + ",";
        output_csv << std::to_string(gravitational_power_log[i]) + ",";
        output_csv << std::to_string(gravitational_energy_log[i]) + ",";
        output_csv << std::to_string(electric_energy_log[i]) + ",";
        output_csv << std::to_string(delta_energy_log[i]) + ",\n";
    }

    spdlog::info("Simulation data saved");
}

Simulator::Simulator(std::unique_ptr<Car> model) :
    car(std::move(model)),
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
