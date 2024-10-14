/* 
Class to run the a full scale simulation on a designated route
*/

#pragma once

#include <stdbool.h>
#include <string.h>
#include <memory>

#include "Route.hpp"
#include "Car.hpp"
#include "Units.hpp"
#include "Luts.hpp"
#include "Config.hpp"

class Simulator {
private:
   /* Step size in seconds when waiting overnight */
   const double OVERNIGHT_STEP_SIZE = 30.0;

   /* Weather forecasting LUTs */
   std::unique_ptr<ForecastLut> wind_speed_lut;
   std::unique_ptr<ForecastLut> wind_dir_lut;
   std::unique_ptr<ForecastLut> dni_lut;
   std::unique_ptr<ForecastLut> dhi_lut;

   /* Simulation parameters */
   double control_stop_charge_time;
   Time race_start; // Start time of race day in 24 hour
   Time race_end;  // End time of race day in 24 hour
   Time race_end_time; // End time of the entire race in 24 hour, date format
   Coord starting_coord;
   double max_soc;

   /* Internal running state of the simulation */
   Time curr_time;
   double battery_energy;
   double accumulated_distance;
   double delta_energy;
   double curr_speed;
   Coord current_coord;
   Coord next_coord;

   /* Logs to write to csv */
   std::vector<double> battery_energy_log;
   std::vector<double> accumulated_distance_log;
   std::vector<std::string> time_log;
   std::vector<double> azimuth_log;
   std::vector<double> elevation_log;
   std::vector<double> bearing_log;
   std::vector<double> latitude_log;
   std::vector<double> longitude_log;
   std::vector<double> altitude_log;
   std::vector<double> speed_log;
   std::vector<double> array_energy_log;
   std::vector<double> array_power_log;
   std::vector<double> motor_power_log;
   std::vector<double> motor_energy_log;
   std::vector<double> aero_power_log;
   std::vector<double> aero_energy_log;
   std::vector<double> rolling_power_log;
   std::vector<double> rolling_energy_log;
   std::vector<double> gravitational_power_log;
   std::vector<double> gravitational_energy_log;
   std::vector<double> electric_energy_log;
   std::vector<double> delta_energy_log;

   /* Route to simulate on and the model to use */
   std::unique_ptr<Car> car;

   /* Reset loop variables. Done before each simulation in run_sim(...) */
   void reset_vars();

   /* Reset logs. Done before each simulation in run_sim(...)*/
   void reset_logs();

   /* Update the logs to write to the csv */
   void update_logs(CarUpdate update);

public:
   /* Load all LUTs upon construction */
   Simulator(std::unique_ptr<Car> model);

   /** @brief Run a full simulation with a car object and a route. Return true if viable
    *
    * @param route: The Route to simulate on
    * @param speed_profile_kph: The speed to use for each segment
   */
   bool run_sim(const std::unique_ptr<Route>& route, std::vector<double> speed_profile_kph);

   /* Write all logs to a csv */
   void write_result(std::string csv_path);
};
