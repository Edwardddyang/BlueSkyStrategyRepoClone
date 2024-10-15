/* 
Class to run the a full scale simulation on a designated route
*/

#pragma once

#include <stdbool.h>
#include <string>
#include <memory>
#include <vector>

#include "route/Route.hpp"
#include "model/Car.hpp"
#include "utils/Units.hpp"
#include "utils/Luts.hpp"
#include "config/Config.hpp"

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
  Time race_start;  // Start time of race day in 24 hour
  Time race_end;   // End time of race day in 24 hour
  Time race_end_time;  // End time of the entire race in 24 hour, date format
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

  /* Track results */
  std::unique_ptr<ResultsLut> results_lut;

  /* Route to simulate on and the model to use */
  std::unique_ptr<Car> car;

  /* Reset loop variables. Done before each simulation in run_sim(...) */
  void reset_vars();

 public:
  /* Load all LUTs upon construction */
  explicit Simulator(std::unique_ptr<Car> model);

  /** @brief Run a full simulation with a car object and a route. Return true if viable
  *
  * @param route: The Route to simulate on
  * @param speed_profile_kph: The speed to use for each segment
  */
  bool run_sim(const std::unique_ptr<Route>& route, std::vector<double> speed_profile_kph);

  /* Write all logs to a csv */
  void write_result(std::string csv_path);

  /* Return the results_lut object. Used primarily for tests */
  ResultsLut get_results_lut() const;
};
