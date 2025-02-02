/* 
Class to run the a full scale simulation on a WSC type route
*/

#pragma once

#include <stdbool.h>
#include <string>
#include <memory>
#include <vector>

#include "sim/Simulator.hpp"
#include "route/Route.hpp"
#include "model/Car.hpp"
#include "utils/Units.hpp"
#include "utils/Luts.hpp"
#include "config/Config.hpp"

class WSCSimulator : public Simulator {
 protected:
  /* Step size in seconds when charging */
  const int CHARGING_STEP_SIZE = 30;

  /* Simulation parameters */
  int control_stop_charge_time;  // Stop time at a control stop in seconds
  Time day_one_start_time;  // Day one start time in 24 hour local time
  Time day_one_end_time;    // Day one end time in 24 hour local time
  Time day_start_time;      // Start time from day 2 onwards in 24 hour local time
  Time day_end_time;        // End time from day 2 onwards in 24 hour local time
  Time race_end_time;       // End time of the entire race in 24 hour local time
  Coord starting_coord;     // Starting coordinates of the car
  double max_soc;

  /* Internal running state of the simulation */
  Time curr_time;
  double battery_energy;
  double accumulated_distance;
  double delta_energy;
  double curr_speed;
  bool is_accelerating;
  double acceleration;
  /* Reset loop variables. Done before each simulation in run_sim(...) */
  void reset_vars();

 public:
  /* Load all LUTs upon construction */
  explicit WSCSimulator(std::shared_ptr<Car> model);

  /** @brief Run a full simulation with a car object and a route. Return true if viable
  *
  * @param route: The Route to simulate on
  * @param race_plan: The race plan to use
  */
  void run_sim(const std::shared_ptr<Route>& route, RacePlan* race_plan);

  /* Write all logs to a csv */
  void write_result(std::string csv_path);

  /* Return the results_lut object. Used primarily for tests */
  ResultsLut get_results_lut() const;
};
