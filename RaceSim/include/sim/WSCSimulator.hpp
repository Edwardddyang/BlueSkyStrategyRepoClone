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

  /* Information about the event */
  const int control_stop_charge_time;  // Stop time at a control stop in seconds
  const double sim_start_soc;          // Starting soc for the simulation. Retrieved from config
  const Coord sim_start_coord;         // Starting coordinates of the car
  const Time sim_start_time;           // Starting time of the simulation i.e. irl time
  const Time day_one_start_time;       // Day one start time in 24 hour local time
  const Time day_one_end_time;         // Day one end time in 24 hour local time
  const Time day_start_time;           // Start time from day 2 onwards in 24 hour local time
  const Time day_end_time;             // End time from day 2 onwards in 24 hour local time
  const Time race_end_time;            // End time of the entire race in 24 hour local time
  const double max_soc;                // Maximum soc of the car retrieved from config

 public:
  /* Load all LUTs upon construction */
  explicit WSCSimulator(std::shared_ptr<Car> model);

  /** @brief Run a full simulation with a car object and a route
  *
  * @param route: The Route to simulate on
  * @param race_plan: The race plan to use
  * @param result_lut: ResultsLut object for writing simulation result
  */
  void run_sim(const std::shared_ptr<Route>& route, RacePlan* race_plan,
               std::shared_ptr<ResultsLut> result_lut) override;
};
