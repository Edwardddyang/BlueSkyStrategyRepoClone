/* 
Class to run the a full scale simulation on a FSGP type route (track race)
*/

#pragma once

#include <stdbool.h>
#include <string>
#include <memory>
#include <vector>

#include "route/Route.hpp"
#include "model/Car.hpp"
#include "model/V2Car.hpp"
#include "utils/Units.hpp"
#include "utils/Luts.hpp"
#include "sim/WSCSimulator.hpp"
#include "config/Config.hpp"

/** @brief Simulator to simulate a RacePlan on the FSGP route. Simulation parameters
 * and states are private to the run_sim() function such that all threads share
 * a single FSGPSimulator object
 */
class FSGPSimulator : public SimulatorBaseCrtp<FSGPSimulator, FSGPRoute> {
 private:
  /* Event information */
  const Coord charging_coord;          // Charging coordinates of the car
  const Time impounding_start_time;    // Impounding start time in 24 hour local time
  const Time impounding_release_time;  // Impounding release time in 24 hour local time
  const double sim_start_soc;          // Starting soc for the simulation. Retrieved from config
  const Coord sim_start_coord;         // Starting coordinates of the car
  const Time sim_start_time;           // Starting time of the simulation i.e. irl time
  const Time day_one_start_time;       // Day one start time in 24 hour local time
  const Time day_one_end_time;         // Day one end time in 24 hour local time
  const Time day_start_time;           // Start time from day 2 onwards in 24 hour local time
  const Time day_end_time;             // End time from day 2 onwards in 24 hour local time
  const Time race_end_time;            // End time of the entire race in 24 hour local time
  const double max_soc;                // Maximum soc of the car retrieved from config
  // Car Model
  std::shared_ptr<V2Car> v2_car;

 public:
  /* Load all LUTs upon construction */
  explicit FSGPSimulator(std::shared_ptr<Car> model);

  /** @brief Run a full simulation with a car object and a route
  *
  * @param route: The Route to simulate on
  * @param race_plan: The race plan to use
  * @param result_lut: ResultsLut object for writing simulation result
  */
  void run_sim_impl(std::shared_ptr<FSGPRoute> route, RacePlan* race_plan,
               std::shared_ptr<ResultsLut> result_lut);
};
