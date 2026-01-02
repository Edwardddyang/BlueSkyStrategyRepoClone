/* 
Class to run the a full scale simulation on a FSGP type route (track race)
*/

#pragma once

#include <stdbool.h>
#include <string>
#include <memory>
#include <vector>

#include "route/Route.hpp"
#include "route/RacePlan.hpp"
#include "model/Car.hpp"
#include "SimUtils/Types.hpp"
#include "SimUtils/Luts.hpp"
#include "sim/Simulator.hpp"
#include "config/Config.hpp"

/** @brief Simulator to simulate a RacePlan on the FSGP route. Simulation parameters
 * and states are private to the run_sim() function such that all threads share
 * a single FSGPSimulator object
 */
class FSGPSimulator : public Simulator<FSGPSimulator, FSGPRacePlan, FSGPRoute> {
 private:
  /* Event information */
  const util::type::Coord charging_coord;          // Charging coordinates of the car
  const util::type::Time impounding_start_time;    // Impounding start time in 24 hour local time
  const util::type::Time impounding_release_time;  // Impounding release time in 24 hour local time
  const double sim_start_soc;                      // Starting soc for the simulation in kWh
  const util::type::Coord sim_start_coord;         // Starting coordinates of the car
  const util::type::Time sim_start_time;           // Starting time of the simulation i.e. irl time
  const util::type::Time day_one_start_time;       // Day one start time in 24 hour local time
  const util::type::Time day_one_end_time;         // Day one end time in 24 hour local time
  const util::type::Time day_start_time;           // Start time from day 2 onwards in 24 hour local time
  const util::type::Time day_end_time;             // End time from day 2 onwards in 24 hour local time
  const util::type::Time race_end_time;            // End time of the entire race in 24 hour local time
  const double max_soc;                            // Maximum soc of the car in kWh

 public:
  /* Load all LUTs upon construction */
  explicit FSGPSimulator(Car model);

  /** @brief Run a full simulation with a car object and a route
  *
  * @param route: The Route to simulate on
  * @param race_plan: The race plan to use
  * @param result_lut: ResultsLut object for writing simulation result
  */
  void run_sim_impl(const FSGPRoute& route, FSGPRacePlan* race_plan,
                    Luts::DataSet* result_lut);
};
