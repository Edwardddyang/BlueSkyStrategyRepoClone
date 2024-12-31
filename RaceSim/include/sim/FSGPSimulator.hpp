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
#include "utils/Units.hpp"
#include "utils/Luts.hpp"
#include "sim/WSCSimulator.hpp"
#include "config/Config.hpp"

class FSGPSimulator : public WSCSimulator {
 private:
  /* Simulation parameters */
  Coord charging_coord;          // Charging coordinates of the car
  Time impounding_start_time;    // Impounding start time in 24 hour local time
  Time impounding_release_time;  // Impounding release time in 24 hour local time

 public:
  /* Load all LUTs upon construction */
  explicit FSGPSimulator(std::shared_ptr<Car> model);

  /** @brief Run a full simulation with a car object and a route. Return true if viable
  *
  * @param route: The Route to simulate on
  * @param race_plan: The race plan to use
  */
  void run_sim(const std::shared_ptr<Route>& route, RacePlan* race_plan);
};
