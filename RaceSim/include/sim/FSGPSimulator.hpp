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
class FSGPSimulator : public WSCSimulator {
 private:
  /* Event information */
  const Coord charging_coord;          // Charging coordinates of the car
  const Time impounding_start_time;    // Impounding start time in 24 hour local time
  const Time impounding_release_time;  // Impounding release time in 24 hour local time

  // Car Model
  std::shared_ptr<V2Car> car;

 public:
  /* Load all LUTs upon construction */
  explicit FSGPSimulator(std::shared_ptr<Car> model);

  /** @brief Run a full simulation with a car object and a route
  *
  * @param route: The Route to simulate on
  * @param race_plan: The race plan to use
  * @param result_lut: ResultsLut object for writing simulation result
  */
  // void run_sim(const std::shared_ptr<Route>& route, RacePlan* race_plan,
  //              std::shared_ptr<ResultsLut> result_lut) override;

  std::vector<double> run_sim(const std::shared_ptr<Route>& route, std::vector<Coord>, std::vector<Time>,
            std::shared_ptr<ResultsLut> result_lut);
};
