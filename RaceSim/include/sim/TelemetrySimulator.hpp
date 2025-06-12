/* 
Class to run a simulation on telemetry data
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

/** @brief Simulator to simulate the path taken by telemetry data through a csv loaded as
 * |Latitude|Longitude|Time|
*/
class TelemetrySimulator : public WSCSimulator {
 private:
  // Car Model
  std::shared_ptr<V2Car> car;

 public:
  /* Load all LUTs upon construction */
  explicit TelemetrySimulator(std::shared_ptr<Car> model);

  /** @brief Unused function from base class */
  void run_sim(const std::shared_ptr<Route>& route, RacePlan* race_plan,
               std::shared_ptr<ResultsLut> result_lut) override;

  /** @brief Run the car through all points on the route */
  void run_sim(const std::shared_ptr<Route>& route,
                              std::shared_ptr<ResultsLut> results_lut);
};
