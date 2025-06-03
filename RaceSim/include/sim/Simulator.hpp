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

// All (derived) class members should be initialized exactly ONCE when the object is constructed.
// Afterwards, they should only be read. This is to ensure that the Simulator object can be
// shared between threads. Do not track the internal state of the simulation e.g. soc, speed
// using class members. I'll find you.
class Simulator {
 protected:
  /* Weather forecasting LUTs */
  std::unique_ptr<ForecastLut> wind_speed_lut;
  std::unique_ptr<ForecastLut> wind_dir_lut;
  std::unique_ptr<ForecastLut> dni_lut;
  std::unique_ptr<ForecastLut> dhi_lut;
  std::unique_ptr<ForecastLut> ghi_lut;

  /* Route to simulate on and the model to use */
  std::shared_ptr<Car> car;

 public:
  /* Load all LUTs upon construction */
  explicit Simulator(std::shared_ptr<Car> model);

  /** @brief Run a full simulation with a car object and a route
  *
  * @param route: The Route to simulate on
  * @param race_plan: The race plan to use
  * @param result_lut: ResultsLut object for storing results
  */
  virtual void run_sim(const std::shared_ptr<Route>& route, RacePlan* race_plan,
                       std::shared_ptr<ResultsLut> result_lut) = 0;
};
