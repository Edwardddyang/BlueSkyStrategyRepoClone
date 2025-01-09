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
 protected:
  /* Weather forecasting LUTs */
  std::unique_ptr<ForecastLut> wind_speed_lut;
  std::unique_ptr<ForecastLut> wind_dir_lut;
  std::unique_ptr<ForecastLut> dni_lut;
  std::unique_ptr<ForecastLut> dhi_lut;

  /* Track results */
  std::unique_ptr<ResultsLut> results_lut;

  /* Route to simulate on and the model to use */
  std::shared_ptr<Car> car;

 public:
  /* Load all LUTs upon construction */
  explicit Simulator(std::shared_ptr<Car> model);

  /** @brief Run a full simulation with a car object and a route. Return true if viable
  *
  * @param route: The Route to simulate on
  * @param race_plan: The race plan to use
  */
  virtual void run_sim(const std::shared_ptr<Route>& route, RacePlan* race_plan) = 0;

  /* Write all logs to a csv */
  void write_result(std::string csv_path);

  /* Return the results_lut object. Used primarily for tests */
  ResultsLut get_results_lut() const;
};
