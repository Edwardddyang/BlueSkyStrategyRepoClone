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
#include "SimUtils/Constants.hpp"
#include "SimUtils/Types.hpp"
#include "SimUtils/Luts.hpp"
#include "sim/Simulator.hpp"
#include "config/Config.hpp"

/** @brief Simulator to simulate the path taken by telemetry data through a csv loaded as
 * |Latitude|Longitude|Altitude|Time|Speed
*/

class TelemetrySimulator {
 private:
  // Car Model
  Car car;

  const double sim_start_soc;                 // Starting soc for the simulation in kWh
  const double max_soc;                       // Maximum soc of the car in kWh

	/* Weather forecasting LUTs */
	const ForecastMatrix wind_speed_lut;
	const ForecastMatrix wind_dir_lut;
	const ForecastMatrix dni_lut;
	const ForecastMatrix dhi_lut;

 public:
  /* Load all LUTs upon construction */
  explicit TelemetrySimulator(Car model);

  /** @brief Run the car through all points on the route */
 void run_sim(const TelemRoute& route,
              Luts::DataSet* results_lut);
};
