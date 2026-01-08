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
#include "config/ConfigParser.hpp"

struct TelemetrySimulatorParams : SimulatorParams {
  const double sim_start_soc;

  TelemetrySimulatorParams(ForecastMatrix wind_speed_lut, ForecastMatrix wind_dir_lut,
                           ForecastMatrix dni_lut, ForecastMatrix dhi_lut,
                           double sim_start_soc) :
                           SimulatorParams(wind_speed_lut, wind_dir_lut, dni_lut, dhi_lut),
                           sim_start_soc(sim_start_soc) {}
};

inline TelemetrySimulatorParams get_telem_simulator_params(ConfigParser* parser) {
  RUNTIME_EXCEPTION(parser != nullptr, "Config parser is null when loading telemetry simulation parameters");

  TelemetrySimulatorParams params{
    ForecastMatrix(parser->get_wind_speed_path()),
    ForecastMatrix(parser->get_wind_direction_path()),
    ForecastMatrix(parser->get_dni_path()),
    ForecastMatrix(parser->get_dhi_path()),
    parser->get_current_soc()
  };

  return params;
}


/** @brief Simulator to simulate the path taken by telemetry data through a csv loaded as
 * |Latitude|Longitude|Altitude|Time|Speed
*/
class TelemetrySimulator {
 private:
  // Car Model
  Car car;

  TelemetrySimulatorParams params;

 public:
  /* Load all LUTs upon construction */
  explicit TelemetrySimulator(TelemetrySimulatorParams params, Car model);

  /** @brief Run the car through all points on the route */
 void run_sim(const TelemRoute& route,
              Luts::DataSet* results_lut);
};
