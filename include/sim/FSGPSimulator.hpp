/*
Class to run the a full scale simulation on a FSGP type route (track race)
*/

#pragma once

#include <utility>

#include "SimUtils/Defines.hpp"
#include "SimUtils/Luts.hpp"
#include "SimUtils/Types.hpp"
#include "config/ConfigParser.hpp"
#include "model/Car.hpp"
#include "route/RacePlan.hpp"
#include "route/Route.hpp"
#include "sim/Simulator.hpp"

// Parameters injected to FSGPSimulator
struct FSGPSimulatorParams : SimulatorParams {
  util::type::Coord charging_coord;  // Charging coordinates of the car
  util::type::Time
      impounding_start_time;  // Impounding start time in 24 hour local time
  util::type::Time
      impounding_release_time;  // Impounding release time in 24 hour local time
  double sim_start_soc;         // Starting soc for the simulation in kWh
  util::type::Coord sim_start_coord;  // Starting coordinates of the car
  util::type::Time
      sim_start_time;  // Starting time of the simulation i.e. irl time
  util::type::Time
      day_one_start_time;  // Day one start time in 24 hour local time
  util::type::Time day_one_end_time;  // Day one end time in 24 hour local time
  util::type::Time
      day_start_time;  // Start time from day 2 onwards in 24 hour local time
  util::type::Time
      day_end_time;  // End time from day 2 onwards in 24 hour local time
  util::type::Time
      race_end_time;  // End time of the entire race in 24 hour local time

  FSGPSimulatorParams(
      ForecastMatrix wind_speed_lut, ForecastMatrix wind_dir_lut,
      ForecastMatrix dni_lut, ForecastMatrix dhi_lut,
      util::type::Coord charging_coord, util::type::Time impounding_start_time,
      util::type::Time impounding_release_time, double sim_start_soc,
      util::type::Coord sim_start_coord, util::type::Time sim_start_time,
      util::type::Time day_one_start_time, util::type::Time day_one_end_time,
      util::type::Time day_start_time, util::type::Time day_end_time,
      util::type::Time race_end_time)
      : SimulatorParams(std::move(wind_speed_lut), std::move(wind_dir_lut),
                        std::move(dni_lut), std::move(dhi_lut)),
        charging_coord(charging_coord),
        impounding_start_time(impounding_start_time),
        impounding_release_time(impounding_release_time),
        sim_start_soc(sim_start_soc),
        sim_start_coord(sim_start_coord),
        sim_start_time(sim_start_time),
        day_one_start_time(day_one_start_time),
        day_one_end_time(day_one_end_time),
        day_start_time(day_start_time),
        day_end_time(day_end_time),
        race_end_time(race_end_time) {}
};

inline FSGPSimulatorParams get_fsgp_simulator_params(ConfigParser* parser) {
  RUNTIME_EXCEPTION(
      parser != nullptr,
      "Config parser is null when loading FSGP simulation parameters");

  FSGPSimulatorParams params{ForecastMatrix(parser->get_wind_speed_path()),
                             ForecastMatrix(parser->get_wind_direction_path()),
                             ForecastMatrix(parser->get_dni_path()),
                             ForecastMatrix(parser->get_dhi_path()),
                             parser->get_overnight_charging_location(),
                             parser->get_impounding_start_time(),
                             parser->get_impounding_release_time(),
                             parser->get_current_soc(),
                             parser->get_gps_coordinates(),
                             parser->get_current_date_time(),
                             parser->get_day_one_start_time(),
                             parser->get_day_one_end_time(),
                             parser->get_day_start_time(),
                             parser->get_day_end_time(),
                             parser->get_race_end_time()};

  return params;
}

/** @brief Simulator to simulate a RacePlan on the FSGP route. Simulation
 * parameters and states are private to the run_sim() function such that all
 * threads share a single FSGPSimulator object
 */
class FSGPSimulator : public Simulator<FSGPSimulator, FSGPRacePlan, FSGPRoute> {
 private:
  FSGPSimulatorParams params;

 public:
  /* Load all LUTs upon construction */
  explicit FSGPSimulator(FSGPSimulatorParams params, Car model);

  /** @brief Run a full simulation with a car object and a route
   *
   * @param route: The Route to simulate on
   * @param race_plan: The race plan to use
   * @param results_lut: ResultsLut object for writing simulation result
   */
  void run_sim_impl(const FSGPRoute& route, FSGPRacePlan* race_plan,
                    Luts::DataSet* results_lut) const;
};
