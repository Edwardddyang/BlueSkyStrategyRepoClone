/*
Class to run the a full scale simulation on a WSC type route
*/

#pragma once

#include <utility>

#include "SimUtils/Constants.hpp"
#include "SimUtils/Defines.hpp"
#include "SimUtils/Luts.hpp"
#include "SimUtils/Types.hpp"
#include "config/ConfigParser.hpp"
#include "model/Car.hpp"
#include "route/RacePlan.hpp"
#include "route/Route.hpp"
#include "sim/Simulator.hpp"

// Parameters injected to WSCSimulator
struct WSCSimulatorParams : SimulatorParams {
  double control_stop_charge_time;    // Control stop time in seconds
  double sim_start_soc;               // Starting soc for the simulation in kWh
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

  WSCSimulatorParams(ForecastMatrix wind_speed_lut, ForecastMatrix wind_dir_lut,
                     ForecastMatrix dni_lut, ForecastMatrix dhi_lut,
                     double control_stop_charge_time, double sim_start_soc,
                     util::type::Coord sim_start_coord,
                     util::type::Time sim_start_time,
                     util::type::Time day_one_start_time,
                     util::type::Time day_one_end_time,
                     util::type::Time day_start_time,
                     util::type::Time day_end_time,
                     util::type::Time race_end_time)
      : SimulatorParams(std::move(wind_speed_lut), std::move(wind_dir_lut),
                        std::move(dni_lut), std::move(dhi_lut)),
        control_stop_charge_time(control_stop_charge_time),
        sim_start_soc(sim_start_soc),
        sim_start_coord(sim_start_coord),
        sim_start_time(sim_start_time),
        day_one_start_time(day_one_start_time),
        day_one_end_time(day_one_end_time),
        day_start_time(day_start_time),
        day_end_time(day_end_time),
        race_end_time(race_end_time) {}
};

inline WSCSimulatorParams get_wsc_simulator_params(ConfigParser* parser) {
  RUNTIME_EXCEPTION(
      parser != nullptr,
      "Config parser is null when loading WSC simulation parameters");

  WSCSimulatorParams params{
      ForecastMatrix(parser->get_wind_speed_path()),
      ForecastMatrix(parser->get_wind_direction_path()),
      ForecastMatrix(parser->get_dni_path()),
      ForecastMatrix(parser->get_dhi_path()),
      util::constants::mins2secs(parser->get_control_stop_charge_time()),
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

class WSCSimulator : public Simulator<WSCSimulator, WSCRacePlan, WSCRoute> {
 private:
  WSCSimulatorParams params;

 public:
  /* Load all LUTs upon construction */
  explicit WSCSimulator(WSCSimulatorParams params, Car model);

  /** @brief Run a full simulation with a car object and a route
   *
   * @param route: The Route to simulate on
   * @param race_plan: The race plan to simulate
   * @param results_lut: ResultsLut object for writing simulation result
   */
  void run_sim_impl(const WSCRoute& route, WSCRacePlan* race_plan,
                    Luts::DataSet* results_lut) const;

  void update_metrics_impl(LogMetrics* metrics, const CarUpdate& update,
                           util::type::Irradiance irr, double battery,
                           double delta_enetry, double distance,
                           util::type::Coord coord, double curr_speed,
                           const util::type::Time& curr_time,
                           double acceleration) const;
};
