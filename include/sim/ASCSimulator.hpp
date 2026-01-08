//
// ASC race simulator
//

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
#include "config/ConfigParser.hpp"

struct ASCSimulatorParams : SimulatorParams {
	const int checkpoint_hold_time;             // Stop time at a control stop in seconds
	const double sim_start_soc;                 // Starting soc for the simulation. Retrieved from config
	const util::type::Coord sim_start_coord;    // Starting coordinates of the car
	const util::type::Time sim_start_time;      // Starting time of the simulation i.e. irl time
	const util::type::Time day_start_time;      // Start time from day 2 onwards in 24 hour local time
	const util::type::Time day_end_time;        // End time from day 2 onwards in 24 hour local time
	const util::type::Time race_end_time;       // End time of the entire race in 24 hour local time

  ASCSimulatorParams(ForecastMatrix wind_speed_lut, ForecastMatrix wind_dir_lut,
                     ForecastMatrix dni_lut, ForecastMatrix dhi_lut,
                     int checkpoint_hold_time, double sim_start_soc,
                     util::type::Coord sim_start_coord,
                     util::type::Time sim_start_time,
                     util::type::Time day_start_time,
                     util::type::Time day_end_time,
                     util::type::Time race_end_time) :
      SimulatorParams(wind_speed_lut, wind_dir_lut, dni_lut, dhi_lut),
      checkpoint_hold_time(checkpoint_hold_time), sim_start_soc(sim_start_soc),
      sim_start_coord(sim_start_coord), sim_start_time(sim_start_time),
      day_start_time(day_start_time), day_end_time(day_end_time),
      race_end_time(race_end_time) {}
};

inline ASCSimulatorParams get_asc_simulator_params(ConfigParser* parser) {
  RUNTIME_EXCEPTION(parser != nullptr, "Config parser is null when loading ASC simulation parameters");

  ASCSimulatorParams params{
    ForecastMatrix(parser->get_wind_speed_path()),
    ForecastMatrix(parser->get_wind_direction_path()),
    ForecastMatrix(parser->get_dni_path()),
    ForecastMatrix(parser->get_dhi_path()),
    parser->get_checkpoint_hold_time(),
    parser->get_current_soc(),
    parser->get_gps_coordinates(),
    parser->get_current_date_time(),
    parser->get_day_start_time(),
    parser->get_day_end_time(),
    parser->get_race_end_time(),
  };

  return params;
}

class ASCSimulator : public Simulator<ASCSimulator, ASCRacePlan, ASCRoute> {
protected:
	ASCSimulatorParams params;

public:
	/* Load all LUTs upon construction */
	explicit ASCSimulator(ASCSimulatorParams params, Car model);

	/** @brief Run a full simulation with a car object and a route
	*
	* @param route: The Route to simulate on
	* @param race_plan: The race plan to use
	* @param result_lut: ResultsLut object for writing simulation result
	*/
	void run_sim_impl(const ASCRoute& route, ASCRacePlan* race_plan,
				            Luts::DataSet* result_lut);

};
