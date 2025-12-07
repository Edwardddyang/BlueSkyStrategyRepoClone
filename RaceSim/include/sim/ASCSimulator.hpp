//
// ASC race simulator
//

#ifndef RACESIM_ASCSIMULATOR_H
#define RACESIM_ASCSIMULATOR_H

#pragma once

#include <stdbool.h>
#include <string>
#include <memory>
#include <vector>

#include "sim/Simulator.hpp"
#include "route/Route.hpp"
#include "model/Car.hpp"
#include "utils/Units.hpp"
#include "utils/Luts.hpp"
#include "config/Config.hpp"

class ASCSimulator : public SimulatorBaseCrtp<ASCSimulator, ASCRoute> {
protected:
	/* Information about the event */
	const int checkpoint_hold_time;  // Stop time at a control stop in seconds
	const double sim_start_soc;          // Starting soc for the simulation. Retrieved from config
	const Coord sim_start_coord;         // Starting coordinates of the car
	const Time sim_start_time;           // Starting time of the simulation i.e. irl time
	const Time day_start_time;           // Start time from day 2 onwards in 24 hour local time
	const Time day_end_time;             // End time from day 2 onwards in 24 hour local time
	const Time race_end_time;            // End time of the entire race in 24 hour local time
	const double max_soc;                // Maximum soc of the car retrieved from config

public:
	/* Load all LUTs upon construction */
	explicit ASCSimulator(std::shared_ptr<Car> model);

	/** @brief Run a full simulation with a car object and a route
	*
	* @param route: The Route to simulate on
	* @param race_plan: The race plan to use
	* @param result_lut: ResultsLut object for writing simulation result
	*/
	void run_sim_impl(std::shared_ptr<ASCRoute> route, RacePlan* race_plan,
				 std::shared_ptr<ResultsLut> result_lut);

};

#endif //RACESIM_ASCSIMULATOR_H