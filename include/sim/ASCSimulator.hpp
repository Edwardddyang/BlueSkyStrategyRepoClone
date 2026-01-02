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
#include "config/Config.hpp"

class ASCSimulator : public Simulator<ASCSimulator, ASCRacePlan, ASCRoute> {
protected:
	/* Information about the event */
	const int checkpoint_hold_time;             // Stop time at a control stop in seconds
	const double sim_start_soc;                 // Starting soc for the simulation. Retrieved from config
	const util::type::Coord sim_start_coord;    // Starting coordinates of the car
	const util::type::Time sim_start_time;      // Starting time of the simulation i.e. irl time
	const util::type::Time day_start_time;      // Start time from day 2 onwards in 24 hour local time
	const util::type::Time day_end_time;        // End time from day 2 onwards in 24 hour local time
	const util::type::Time race_end_time;       // End time of the entire race in 24 hour local time
	const double max_soc;                       // Maximum soc of the car retrieved from config

public:
	/* Load all LUTs upon construction */
	explicit ASCSimulator(Car model);

	/** @brief Run a full simulation with a car object and a route
	*
	* @param route: The Route to simulate on
	* @param race_plan: The race plan to use
	* @param result_lut: ResultsLut object for writing simulation result
	*/
	void run_sim_impl(const ASCRoute& route, ASCRacePlan* race_plan,
				            Luts::DataSet* result_lut);

};
