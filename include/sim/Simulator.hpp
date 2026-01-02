/* 
Class to run the a full scale simulation on a designated route
*/

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
#include "config/Config.hpp"

using ForecastMatrix = Luts::ForecastLut<double, util::type::Time>;
inline constexpr uint32_t SECONDS_IN_DAY = 86400;

// Simulator interface
// All (derived) class members should be initialized exactly ONCE when the object is constructed.
// Afterwards, they should only be read. This is to ensure that the Simulator object can be
// shared between threads. Do not track the internal state of the simulation e.g. soc, speed
// using class members. I'll find you.
template <typename Derived, RacePlanType RacePlan, RouteType Route>
class Simulator {
 protected:
	/* Step size in seconds when charging */
	const int CHARGING_STEP_SIZE = 30;

	/* Weather forecasting LUTs */
	const ForecastMatrix wind_speed_lut;
	const ForecastMatrix wind_dir_lut;
	const ForecastMatrix dni_lut;
	const ForecastMatrix dhi_lut;

	/* Route to simulate on and the model to use */
	const Car car;

  /** @brief Get step size while the car is charging */
  inline double get_charging_step_size(const util::type::Time& curr_time,
                                       const util::type::Time& charge_end) const {
    const double remaining_time = charge_end - curr_time;
    if (remaining_time > CHARGING_STEP_SIZE) {
      return static_cast<double>(CHARGING_STEP_SIZE);
    } else {
      return remaining_time;
    }
  }

 public:
	/* Load all LUTs upon construction */
	explicit Simulator(Car model) :
          car(std::move(model)),
          wind_speed_lut(Config::get_instance().get_wind_speed_path()),
          wind_dir_lut(Config::get_instance().get_wind_direction_path()),
          dni_lut(Config::get_instance().get_dni_path()),
          dhi_lut(Config::get_instance().get_dhi_path()) {}

	/** @brief Run a full simulation with a car object and a route
	*
	* @param route: The Route to simulate on
	* @param race_plan: The race plan to use
	* @param result_lut: ResultsLut object for storing results
	*/
	void run_sim(const Route& route,
               RacePlan* race_plan,
	             Luts::DataSet& result_lut) const {
    static_cast<const Derived*>(this)->run_sim_impl(route, race_plan, result_lut);
  }
};
