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
#include "config/ConfigParser.hpp"

using ForecastMatrix = Luts::ForecastLut<double, util::type::Time>;
inline constexpr uint32_t SECONDS_IN_DAY = 86400;

// Logs to keep track of race plan metrics
struct LogMetrics {
  std::vector<double> battery;
  std::vector<double> accumulated_distance;
  std::vector<util::type::Time> timestamp;
  std::vector<double> azimuth;
  std::vector<double> elevation;
  std::vector<double> bearing;
  std::vector<double> latitude;
  std::vector<double> longitude;
  std::vector<double> altitude;
  std::vector<double> speed;
  std::vector<double> acceleration;
  std::vector<double> dni;
  std::vector<double> dhi;
  std::vector<double> array_power;
  std::vector<double> array_energy;
  std::vector<double> aero_power;
  std::vector<double> aero_energy;
  std::vector<double> roll_power;
  std::vector<double> roll_energy;
  std::vector<double> gravity_power;
  std::vector<double> gravity_energy;
  std::vector<double> acceleration_power;
  std::vector<double> acceleration_energy;
  std::vector<double> electric_energy;
  std::vector<double> motor_power;
  std::vector<double> motor_energy;
  std::vector<double> delta_battery;

  /** @brief Update simulation metrics */
  void update_metrics(const CarUpdate& update, util::type::Irradiance irr, double battery_energy,
                      double delta_energy, double distance, util::type::Coord coord, double curr_speed,
                      const util::type::Time& curr_time, double accel) {
    battery.push_back(battery_energy);
    accumulated_distance.push_back(distance);
    timestamp.push_back(curr_time);
    azimuth.push_back(update.az_el.Az);
    elevation.push_back(update.az_el.El);
    bearing.push_back(update.bearing);
    latitude.push_back(coord.lat);
    longitude.push_back(coord.lon);
    altitude.push_back(coord.alt);
    speed.push_back(curr_speed);
    acceleration.push_back(accel);
    dni.push_back(irr.dni);
    dhi.push_back(irr.dhi);
    array_power.push_back(update.array.power);
    array_energy.push_back(update.array.energy);
    aero_power.push_back(update.aero.power);
    aero_energy.push_back(update.aero.energy);
    roll_power.push_back(update.rolling.power);
    roll_energy.push_back(update.rolling.energy);
    gravity_power.push_back(update.gravitational.power);
    gravity_energy.push_back(update.gravitational.energy);
    acceleration_power.push_back(update.acceleration.power);
    acceleration_energy.push_back(update.acceleration.energy);
    electric_energy.push_back(update.electric);
    motor_power.push_back(update.motor_power);
    motor_energy.push_back(update.motor_energy);
    delta_battery.push_back(delta_energy);
  }

  /** @brief Register metrics with dataset */
  void register_dataset(Luts::DataSet* dataset) {
    RUNTIME_EXCEPTION(dataset != nullptr, "Cannot register in null dataset");
    dataset->register_column("Battery Charge(kWh)", battery);
    dataset->register_column("Accumulated Distance(m)", accumulated_distance);
    dataset->register_column("DateTime", timestamp);
    dataset->register_column("Azimuth(Degrees)", azimuth);
    dataset->register_column("Elevation(Degrees)", elevation);
    dataset->register_column("Bearing(Degrees)", bearing);
    dataset->register_column("Latitude", latitude);
    dataset->register_column("Longitude", longitude);
    dataset->register_column("Altitude(m)", altitude);
    dataset->register_column("Speed(m/s)", speed);
    dataset->register_column("Acceleration(m/s^2)", acceleration);
    dataset->register_column("DNI(W/m^2)", dni);
    dataset->register_column("DHI(W/m^2)", dhi);
    dataset->register_column("Array Power(W)", array_power);
    dataset->register_column("Array Energy(kWh)", array_energy);
    dataset->register_column("Aero Power(W)", aero_power);
    dataset->register_column("Aero Energy(kWh)", aero_energy);
    dataset->register_column("Roll Power(W)", roll_power);
    dataset->register_column("Roll Energy(kWh)", roll_energy);
    dataset->register_column("Gravity Power(W)", gravity_power);
    dataset->register_column("Gravity Energy(kWh)", gravity_energy);
    dataset->register_column("Acceleration Power(W)", acceleration_power);
    dataset->register_column("Acceleration Energy(kWh)", acceleration_energy);
    dataset->register_column("Electric Energy(kWh)", electric_energy);
    dataset->register_column("Motor Power(W)", motor_power);
    dataset->register_column("Motor Energy(kWh)", motor_energy);
    dataset->register_column("Delta Battery(kWh)", delta_battery);
  }
};

/** Injected simulation parameters */
struct SimulatorParams {
  const ForecastMatrix wind_speed_lut;
  const ForecastMatrix wind_dir_lut;
  const ForecastMatrix dni_lut;
  const ForecastMatrix dhi_lut;

  SimulatorParams(ForecastMatrix wind_speed,
                  ForecastMatrix wind_dir,
                  ForecastMatrix dni,
                  ForecastMatrix dhi) :
    wind_speed_lut(std::move(wind_speed)),
    wind_dir_lut(std::move(wind_dir)),
    dni_lut(std::move(dni)), dhi_lut(std::move(dhi)) {}
};

/** Fill simulation parameters from configuration file */
inline SimulatorParams get_simulator_params(ConfigParser* parser) {
  RUNTIME_EXCEPTION(parser != nullptr, "Config parser is null when loading simulation parameters");
  SimulatorParams parameters{
    ForecastMatrix(parser->get_wind_speed_path()),
    ForecastMatrix(parser->get_wind_direction_path()),
    ForecastMatrix(parser->get_dni_path()),
    ForecastMatrix(parser->get_dhi_path())
  };
  return parameters;
}

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
	explicit Simulator(Car model) : car(std::move(model)) {}

	/** @brief Run a full simulation with a car object and a route
	*
	* @param route: The Route to simulate on
	* @param race_plan: The race plan to use
	* @param result_lut: ResultsLut object for storing results
	*/
	void run_sim(const Route& route,
               RacePlan* race_plan,
	             Luts::DataSet* result_lut) const {
    static_cast<const Derived*>(this)->run_sim_impl(route, race_plan, result_lut);
  }
};
