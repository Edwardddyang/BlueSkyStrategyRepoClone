/* Energy model of the solar car  */

#pragma once

#include <memory>

#include "SimUtils/Types.hpp"
#include "SimUtils/Luts.hpp"
#include "config/Config.hpp"
#pragma warning(push)
#pragma warning(disable : 4267) // Disable size_t to long warning
#include <libIntegrate/Integrate.hpp>
#pragma warning(pop)

using EffLut = Luts::KeyLut<double, double, double>;
using BasicLut = Luts::BaseLut<double>;

/* Description of an energy loss/gain */
struct EnergyUpdate {
  double power;  // Instataneous power drawn/generated in Watts
  double energy; // Energy lost or gained in kWh
  double force;  // Associated force in N

  EnergyUpdate(double power, double energy) : power(power), energy(energy), force(0.0) {}
  EnergyUpdate(double power, double energy, double force) : power(power), energy(energy),
                                                            force(force) {}
  EnergyUpdate() : power(0.0), energy(0.0), force(0.0) {}
};

/* Unit update of the car when travelling between two coordinates */
struct CarUpdate {
  EnergyUpdate aero;
  EnergyUpdate rolling;
  EnergyUpdate gravitational;
  EnergyUpdate array;
  EnergyUpdate acceleration;
  util::type::SolarAngle az_el;
  double motor_power;
  double motor_energy;
  double bearing;
  double electric;
  double delta_energy;
  double delta_distance;
  double delta_time;

  CarUpdate(EnergyUpdate aero = EnergyUpdate(),
            EnergyUpdate rolling = EnergyUpdate(),
            EnergyUpdate gravitational = EnergyUpdate(),
            EnergyUpdate array = EnergyUpdate(),
            EnergyUpdate accel = EnergyUpdate(),
            util::type::SolarAngle az_el = util::type::SolarAngle(),
            double motor_power = 0.0,
            double motor_energy = 0.0,
            double bearing = 0.0,
            double electric = 0.0,
            double delta_energy = 0.0,
            double delta_distance = 0.0,
            double delta_time = 0.0) : aero(aero), rolling(rolling), gravitational(gravitational),
            array(array), acceleration(accel), az_el(az_el), motor_power(motor_power), motor_energy(motor_energy),
            bearing(bearing), electric(electric), delta_energy(delta_energy), delta_distance(delta_distance),
            delta_time(delta_time) {}
};

/** Common geometry parameters that must be calculated when computing energy losses */
struct Geometry {
  double bearing;
  double sin_theta;
  double cos_theta;
  double distance;
  util::type::SolarAngle az_el;

  Geometry(double bearing = 0.0, double sin_theta = 0.0, double cos_theta = 0.0,
           double distance = 0.0, util::type::SolarAngle az_el = util::type::SolarAngle()) :
           bearing(bearing), sin_theta(sin_theta), cos_theta(cos_theta), distance(distance),
          az_el(az_el) {}
};

/** Motor energy loss and its components */
struct MotorEnergyLoss {
  double aero_loss;         // In kWh
  double rolling_loss;      // In kWh
  double gravity_loss;      // In kWh
  double acceleration_loss; // In kWh
  double motor_energy;      // In kWn

  MotorEnergyLoss(double aero_loss, double rolling_loss, double gravity_loss,
                  double acceleration_loss, double motor_energy) :
                  aero_loss(aero_loss), rolling_loss(rolling_loss), gravity_loss(gravity_loss),
                  acceleration_loss(acceleration_loss), motor_energy(motor_energy) {}
};

// All class members should be initialized when constructed
// to ensure thread safety
class Car {
 private:
  /* Car parameters */
  const double mass;                      // kg
  const double cda;                       // Unitless
  const double motor_efficiency;          // Unitless
  const double regen_efficiency;          // Unitless
  const double battery_efficiency;        // Unitless
  const double passive_electric_loss;     // Watts
  const double air_density;               // kg / m^3
  const double array_area;                // m^2
  const double max_soc;                   // kWh
  const double tire_pressure;             // bar
  const double array_efficiency;          // Unitless
  const double max_braking_force;         // Newtons
  const double max_motor_power;           // Watts
  const EffLut yint_rolling_resistance;   // Unitless
  const EffLut slope_rolling_resistance;  // s / m
  const BasicLut power_factors;           // m^2

  // Integrator for calculating acceleration energy losses
  _1D::SimpsonRule<double> integrator;

  // Number of data points to use during integration
  const int num_data_points_per_second = 10;

 public:
  Car();
 
  /** @brief Compute the aerodynamic loss over a time period
   *
   * @param speed: Speed of the car in m/s
   * @param car_bearing: Bearing of the car in degrees cw from north
   * @param wind: Wind direction [deg cw from north] and wind speed [m/s]
   * @param delta_time_s: Time interval in seconds
   */
  EnergyUpdate compute_aero_loss(double speed, double car_bearing,
                                 util::type::Wind wind, double delta_time_s) const;

  /** @brief Compute the rolling resistance loss over a time period
   *
   * @param speed: Speed of the car in m/s
   * @param delta_time: Time interval in seconds
   * @param cos_theta: cos(elevation angle of the slant)
   */
  EnergyUpdate compute_rolling_loss(double speed, double delta_time, double cos_theta) const;

  /** @brief Compute the gravitational loss over a time period
   *
   * @param distance: Distance along the slant of the hill in m
   * @param delta_time: Time interval in seconds
   * @param sin_theta: sin(elevation angle of the slant)
   */
  EnergyUpdate compute_gravitational_loss(double distance, double delta_time, double sin_theta) const;

  /** @brief Compute the electrical loss over a time period
   *
   * @param delta_time: Time interval in seconds
   */
  double compute_electric_loss(double delta_time) const;

  /** @brief Compute the array energy gains
   *
   * @param delta_time: Time interval in seconds
   * @param dni: Direct normal irradiance in W/m^2
   * @param dhi: Diffuse horizontal irradiance in W/m^2
   * @param az: Azimuth angle in degrees from true north
   * @param el: Elevation angle in degrees
   */
  EnergyUpdate compute_array_gain(double delta_time, util::type::Irradiance irr,
                                  double az, double el) const;

  /** @brief Calculate geometry parameters between two route coordinates
  * @param delta_distance Can be passed in if distances are pre-computed
  */
  Geometry calculate_geometry(util::type::Coord coord_one, util::type::Coord coord_two,
                              const util::type::Time& time, double delta_distance = -1.0) const;

  /** @brief Calculate motor energy loss */
  MotorEnergyLoss calculate_motor_loss(double init_speed, double acceleration, double delta_time,
                                       const Geometry& g, util::type::Wind wind) const;
  
  /** @brief Compute energy change when moving between two points in a straight line at
   * constant speed
   * 
   * @param coord_one: starting coordinate
   * @param coord_two: ending coordinate
   * @param speed: Speed of the car in m/s
   * @param time: current time at coord_one
   * @param wind, irr: Weather forecast taken at coord_one
   * @param distance: The distance between the coordinates. Can be passed in
   * if using pre-computed distances
   *
   * @return CarUpdate: State updates to the car after moving from coord_one to coord_two
  */
  CarUpdate compute_constant_travel_update(util::type::Coord coord_one,
                                           util::type::Coord coord_two,
                                           double speed,
                                           const util::type::Time& time,
                                           util::type::Wind wind,
                                           util::type::Irradiance irr,
                                           double distance = -1.0) const;

  /** @brief Compute enrgy change when moving between two points in a straight line
  * with some acceleration
  *
  * @param coord_one: Starting coordinate
  * @param coord_two: Ending coordinate
  * @param init_speed: Initial speed in m/s
  * @param acceleration: Acceleration in m/s^2
  * @param time: Current time at coord_one
  * @param wind, irr: Weather forecast taken at coord_one
  * @param delta_distance: The distance between the coordinates. Can be passed in
  * if using pre-computed distances
  */
  CarUpdate compute_acceleration_travel_update(util::type::Coord coord_one,
                                               util::type::Coord coord_two,
                                               double init_speed,
                                               double acceleration,
                                               const util::type::Time& time,
                                               util::type::Wind wind,
                                               util::type::Irradiance irr,
                                               double delta_distance = -1.0) const;

  /** @brief Compute energy change when moving two points in a straight line
  * with a component of acceleration and a component of constant speed
  *
  * @param coord_one: Starting coordinate
  * @param coord_two: Ending coordinate
  * @param init_speed: Initial speed in m/s
  * @param acceleration: Acceleration in m/s^2
  * @param time: Current time at coord_one
  * @param wind, irr: Weather forecast taken at coord_one
  * @param acceleration_distance: Distance spent accelerating
  * @param constant_distance: Distance spent at constant speed
  */
  CarUpdate compute_dual_travel_update(util::type::Coord coord_one,
                                       util::type::Coord coord_two,
                                       double init_speed,
                                       double acceleration,
                                       const util::type::Time& time,
                                       util::type::Wind wind,
                                       util::type::Irradiance irr,
                                       double acceleration_distance,
                                       double constant_distance) const;

  /** @brief Compute energy change during a static stop in kWh
   *
   * @param coord: Location of the car
   * @param irr: Irradiance of the sun
   * @param az_el: Azimuth/elevation angles of the sun
   * @param time: Time of day
   * @param charge_time: Length of static stop in seconds
   * @param irr: Irradiance of the sun (assumed to be constant over the entire stop)
   */
  double compute_static_energy(util::type::Coord coord,
                               util::type::Irradiance irr,
                               util::type::SolarAngle az_el,
                               const util::type::Time& time,
                               double charge_time) const;
};
