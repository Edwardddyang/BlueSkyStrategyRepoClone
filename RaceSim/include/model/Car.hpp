/*
Base class for force balance car models
*/

#pragma once

#include <memory>

#include "utils/Units.hpp"
#include "utils/Luts.hpp"
#include "config/Config.hpp"

// All class members should be initialized once when constructed to ensure
// thread safety
class Car {
 protected:
  /* Car parameters */
  double mass;
  EffLut yint_rolling_resistance;
  EffLut slope_rolling_resistance;
  BasicLut power_factors;
  double cda;
  double motor_efficiency;
  double regen_efficiency;
  double battery_efficiency;
  double passive_electric_loss;
  double air_density;
  double array_area;
  double max_soc;
  double tire_pressure;
  double array_efficiency;

 public:
  Car();

  // Below functions don't have to be implemented, but are the same for both V1Car and V2Car

  /** @brief Compute the aerodynamic loss over a time period
   *
   * @param speed: Speed of the car in m/s
   * @param car_bearing: Bearing of the car in degrees cw from north
   * @param wind: Wind direction [deg cw from north] and wind speed [m/s]
   * @param delta_time_s: Time interval in seconds
   */
  virtual EnergyChange compute_aero_loss(double speed, double car_bearing, Wind wind, double delta_time_s) {
    return EnergyChange();
  }

  /** @brief Compute the rolling resistance loss over a time period
   *
   * @param speed: Speed of the car in m/s
   * @param delta_time: Time interval in seconds
   * @param cos_theta: cos(elevation angle of the slant)
   */
  virtual EnergyChange compute_rolling_loss(double speed, double delta_time, double cos_theta) {
    return EnergyChange();
  }

  /** @brief Compute the gravitational loss over a time period
   *
   * @param distance: Distance along the slant of the hill in m
   * @param delta_time: Time interval in seconds
   * @param sin_theta: sin(elevation angle of the slant)
   */
  virtual EnergyChange compute_gravitational_loss(double distance, double delta_time, double sin_theta) {
    return EnergyChange();
  }

  /** @brief Compute the electrical loss over a time period
   *
   * @param delta_time: Time interval in seconds
   */
  virtual double compute_electric_loss(double delta_time) {
    return -1.0;
  }

  /** @brief Compute the array energy gains
   *
   * @param delta_time: Time interval in seconds
   * @param dni: Direct normal irradiance in W/m^2
   * @param dhi: Diffuse horizontal irradiance in W/m^2
   * @param az: Azimuth angle in degrees from true north
   * @param el: Elevation angle in degrees
   */
  virtual EnergyChange compute_array_gain(double delta_time, double dni, double dhi, double az, double el) {
    return EnergyChange();
  }

  /** @brief Compute energy change when moving between two points in a straight line 
   * 
   * @param coord_one: starting coordinate
   * @param coord_two: ending coordinate
   * @param init_speed: initial speed of the car
   * @param acceleration: acceleration of the car
   * @param time: current time at coord_one
   * @param Wind, Irradiance: Weather forecast taken at coord_one
   * @param delta_distance: The distance between the coordinates. Can be passed in
   * if using pre-computed distances
   *
   * @return CarUpdate: State updates to the car after moving from coord_one to coord_two
  */
  virtual CarUpdate compute_travel_update(Coord coord_one,
                                          Coord coord_two,
                                          double init_speed,
                                          double acceleration,
                                          Time* time,
                                          Wind wind,
                                          Irradiance irr,
                                          double distance = -1.0) {
    return CarUpdate();
  }

  /** @brief Compute energy change during a static stop
   *
   * @param coord: Location of the car
   * @param time: Time of day
   * @param charge_time: Length of static stop in seconds
   * @param irr: Irradiance of the sun (assumed to be constant over the entire stop)
   */
  virtual double compute_static_energy(Coord coord, Time* time, double charge_time, Irradiance irr) {
    return -1.0;
  }
};
