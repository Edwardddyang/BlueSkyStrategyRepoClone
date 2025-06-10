/*
Acceleration energy model of the car
*/
#pragma once

#include "model/V1Car.hpp"
#include "utils/Units.hpp"
#include "libIntegrate/Integrate.hpp"

class V2Car : public V1Car {
 private:
  const int num_data_points_per_second = 100;
  int num_tires;
  double tire_radius;
  double tire_inertia;
  double max_braking_force;
  double max_motor_power;  // In W
  _1D::SimpsonRule<double> integrator;

 public:
  V2Car();

  EnergyChange compute_fsgp_array_gain_ghi(double delta_time, Irradiance irr);
  // Compute
  CarUpdate compute_travel_update(Coord coord_one,
                                  Coord coord_two,
                                  double init_speed,
                                  double final_speed,
                                  double acceleration,
                                  Time* time,
                                  Wind wind,
                                  Irradiance irr,
                                  double acceleration_distance,
                                  double constant_distance);
};
