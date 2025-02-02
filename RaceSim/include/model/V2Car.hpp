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
  _1D::SimpsonRule<double> integrator;

 public:
    V2Car();

    // Compute energy change when moving between two points in a straight line.
    // Note that unlike V1Car, acceleration is not always 0
    // Implementation is based on https://www.overleaf.com/read/xzbrzqtbsrhn#2ac2ad
    CarUpdate compute_travel_update(Coord coord_one,
                                    Coord coord_two,
                                    double init_speed,
                                    double acceleration,
                                    Time* time,
                                    Wind wind,
                                    Irradiance irr,
                                    double distance = -1.0) override;
};
