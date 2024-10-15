/*
Model of the car implemented for gen 11.5
*/
#pragma once

#include "model/Car.hpp"
#include "utils/Units.hpp"

class V1Car : public Car {
 private:
  /* Maximum power of the battery pack in kwh */
  double max_power;

  /* Compute the net battery energy change */
  double compute_net_battery_change(double array, double aero, double rolling,
                                    double gravity, double electric, double motor);

 public:
    V1Car();

    /* Compute the aerodynamic loss */
    EnergyChange compute_aero_loss(double speed, double car_bearing, Wind wind, double delta_time) override;

    /* Compute the rolling resistance loss */
    EnergyChange compute_rolling_loss(double speed, double delta_time) override;

    /* Compute the gravitational loss */
    EnergyChange compute_gravitational_loss(double delta_altitude, double delta_time) override;

    /* Compute electric loss */
    double compute_electric_loss(double delta_time) override;

    /* Compute the array energy gains */
    EnergyChange compute_array_gain(double delta_time, double dni, double dhi, double az, double el) override;

    /* Compute energy change when moving between two points in a straight line */
    CarUpdate compute_travel_update(Coord coord_one,
                                    Coord coord_two,
                                    double speed,
                                    Time* time,
                                    Wind wind,
                                    Irradiance irr) override;

    /* Compute energy change during a static stop */
    double compute_static_energy(Coord coord, Time* time, double charge_time, Irradiance irr) override;
};
