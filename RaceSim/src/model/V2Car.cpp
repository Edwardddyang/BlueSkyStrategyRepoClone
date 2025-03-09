#include <algorithm>
#include <vector>
#include <iostream>
#include <fstream>

#include "model/V2Car.hpp"
#include "utils/CustomException.hpp"

CarUpdate V2Car::compute_travel_update(Coord coord_one,
                                       Coord coord_two,
                                       double init_speed,
                                       double acceleration,
                                       Time* time,
                                       Wind wind,
                                       Irradiance irr,
                                       double distance) {
  /* Get orientation of the car */
  const double bearing = get_bearing(coord_one, coord_two);
  SolarAngle az_el = get_az_el_from_bearing(bearing, coord_one, time);
  if (az_el.El < 0) {
    az_el.El = 0.0;
  }

  /* Get time and distance travelled */
  const double delta_distance = distance == -1.0 ? get_distance(coord_one, coord_two) : distance;
  const double delta_time = calc_time(init_speed, acceleration, delta_distance);
  const double delta_altitude = coord_two.alt - coord_one.alt;
  const double sin_angle = delta_altitude / delta_distance;

  EnergyChange aero_loss;
  EnergyChange rolling_loss;
  EnergyChange gravity_loss;
  const double electric_loss = compute_electric_loss(delta_time);
  const EnergyChange array_gain = compute_array_gain(delta_time, irr.dni, irr.dhi, az_el.Az, az_el.El);

  if (acceleration < 0.0) {
    // When decelerating, we must ensure that the maximum braking force is not exceeded
    // Use init_speed since it will be the maximum speed
    aero_loss = compute_aero_loss(init_speed, bearing, wind, 0.0);
    rolling_loss = compute_rolling_loss(init_speed, 0.0);
    gravity_loss = compute_gravitational_loss(delta_distance, 0.0, sin_angle);

    const double resistive_force = aero_loss.force + rolling_loss.force + gravity_loss.force;
    const double braking_force = mass * std::abs(acceleration) - resistive_force;

    if (braking_force > max_braking_force) {
      throw InvalidCalculation("Braking force is too large");
    }

    const double delta_battery = -1.0 * electric_loss;
    return CarUpdate(EnergyChange(), EnergyChange(), EnergyChange(), array_gain, EnergyChange(), az_el,
                    0.0, 0.0, bearing, electric_loss, delta_battery, delta_distance, delta_time);
  } else if (acceleration == 0.0) {
    aero_loss = compute_aero_loss(init_speed, bearing, wind, delta_time);
    rolling_loss = compute_rolling_loss(init_speed, delta_time);
    gravity_loss = compute_gravitational_loss(delta_distance, delta_time, sin_angle);
    double motor_power = aero_loss.power + rolling_loss.power + gravity_loss.power;
    double motor_loss = aero_loss.energy + rolling_loss.energy + gravity_loss.energy;

    // If the energy draw is negative i.e. resistive forces propel the car forward, assume we don't
    // draw energy from the motor
    if (motor_loss < 0.0) {
      motor_loss = 0.0;
      motor_power = 0.0;
    } else {
      motor_loss = motor_loss / motor_efficiency;
    }

    const double battery_energy_in = array_gain.energy * battery_efficiency;
    const double battery_energy_out = (motor_loss + electric_loss)  / battery_efficiency;

    const double delta_battery = battery_energy_in - battery_energy_out;

    return CarUpdate(aero_loss, rolling_loss, gravity_loss,
                    array_gain, EnergyChange(), az_el, motor_power, motor_loss,
                    bearing, electric_loss, delta_battery,
                    delta_distance, delta_time);
  } else {
    const int num_data_points = static_cast<int>(delta_time * num_data_points_per_second);

    // y-axes of integration (in watts)
    std::vector<double> aero_power_data(num_data_points, 0.0);
    std::vector<double> rolling_power_data(num_data_points, 0.0);
    std::vector<double> acceleration_power_data(num_data_points, 0.0);
    std::vector<double> gravity_power_data(num_data_points, 0.0);

    // x-axis of integration
    std::vector<double> timestep_data(num_data_points);

    const double timestep = delta_time / num_data_points;
    double time = 0.0;

    // This is constant for the duration of the acceleration
    gravity_loss = compute_gravitational_loss(delta_distance, delta_time, sin_angle);
    /* Take steps from 0 to delta_time and compute the integral numerically */
    for (size_t i=0; i < num_data_points; i++) {
      const double speed = calc_final_speed(init_speed, acceleration, time);
      time = time + timestep;

      if (speed == 0.0) {
        aero_power_data[i] = 0.0;
        rolling_power_data[i] = 0.0;
      } else {
        aero_loss = compute_aero_loss(speed, bearing, wind, timestep);
        rolling_loss = compute_rolling_loss(speed, timestep);
        const double acceleration_power = mass * acceleration * speed;

        aero_power_data[i] = aero_loss.power;
        rolling_power_data[i] = rolling_loss.power;
        acceleration_power_data[i] = acceleration_power;
        gravity_power_data[i] = gravity_loss.power;
      }

      timestep_data[i] = time;
    }

    const double aero_energy_loss = joules2kwh(integrator(timestep_data, aero_power_data));
    const double rolling_energy_loss = joules2kwh(integrator(timestep_data, rolling_power_data));
    const double acceleration_energy_loss = joules2kwh(integrator(timestep_data, acceleration_power_data));
    const double gravitational_energy_loss = joules2kwh(integrator(timestep_data, gravity_power_data));
    double motor_energy = (aero_energy_loss + rolling_energy_loss +
                          acceleration_energy_loss + gravitational_energy_loss)
                          / motor_efficiency;

    // Assume no regen
    if (motor_energy < 0.0) {
      motor_energy = 0.0;
    }

    const double battery_energy_in = array_gain.energy * battery_efficiency;
    const double battery_energy_out = (motor_energy + electric_loss)  / battery_efficiency;
    const double delta_battery = battery_energy_in - battery_energy_out;

    const double average_motor_power = kwh2joules(motor_energy) / delta_time;

    const EnergyChange average_aero_loss(kwh2joules(aero_energy_loss) / delta_time, aero_energy_loss);
    const EnergyChange average_rolling_loss(kwh2joules(rolling_energy_loss) / delta_time, rolling_energy_loss);
    const EnergyChange average_acceleration_loss(kwh2joules(acceleration_energy_loss / delta_time),
                                                  acceleration_energy_loss);
    const EnergyChange average_gravity_loss(kwh2joules(gravitational_energy_loss / delta_time),
                                            gravitational_energy_loss);

    return CarUpdate(average_aero_loss, average_rolling_loss, average_gravity_loss,
                    array_gain, average_acceleration_loss,
                    az_el, average_motor_power, motor_energy,
                    bearing, electric_loss, delta_battery,
                    delta_distance, delta_time);
  }
}

V2Car::V2Car() : V1Car() {tire_inertia = Config::get_instance()->get_tire_inertia();
                          num_tires = Config::get_instance()->get_num_tires();
                          tire_radius = Config::get_instance()->get_tire_radius();
                          max_braking_force = Config::get_instance()->get_max_deceleration() * mass;}
