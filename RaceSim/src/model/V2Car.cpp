#include <algorithm>
#include <vector>

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
  double delta_altitude = coord_two.alt - coord_one.alt;

  EnergyChange aero_loss;
  EnergyChange rolling_loss;
  EnergyChange gravity_loss;
  const double electric_loss = compute_electric_loss(delta_time);
  const EnergyChange array_gain = compute_array_gain(delta_time, irr.dni, irr.dhi, az_el.Az, az_el.El);

  if (acceleration < 0.0) {
    // When decelerating, we must ensure that the maximum braking force is not exceeded
    aero_loss = compute_aero_loss(init_speed, bearing, wind, 0.0);
    rolling_loss = compute_rolling_loss(init_speed, 0.0);
    gravity_loss = compute_gravitational_loss(delta_altitude, 0.0);

    const double resistive_force = aero_loss.force + rolling_loss.force + gravity_loss.force;
    const double braking_force = mass * std::abs(acceleration) - resistive_force;

    if (braking_force > max_braking_force) {
      throw InvalidCalculation("Braking force is too large");
    }

    const double delta_battery = -1.0 * electric_loss;
    return CarUpdate(EnergyChange(), EnergyChange(), EnergyChange(), array_gain, az_el, 0.0, 0.0, bearing,
                    electric_loss, delta_battery, delta_distance, delta_time);
  } else if (acceleration == 0.0) {
    aero_loss = compute_aero_loss(init_speed, bearing, wind, delta_time);
    rolling_loss = compute_rolling_loss(init_speed, delta_time);
    gravity_loss = compute_gravitational_loss(delta_altitude, delta_time);
    double motor_power = aero_loss.power + rolling_loss.power + gravity_loss.power;
    double motor_loss = aero_loss.energy + rolling_loss.energy + gravity_loss.energy;

    // If the energy draw is negative i.e. resistive forces propel the car forward, assume we don't
    // draw energy from the motor
    if (motor_loss < 0.0) {
      motor_loss = 0.0;
      motor_power = 0.0;
    }
    const double battery_energy_in = array_gain.energy * battery_efficiency;
    const double battery_energy_out = (motor_loss + electric_loss)  / battery_efficiency;

    const double delta_battery = battery_energy_in - battery_energy_out;

    return CarUpdate(aero_loss, rolling_loss, gravity_loss,
                    array_gain, az_el, motor_power, motor_loss,
                    bearing, electric_loss, delta_battery,
                    delta_distance, delta_time);
  } else {
    const int num_data_points = static_cast<int>(delta_time * num_data_points_per_second);
    std::vector<double> motor_power_data(num_data_points);
    std::vector<double> position_data(num_data_points);
    const double position_increments = delta_distance / num_data_points;
    double position = 0.0;
    double average_motor_power = 0.0;

    /* Take steps from 0 to delta_distance and compute the integral numerically */
    for (size_t i=0; i < num_data_points; i++) {
      double speed = std::sqrt(init_speed * init_speed + 2.0 * acceleration * position);

      if (speed == 0.0) {
        motor_power_data[i] = 0.0;
      } else {
        double timestep = position_increments / speed;
        aero_loss = compute_aero_loss(speed, bearing, wind, timestep);
        rolling_loss = compute_rolling_loss(speed, timestep);

        double altitude_timestep = (i / num_data_points) * delta_altitude;
        gravity_loss = compute_gravitational_loss(altitude_timestep, timestep);

        motor_power_data[i] = aero_loss.power + rolling_loss.power + gravity_loss.power;
      }

      average_motor_power += motor_power_data[i];
      position_data[i] = position;

      position = position + position_increments;
    }

    average_motor_power = average_motor_power / num_data_points;

    // Integral of power over distance gives units in joules, need to convert to kwh
    double motor_energy = joules2kwh(integrator(position_data, motor_power_data)) / motor_efficiency;
    if (motor_energy < 0.0) {
      motor_energy = 0.0;
    }

    const double battery_energy_in = array_gain.energy * battery_efficiency;
    const double battery_energy_out = (motor_energy + electric_loss)  / battery_efficiency;

    const double delta_battery = battery_energy_in - battery_energy_out;

    return CarUpdate(aero_loss, rolling_loss, gravity_loss,
                    array_gain, az_el, average_motor_power, motor_energy,
                    bearing, electric_loss, delta_battery,
                    delta_distance, delta_time);
  }
}

V2Car::V2Car() : V1Car() {tire_inertia = Config::get_instance()->get_tire_inertia();
                          num_tires = Config::get_instance()->get_num_tires();
                          tire_radius = Config::get_instance()->get_tire_radius();
                          max_braking_force = Config::get_instance()->get_max_deceleration() * mass;}
