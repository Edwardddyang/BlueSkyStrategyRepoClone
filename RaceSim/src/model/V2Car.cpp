#include <algorithm>
#include <vector>

#include "model/V2Car.hpp"

double V2Car::calculate_acceleration_time(double v1, double a, double d) {
  double discriminant = v1 * v1 + 2 * a * d;
  RUNTIME_EXCEPTION(discriminant >= 0, "Acceleration formula error");
  double t1 = (-v1 + std::sqrt(discriminant)) / a;
  double t2 = (-v1 - std::sqrt(discriminant)) / a;
  // Return the positive time
  if (t1 > 0 && t2 > 0) {
    return std::min(t1, t2);  // Both are positive, return the smaller one
  } else if (t1 > 0) {
    return t1;
  } else if (t2 > 0) {
    return t2;
  }

  RUNTIME_EXCEPTION(false, "Acceleration calculation failed.");
}

CarUpdate V2Car::compute_travel_update(Coord coord_one,
                                       Coord coord_two,
                                       double init_speed,
                                       double acceleration,
                                       Time* time,
                                       Wind wind,
                                       Irradiance irr) {
  /* Get orientation of the car */
  const double bearing = get_bearing(coord_one, coord_two);
  SolarAngle az_el = get_az_el_from_bearing(bearing, coord_one, time);
  if (az_el.El < 0) {
    az_el.El = 0.0;
  }

  /* Get time and distance travelled */
  const double delta_distance = get_distance(coord_one, coord_two);
  const double delta_time = calculate_acceleration_time(init_speed, acceleration, delta_distance);
  double delta_altitude = coord_two.alt - coord_one.alt;

  const int num_seconds = 0.0 < delta_time < 1.0 ? 1 : static_cast<int>(delta_time);
  const int num_data_points = num_seconds * num_data_points_per_second;
  std::vector<double> motor_power_data(num_data_points);
  std::vector<double> position_data(num_data_points);
  double position = 0.0;
  const double position_increments = delta_distance / num_data_points;
  double average_motor_power = 0.0;

  EnergyChange aero_loss;
  EnergyChange rolling_loss;
  EnergyChange gravity_loss;

  /* Take steps from 0 to delta_distance and compute the integral numerically */
  double electric_loss = compute_electric_loss(delta_time);
  for (size_t i=0; i < num_data_points; i++) {
    double speed = std::sqrt(init_speed * init_speed + 2.0 * acceleration * position);
    double timestep = position_increments / speed;
    aero_loss = compute_aero_loss(speed, bearing, wind, timestep);
    rolling_loss = compute_rolling_loss(speed, timestep);

    double altitude_timestep = (i / num_data_points) * delta_altitude;
    gravity_loss = compute_gravitational_loss(altitude_timestep, timestep);

    motor_power_data[i] = aero_loss.power + rolling_loss.power + gravity_loss.power;

    average_motor_power += motor_power_data[i];
    position_data[i] = position;

    position = position + position_increments;
  }

  average_motor_power = average_motor_power / num_data_points;
  double motor_energy = integrator(position_data, motor_power_data);

  // Calculate net battery change
  double motor_energy_draw = (1.0 / motor_efficiency) * motor_energy;
  EnergyChange array_gain = compute_array_gain(delta_time, irr.dni, irr.dhi, az_el.Az, az_el.El);

  double battery_energy_in = array_gain.energy * battery_efficiency;
  double battery_energy_out = (motor_energy_draw + electric_loss) * (1.0 / battery_efficiency);

  double delta_battery = battery_energy_in - battery_energy_out;

  return CarUpdate(aero_loss, rolling_loss, gravity_loss,
                  array_gain, az_el, average_motor_power, motor_energy_draw,
                  bearing, electric_loss, delta_battery,
                  delta_distance, delta_time);
}

V2Car::V2Car() : V1Car() {tire_inertia = Config::get_instance()->get_tire_inertia();
                          num_tires = Config::get_instance()->get_num_tires();
                          tire_radius = Config::get_instance()->get_tire_radius();}
