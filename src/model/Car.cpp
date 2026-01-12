#include "model/Car.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "SimUtils/Constants.hpp"
#include "SimUtils/CustomException.hpp"
#include "SimUtils/Geography.hpp"
#include "SimUtils/Types.hpp"
#include "SimUtils/Utilities.hpp"

/* Load all LUTs and car parameters */
Car::Car(CarParams params) : params(std::move(params)) {}

EnergyUpdate Car::compute_aero_loss(double speed, double car_bearing,
                                    util::type::Wind wind,
                                    double delta_time) const {
  const double speed_relative_to_wind =
      util::geo::get_speed_relative_to_wind(speed, car_bearing, wind);  // m/s
  const double force = 0.5 * params.air_density * params.cda *
                       pow(speed_relative_to_wind, 2);  // N
  const double power = force * speed;                   // Watt = Newton * m/s
  const double energy = util::constants::watts2kwh(delta_time, power);  // kwh

  return EnergyUpdate(power, energy, force);
}

EnergyUpdate Car::compute_rolling_loss(double speed, double delta_time,
                                       double cos_theta) const {
  const double y_int_rr = params.yint_rolling_resistance.get_value(
      params.tire_pressure, util::constants::mps2kph(speed));
  const double slope_rr = params.slope_rolling_resistance.get_value(
      params.tire_pressure, util::constants::mps2kph(speed));

  const double rolling_coefficient =
      (y_int_rr + slope_rr * speed);  // Unitless, slope_rr is units of s / m
  const double normal_force =
      params.mass * util::constants::GRAVITY_ACCELERATION * cos_theta;  // N
  const double force = rolling_coefficient * normal_force;              // N
  const double power = force * speed;  // Watt = Newton * m/s
  const double energy = util::constants::watts2kwh(delta_time, power);  // kwh

  return EnergyUpdate(power, energy, force);
}

EnergyUpdate Car::compute_gravitational_loss(double speed, double delta_time,
                                             double sin_theta) const {
  const double force =
      params.mass * util::constants::GRAVITY_ACCELERATION * sin_theta;  // N
  const double power = force * speed;  // Watt = Newton * m/s
  const double energy = util::constants::watts2kwh(delta_time, power);  // kwh

  return EnergyUpdate(power, energy, force);
}

double Car::compute_electric_loss(double delta_time) const {
  const double energy =
      delta_time * params.passive_electric_loss;  // Joules = s * W
  return util::constants::joules2kwh(energy);
}

EnergyUpdate Car::compute_array_gain(double delta_time,
                                     util::type::Irradiance irr, double az,
                                     double el) const {
  const auto az_idx = static_cast<size_t>(round(az));
  const auto el_idx = static_cast<size_t>(round(el));
  const double power_factor =
      params.power_factors.get_value(el_idx, az_idx);  // NOLINT
  const double power =
      (power_factor * irr.dni) +
      (irr.dhi * params.array_efficiency * params.array_area);          // Watts
  const double energy = util::constants::watts2kwh(delta_time, power);  // kwh
  return EnergyUpdate(power, energy);
}

Geometry Car::calculate_geometry(util::type::Coord coord_one,
                                 util::type::Coord coord_two,
                                 const util::type::Time& time,
                                 double delta_distance) {
  const double bearing = util::geo::get_bearing(coord_one, coord_two);
  const util::type::SolarAngle az_el =
      util::geo::get_az_el_from_bearing(bearing, coord_one, time);
  const double delta_altitude = coord_two.alt - coord_one.alt;
  const double distance = delta_distance == -1.0
                              ? util::geo::get_distance(coord_one, coord_two)
                              : delta_distance;
  const double sin_angle = delta_altitude / distance;
  const double base_squared =
      distance * distance - delta_altitude * delta_altitude;
  if (base_squared < 0.0) {
    throw util::error::InvalidCalculation(
        "Negative square root in base calculation");
  }
  const double base_distance = std::sqrt(base_squared);
  const double cos_angle = base_distance / distance;

  return Geometry(bearing, sin_angle, cos_angle, distance, az_el);
}

MotorEnergyLoss Car::calculate_motor_loss(double init_speed,
                                          double acceleration,
                                          double delta_time, const Geometry& g,
                                          util::type::Wind wind) const {
  EnergyUpdate aero_loss;
  EnergyUpdate rolling_loss;
  EnergyUpdate gravity_loss;
  if (acceleration < 0.0) {
    // When decelerating, we must ensure that the maximum braking force is not
    // exceeded
    // Get power, set delta time = 0.0
    aero_loss = compute_aero_loss(init_speed, g.bearing, wind, 0.0);
    rolling_loss = compute_rolling_loss(init_speed, 0.0, g.cos_theta);
    gravity_loss = compute_gravitational_loss(init_speed, 0.0, g.sin_theta);

    const double resistive_force =
        aero_loss.force + rolling_loss.force + gravity_loss.force;
    const double braking_force =
        params.mass * std::abs(acceleration) - resistive_force;
    if (braking_force > params.max_braking_force) {
      throw util::error::InvalidCalculation("Braking force is too large");
    }

    // Deceleration draws nothing from the motor
    return MotorEnergyLoss(0.0, 0.0, 0.0, 0.0, 0.0);
  } else {
    // Forward acceleration
    const auto num_data_points =
        static_cast<size_t>(delta_time * num_data_points_per_second);

    // y-axes of integration in watts
    std::vector<double> aero_power_data(num_data_points, 0.0);
    std::vector<double> rolling_power_data(num_data_points, 0.0);
    std::vector<double> acceleration_power_data(num_data_points, 0.0);
    std::vector<double> gravity_power_data(num_data_points, 0.0);

    // x-axis of integration
    std::vector<double> timestep_data(num_data_points);

    const auto timestep = delta_time / static_cast<double>(num_data_points);
    double time = 0.0;

    /* Take steps from 0 to delta_time and compute the integral numerically */
    for (size_t i = 0; i < num_data_points; i++) {
      const double speed =
          util::calc_final_speed(init_speed, acceleration, time);
      time = time + timestep;

      if (speed == 0.0) {
        aero_power_data[i] = 0.0;
        rolling_power_data[i] = 0.0;
        gravity_power_data[i] = 0.0;
      } else {
        aero_loss = compute_aero_loss(speed, g.bearing, wind, timestep);
        rolling_loss = compute_rolling_loss(speed, timestep, g.cos_theta);
        gravity_loss = compute_gravitational_loss(speed, timestep, g.sin_theta);
        const double acceleration_power = params.mass * acceleration * speed;

        aero_power_data[i] = aero_loss.power;
        rolling_power_data[i] = rolling_loss.power;
        acceleration_power_data[i] = acceleration_power;
        gravity_power_data[i] = gravity_loss.power;
        const double instataneous_motor_power =
            aero_loss.power + rolling_loss.power + acceleration_power +
            gravity_loss.power;
        if (instataneous_motor_power > params.max_motor_power) {
          throw util::error::InvalidCalculation(
              "Maximum motor power exceeded during acceleration segment");
        }
      }

      timestep_data[i] = time;
    }

    const double aero_energy_loss =
        util::constants::joules2kwh(integrator(timestep_data, aero_power_data));
    const double rolling_energy_loss = util::constants::joules2kwh(
        integrator(timestep_data, rolling_power_data));
    const double acceleration_energy_loss = util::constants::joules2kwh(
        integrator(timestep_data, acceleration_power_data));
    const double gravitational_energy_loss = util::constants::joules2kwh(
        integrator(timestep_data, gravity_power_data));
    double motor_energy =
        (aero_energy_loss + rolling_energy_loss + acceleration_energy_loss +
         gravitational_energy_loss) /
        params.motor_efficiency;

    // Assume no regen
    motor_energy = std::max(motor_energy, 0.0);

    return MotorEnergyLoss(aero_energy_loss, rolling_energy_loss,
                           gravitational_energy_loss, acceleration_energy_loss,
                           motor_energy);
  }
}

CarUpdate Car::compute_constant_travel_update(
    util::type::Coord coord_one, util::type::Coord coord_two, double speed,
    const util::type::Time& time, util::type::Wind wind,
    util::type::Irradiance irr, double distance) const {
  // Get geometry of the segment
  const Geometry g = calculate_geometry(coord_one, coord_two, time, distance);
  const double delta_time = util::calc_time(speed, 0.0, g.distance);

  const double electric_loss = compute_electric_loss(delta_time);
  const EnergyUpdate array_gain =
      compute_array_gain(delta_time, irr, g.az_el.Az, g.az_el.El);

  const EnergyUpdate aero_loss =
      compute_aero_loss(speed, g.bearing, wind, delta_time);
  const EnergyUpdate rolling_loss =
      compute_rolling_loss(speed, delta_time, g.cos_theta);
  const EnergyUpdate gravity_loss =
      compute_gravitational_loss(speed, delta_time, g.sin_theta);

  double motor_power =
      aero_loss.power + rolling_loss.power + gravity_loss.power;
  double motor_energy =
      aero_loss.energy + rolling_loss.energy + gravity_loss.energy;

  if (motor_power > params.max_motor_power) {
    throw util::error::InvalidCalculation(
        "Maximum motor power exceeded during constant speed segment");
  }

  // If resistive forces are propelling the car forwards, assume that
  // energy/power is drawn
  if (motor_energy < 0.0) {
    motor_power = 0.0;
    motor_energy = 0.0;
  } else {
    motor_energy = motor_energy / params.motor_efficiency;
  }

  const double battery_energy_in =
      array_gain.energy * params.battery_efficiency;
  const double battery_energy_out =
      (motor_energy + electric_loss) / params.battery_efficiency;
  const double delta_battery = battery_energy_in - battery_energy_out;

  return CarUpdate(aero_loss, rolling_loss, gravity_loss, array_gain,
                   EnergyUpdate(), g.az_el, motor_power, motor_energy,
                   g.bearing, electric_loss, delta_battery, g.distance,
                   delta_time);
}

CarUpdate Car::compute_acceleration_travel_update(
    util::type::Coord coord_one, util::type::Coord coord_two, double init_speed,
    double acceleration, const util::type::Time& time, util::type::Wind wind,
    util::type::Irradiance irr, double delta_distance) const {
  // Get segment geometry
  const Geometry g =
      calculate_geometry(coord_one, coord_two, time, delta_distance);
  const double delta_time =
      util::calc_time(init_speed, acceleration, delta_distance);

  const double electric_loss = compute_electric_loss(delta_time);
  const EnergyUpdate array_gain =
      compute_array_gain(delta_time, irr, g.az_el.Az, g.az_el.El);

  const MotorEnergyLoss m =
      calculate_motor_loss(init_speed, acceleration, delta_time, g, wind);

  const double battery_energy_in =
      array_gain.energy * params.battery_efficiency;
  const double battery_energy_out =
      (m.motor_energy + electric_loss) / params.battery_efficiency;
  const double delta_battery = battery_energy_in - battery_energy_out;

  const double average_motor_power =
      util::constants::kwh2joules(m.motor_energy) / delta_time;

  const EnergyUpdate average_aero_loss(
      util::constants::kwh2joules(m.aero_loss) / delta_time, m.aero_loss);
  const EnergyUpdate average_rolling_loss(
      util::constants::kwh2joules(m.rolling_loss) / delta_time, m.rolling_loss);
  const EnergyUpdate average_acceleration_loss(
      util::constants::kwh2joules(m.acceleration_loss / delta_time),
      m.acceleration_loss);
  const EnergyUpdate average_gravity_loss(
      util::constants::kwh2joules(m.gravity_loss / delta_time), m.gravity_loss);

  return CarUpdate(average_aero_loss, average_rolling_loss,
                   average_gravity_loss, array_gain, average_acceleration_loss,
                   g.az_el, average_motor_power, m.motor_energy, g.bearing,
                   electric_loss, delta_battery, g.distance, delta_time);
}

CarUpdate Car::compute_dual_travel_update(
    util::type::Coord coord_one, util::type::Coord coord_two, double init_speed,
    double acceleration, const util::type::Time& time, util::type::Wind wind,
    util::type::Irradiance irr, double acceleration_distance,
    double constant_distance) const {
  // Get time and distance travelled by each component
  const double delta_distance = acceleration_distance + constant_distance;
  const double delta_time_acceleration =
      util::calc_time(init_speed, acceleration, acceleration_distance);
  const double final_acceleration_speed =
      util::calc_final_speed(init_speed, acceleration, delta_time_acceleration);
  const double delta_time_constant =
      util::calc_time_v(final_acceleration_speed, constant_distance);
  const double delta_time = delta_time_acceleration + delta_time_constant;

  const Geometry g =
      calculate_geometry(coord_one, coord_two, time, delta_distance);

  // These are not dependent on whether the car accelerates or travels at
  // constant speed
  const double electric_loss = compute_electric_loss(delta_time);
  const EnergyUpdate array_gain =
      compute_array_gain(delta_time, irr, g.az_el.Az, g.az_el.El);
  const double battery_energy_in =
      array_gain.energy * params.battery_efficiency;

  // Calculate acceleration component
  const MotorEnergyLoss m =
      calculate_motor_loss(init_speed, acceleration, delta_time, g, wind);
  const double motor_energy_out_acceleration = m.motor_energy;

  // Track average aero, rolling, gravitational, acceleration losses
  double aero_energy_loss = m.aero_loss;
  double rolling_energy_loss = m.rolling_loss;
  double gravity_energy_loss = m.gravity_loss;
  const double acceleration_energy_loss = m.acceleration_loss;

  // Calculate constant speed component
  const EnergyUpdate aero_loss = compute_aero_loss(
      final_acceleration_speed, g.bearing, wind, delta_time_constant);
  const EnergyUpdate rolling_loss = compute_rolling_loss(
      final_acceleration_speed, delta_time_constant, g.cos_theta);
  const EnergyUpdate gravity_loss = compute_gravitational_loss(
      final_acceleration_speed, delta_time_constant, g.sin_theta);

  const double motor_power =
      aero_loss.power + rolling_loss.power + gravity_loss.power;
  double motor_energy_out_constant =
      aero_loss.energy + rolling_loss.energy + gravity_loss.energy;

  aero_energy_loss += aero_loss.energy;
  rolling_energy_loss += rolling_loss.energy;
  gravity_energy_loss += gravity_loss.energy;

  if (motor_power > params.max_motor_power) {
    throw util::error::InvalidCalculation(
        "Maximum motor power exceeded during constant speed segment");
  }

  motor_energy_out_constant = std::max(0.0, motor_energy_out_constant);

  // Calculate net delta
  const double total_motor_energy =
      motor_energy_out_acceleration + motor_energy_out_constant;
  const double battery_energy_out =
      (total_motor_energy + electric_loss) / params.battery_efficiency;
  const double delta_battery = battery_energy_in - battery_energy_out;

  const double average_motor_power =
      util::constants::kwh2joules(total_motor_energy) / delta_time;

  const EnergyUpdate average_aero_loss(
      util::constants::kwh2joules(aero_energy_loss) / delta_time,
      aero_energy_loss);
  const EnergyUpdate average_rolling_loss(
      util::constants::kwh2joules(rolling_energy_loss) / delta_time,
      rolling_energy_loss);
  const EnergyUpdate average_acceleration_loss(
      util::constants::kwh2joules(acceleration_energy_loss /
                                  delta_time_acceleration),
      acceleration_energy_loss);
  const EnergyUpdate average_gravity_loss(
      util::constants::kwh2joules(gravity_energy_loss / delta_time),
      gravity_energy_loss);
  return CarUpdate(average_aero_loss, average_rolling_loss,
                   average_gravity_loss, array_gain, average_acceleration_loss,
                   g.az_el, average_motor_power, total_motor_energy, g.bearing,
                   electric_loss, delta_battery, g.distance, delta_time);
}

double Car::compute_static_energy(util::type::Irradiance irr,
                                  util::type::SolarAngle az_el,
                                  double charge_time) const {
  const double electric_loss = compute_electric_loss(charge_time);
  const EnergyUpdate array_gain =
      compute_array_gain(charge_time, irr, az_el.Az, az_el.El);

  const double battery_energy_in =
      array_gain.energy * params.battery_efficiency;
  const double battery_energy_out = electric_loss / params.battery_efficiency;

  const double delta_battery = battery_energy_in - battery_energy_out;

  return delta_battery;
}
