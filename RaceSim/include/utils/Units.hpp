/*
Definitions and functions for scientific units, custom objects, conversion functions and utility functions
*/

#pragma once

#include <time.h>

#include <vector>
#include <string>
#include <ctime>
#include <iostream>
#include <chrono>

#include "date/date.h"

// Forward declarations
struct ForecastCoord;
struct Coord;

/* Constants */
#define PI (3.14159265358979323846264338327950288)
#define DEGREES_IN_PI (180)
#define HOURS_TO_SECONDS (3.6e3)
#define MINUTES_TO_SECONDS 60.0
#define HOURS_TO_MINUTES 60.0
#define JOULES_TO_KWH (3.6e6)
#define MPS_TO_KPH (3.6)
#define GRAVITY_ACCELERATION (9.81)
#define KM_TO_M (1000.0)
#define SECONDS_IN_DAY 86400

/* Custom units */

/* A forecast coordinate is represented only by latitude and longitude */
struct ForecastCoord {
  double lat;
  double lon;

  ForecastCoord(double latitude, double longitude) : lat(latitude), lon(longitude) {}
  ForecastCoord() : lat(0), lon(0) {}
};

/* Standard latitude/latitude/altitude coordinate */
struct Coord {
  double lat;
  double lon;
  double alt;

  Coord(double latitude, double longitude, double altitude) : lat(latitude), lon(longitude), alt(altitude) {}
  Coord(double latitude, double longitude) : lat(latitude), lon(longitude), alt(0.0) {}
  Coord() : lat(0), lon(0), alt(0) {}

  /* Conversion operator from type ForecastCoord to Coord by clearing the altitude value */
  explicit Coord(const struct ForecastCoord& fc) : lat(fc.lat), lon(fc.lon), alt(0.0) {}
};

struct Wind {
  double bearing;  // degrees
  double speed;  // m/s

  Wind(double b, double s) : bearing(b), speed(s) {}
  Wind() : bearing(0), speed(0) {}
};

struct SolarAngle {
  double Az;
  double El;

  SolarAngle(double az_angle, double el_angle) : Az(az_angle), El(el_angle) {}
  SolarAngle() : Az(0.0), El(0.0) {}
};

struct TireCoeff {
  double yint;
  double slope;

  TireCoeff(double y, double s) : yint(y), slope(s) {}
  TireCoeff() : yint(0), slope(0) {}
};

struct Irradiance {
  double dni;
  double dhi;

  Irradiance(double _dni, double _dhi) : dni(_dni), dhi(_dhi) {}
  Irradiance() : dni(0), dhi(0) {}
};

/* Each energy loss/gain (e.g. rolling resistance, aerodynamic, gravity) is characterized by both its
   actual energy lost/gained and the instataneous power drawn/generated
*/
struct EnergyChange {
  double power;  // In W
  double energy;  // In kWh
  double force;  // In N

  EnergyChange(double power, double energy) : power(power), energy(energy), force(0.0) {}
  EnergyChange(double power, double energy, double force) : power(power), energy(energy),
                                                            force(force) {}
  EnergyChange() : power(0.0), energy(0.0), force(0.0) {}
};

/* Unit update of the car when travelling between two coordinates */
struct CarUpdate {
  EnergyChange aero;
  EnergyChange rolling;
  EnergyChange gravitational;
  EnergyChange array;
  EnergyChange acceleration;
  SolarAngle az_el;
  double motor_power;
  double motor_energy;
  double bearing;
  double electric;
  double delta_energy;
  double delta_distance;
  double delta_time;

  CarUpdate(EnergyChange aero = EnergyChange(),
            EnergyChange rolling = EnergyChange(),
            EnergyChange gravitational = EnergyChange(),
            EnergyChange array = EnergyChange(),
            EnergyChange accel = EnergyChange(),
            SolarAngle az_el = SolarAngle(),
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

/* Unit conversions */
inline double rad2deg(double radians) { return radians * DEGREES_IN_PI / PI; }
inline double deg2rad(double degrees) { return degrees * PI / DEGREES_IN_PI; }
inline double hours2secs(double hours) { return hours * HOURS_TO_SECONDS; }
inline double secs2hours(double seconds) { return seconds / HOURS_TO_SECONDS; }
inline double mins2secs(double minutes) {return minutes * MINUTES_TO_SECONDS; }
inline double secs2mins(double seconds) {return seconds / MINUTES_TO_SECONDS; }
inline double kph2mps(double kph) { return kph / MPS_TO_KPH; }
inline double mps2kph(double mps) { return mps * MPS_TO_KPH; }
inline double joules2kwh(double joules) { return joules / JOULES_TO_KWH; }
inline double watts2kwh(double time, double watts) { return watts * (secs2hours(time) / 1000.0);}
inline double joules2watts(double joules, double time) { return joules / time; }
inline double meters2km(double m) {return m/KM_TO_M;}
inline double km2meters(double km) {return km * KM_TO_M;}
inline double kwh2joules(double kwh) { return kwh * JOULES_TO_KWH; }
inline double kwh2watts(double kwh) { return kwh * 1000.0; }
