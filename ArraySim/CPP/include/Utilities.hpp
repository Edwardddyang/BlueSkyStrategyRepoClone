#pragma once

#include <string>

#define PI (3.14159265358979323846264338327950288)
#define DEGREES_IN_PI (180)
#define HOURS_TO_SECONDS (3.6e3)
#define MINUTES_TO_SECONDS 60.0
#define HOURS_TO_MINUTES 60.0
#define MPS_TO_KPH (3.6)
#define GRAVITY_ACCELERATION (9.81)
#define KM_TO_M (1000.0)

inline double rad2deg(double radians) { return radians * DEGREES_IN_PI / PI; }
inline double deg2rad(double degrees) { return degrees * PI / DEGREES_IN_PI; }
inline double hours2secs(double hours) { return hours * HOURS_TO_SECONDS; }
inline double secs2hours(double seconds) { return seconds / HOURS_TO_SECONDS; }
inline double mins2secs(double minutes) {return minutes * MINUTES_TO_SECONDS; }
inline double secs2mins(double seconds) {return seconds / MINUTES_TO_SECONDS; }
inline double kph2mps(double kph) { return kph / MPS_TO_KPH; }
inline double mps2kph(double mps) { return mps * MPS_TO_KPH; }

bool isDouble(std::string str);
