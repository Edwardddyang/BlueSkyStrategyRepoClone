/* Wraps the c++ time data structures 
   Note: We use the date and chrono libraries in order to make conversions between tm and time_t 
   representations since  mktime in c++ implicitly does a timezone conversion based on the user's machine.
*/

#pragma once

#include <chrono>
#include <ctime>
#include <iostream>
#include <string>

#include "date/date.h"
#include "utils/Defines.hpp"

class Time {
 private:
  /* C++ tm structs for both the local time and utc timepoint */
  tm m_datetime_local;
  tm m_datetime_utc;

  /* Seconds since epoch beginning for both the local time and utc timepoint */
  time_t t_datetime_local;
  time_t t_datetime_utc;

  /* Special field for milliseconds since the tm struct's resolution is up to seconds */
  double m_milliseconds;

  /* UTC Adjustment in seconds */
  double utc_adjustment;

  /* Indication for a HH:MM:SS only timestamp - time_t and m_datetime_utc members are
   * interpreted as meaningless
   */
  bool hh_mm_ss_only;

  std::string local_time;

  /** @brief Construct a Time object from a string in HH:MM:SS format 
   * Note: time_t members will not be set and have garbage data
   * @param local_time_point: String in HH:MM:SS local 24 hour format
  */
  void HH_MM_SS_constructor(const std::string local_time_point);

 public:
  Time() {}

  /** @brief Create time with specific starting time
   *
   * @param local_time_point string in YYYY-MM-DD HH:MM:SS local 24 hour time format
   * @param utc_adjustment Adjustment in hours from local time to utc e.g. If the time
   * being represented is 10:30am in Alice Springs which is 9.5 hours ahead of UTC, then
   * this should be -9.5
  */
  Time(std::string local_time_point, double utc_adjustment);

  /** 
   * Create time without YYYY/MM/DD information. This will only fill in the m_datetime_local struct
   *
   * @param local_time_point string in HH:MM:SS local 24 hour time format
  */
  explicit Time(std::string local_time_point);

  /** @brief Copy the HH:MM:SS from another Time object to this one
   * Note: This Time object cannot be a hh_mm_ss_only type timestamp
   */
  void copy_hh_mm_ss(const Time& other);

  /** Return true if the lhs local timestamp is ahead of the rhs local timestamp */
  bool operator>(const Time& other) const;
  static bool gt(const Time* lhs, const Time* rhs);

  /** Return true if the rhs local timestamp is ahead of the lhs local timestamp */
  bool operator<(const Time& other) const;
  static bool lt(const Time* lhs, const Time* rhs);

  /** Return true if the lhs local timestamp is ahead of or equal to the rhs local timestamp */
  bool operator>=(const Time& other) const;

  /** Return true if the rhs local timestamp is ahead of or equal to the lhs local timestamp */
  bool operator<=(const Time& other) const;

  /** @brief Return the difference between two Time objects in seconds */
  double operator-(const Time& other) const;

  /** Get an epoch timestamp representing the utc time */
  inline time_t get_utc_time_point() const { return t_datetime_utc; }

  /** Get an epoch timestamp representing the local time */
  inline time_t get_local_time_point() const { return t_datetime_local; }

  /** Get c++ tm struct of the current utc time */
  inline tm get_utc_tm() const { return m_datetime_utc; }

  /** Get c++ tm struct of the current local time */
  inline tm get_local_tm() const { return m_datetime_local; }

  /** Get the current local time formatted according to the string in SolCast CSVs YYYYMMDDHHMM00 */
  uint64_t get_forecast_csv_time() const;

  /** @brief Advance the current time by a certain number of seconds
   *
   * @param seconds: Number of seconds to advance the current timestamp
  */
  void update_time_seconds(const double seconds);

  /** @brief Same function as update_time_seconds but returns a new Time object */
  Time operator+(const double seconds) const;

  /** @brief Set the tm structs
   * 
   * Note: No checks are performed to ensure validity
   * 
   * @param curr_time_local new local time
   * @param curr_time_utc corresponding utc time
  */
  void update_current_time(tm curr_time_local, tm curr_time_utc) {
    m_datetime_local = curr_time_local; m_datetime_utc = curr_time_utc;
  }

  /** Print human readable local time */
  void print_local_readable_time() const {std::cout << get_local_readable_time() << std::endl;}

  /** Print human readable utc time */
  void print_utc_readable_time() const {std::cout << get_utc_readable_time() << std::endl;}

  /** Get human readable local time as a string */
  std::string get_local_readable_time() const;

  /** Get human readable utc time as a string */
  std::string get_utc_readable_time() const;
};
