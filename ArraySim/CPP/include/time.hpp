/* Wraps the c++ time data structures 
   Note: We use the date and chrono libraries in order to make conversions between tm and time_t 
   representations since  mktime in c++ implicitly does a timezone conversion based on the user's machine.
*/

#ifndef CUSTOM_TIME_H
#define CUSTOM_TIME_H

#include <chrono>
#include <date.h>
#include <ctime>
#include <iostream>

class Time {
private:
	/* C++ tm structs and unix epoch times for both the local time and utc timepoint */
    tm m_datetime_local;
	time_t t_datetime_local;

	/* Special field for milliseconds since the tm struct's resolution is up to seconds */
    double m_milliseconds;

	/* Indication for a HH:MM:SS only timestamp */
	bool hh_mm_ss_only;

	/* Constructor for a string in HH:MM:SS timestamp */
	void HH_MM_SS_constructor(std::string local_time_point);
	
public:
    Time() {};

	/**
	 * @brief Create time with specific starting time
	 * @param local_time_point string in YYYY-MM-DD HH:MM:SS local 24 hour time format
	 * @param utc_adjustment Adjustment in hours from local time to utc e.g. If the time being represented is
     * 10:30am in Alice Springs which is 9.5 hours ahead of UTC, then this should be -9.5
	*/
	Time(std::string local_time_point);

	/** @brief Return true if the lhs local timestamp is ahead of the rhs local timestamp */
    bool operator>(const Time& other) const;

	/** @brief Return true if the rhs local timestamp is ahead of the lhs local timestamp */
	bool operator<(const Time& other) const;

	/** @brief Get current hour in 24 hour format */
    inline int get_local_hours() { return m_datetime_local.tm_hour; }

	/** @brief Get epoch timestamp representing the local time */
	inline time_t get_local_time_point() { return t_datetime_local; }

	/** @brief Get c++ tm struct of the current local time */
	inline tm get_local_tm() { return m_datetime_local; }

	/** @brief Advance the current time by a certain number of seconds */
    void update_time_seconds(double seconds);

	/** @brief Print human readable local time */
	void print_local_readable_time() {std::cout << get_local_readable_time() << std::endl;}
	
	/** @brief Get human readable local time as a string */
	std::string get_local_readable_time() const;
};

#endif /* CUSTOM_TIME_H */
