#include <string>
#include <cstdint>
#include <cmath>
#include <ctime>
#include <chrono>
#include <date.h>
#include "time.hpp"
#include "Globals.h"
#include <iomanip>

Time::Time(std::string local_time_point) {
	std::istringstream iss(local_time_point);
	std::string date_str, time_str;
	iss >> date_str >> time_str;

	RUNTIME_ASSERT(!iss.fail(), "Error while parsing time string: " + local_time_point);
	if (time_str.empty()) {
		/* The timestamp is in HH:MM:SS format, call the other constructor */
		HH_MM_SS_constructor(local_time_point);
		return;
	} 

	std::istringstream rss(local_time_point);
    date::sys_time<std::chrono::seconds> epoch_time;
    rss >> date::parse("%F %T", epoch_time);
	RUNTIME_ASSERT(!rss.fail(), "Error while parsing time string: " + local_time_point);

    // Convert sys_time to time_t
    t_datetime_local = std::chrono::system_clock::to_time_t(epoch_time);
	m_datetime_local = *gmtime(&t_datetime_local);
	m_milliseconds = 0;

	hh_mm_ss_only = false;
	return;
}

void Time::HH_MM_SS_constructor(std::string local_time_point) {
	std::istringstream iss(local_time_point);
	int hours, minutes, seconds;

	char delimiter;
	iss >> hours >> delimiter >> minutes >> delimiter >> seconds;
	RUNTIME_ASSERT(!iss.fail(), "Error while parsing HH:MM:SS only time string: " + local_time_point);

    time_t now = time(nullptr);
    m_datetime_local = *localtime(&now);
	m_datetime_local.tm_hour = hours;
	m_datetime_local.tm_min = minutes;
	m_datetime_local.tm_sec = seconds;

	hh_mm_ss_only = true;
	return;
}

/* Compare each member of the tm struct */
bool Time::operator>(const Time& other) const {

	if (!this->hh_mm_ss_only && !other.hh_mm_ss_only) {
		if (this->m_datetime_local.tm_year > other.m_datetime_local.tm_year) {
			return true;
		} else if (this->m_datetime_local.tm_year < other.m_datetime_local.tm_year) {
			return false;
		}

		if (this->m_datetime_local.tm_mon > other.m_datetime_local.tm_mon) {
			return true;
		} else if (this->m_datetime_local.tm_mon < other.m_datetime_local.tm_mon) {
			return false;
		}

		if (this->m_datetime_local.tm_mday > other.m_datetime_local.tm_mday) {
			return true;
		} else if (this->m_datetime_local.tm_mday < other.m_datetime_local.tm_mday) {
			return false;
		}
	}

	if (this->m_datetime_local.tm_hour > other.m_datetime_local.tm_hour) {
		return true;
	} else if (this->m_datetime_local.tm_hour < other.m_datetime_local.tm_hour) {
		return false;
	}

	if (this->m_datetime_local.tm_min > other.m_datetime_local.tm_min) {
		return true;
	} else if (this->m_datetime_local.tm_min < other.m_datetime_local.tm_min) {
		return false;
	}

	if (this->m_datetime_local.tm_sec > other.m_datetime_local.tm_sec) {
		return true;
	} else if (this->m_datetime_local.tm_sec < other.m_datetime_local.tm_sec) {
		return false;
	}

	if (this->m_milliseconds > other.m_milliseconds) {
		return true;
	} else if (this->m_milliseconds < other.m_milliseconds) {
		return false;
	}

	return false;
	
}

bool Time::operator<(const Time& other) const {

	if (!this->hh_mm_ss_only && !other.hh_mm_ss_only) {
		if (this->m_datetime_local.tm_year < other.m_datetime_local.tm_year) {
			return true;
		} else if (this->m_datetime_local.tm_year > other.m_datetime_local.tm_year) {
			return false;
		}

		if (this->m_datetime_local.tm_mon < other.m_datetime_local.tm_mon) {
			return true;
		} else if (this->m_datetime_local.tm_mon > other.m_datetime_local.tm_mon) {
			return false;
		}

		if (this->m_datetime_local.tm_mday < other.m_datetime_local.tm_mday) {
			return true;
		} else if (this->m_datetime_local.tm_mday > other.m_datetime_local.tm_mday) {
			return false;
		}
	}

	if (this->m_datetime_local.tm_hour < other.m_datetime_local.tm_hour) {
		return true;
	} else if (this->m_datetime_local.tm_hour > other.m_datetime_local.tm_hour) {
		return false;
	}

	if (this->m_datetime_local.tm_min < other.m_datetime_local.tm_min) {
		return true;
	} else if (this->m_datetime_local.tm_min > other.m_datetime_local.tm_min) {
		return false;
	}

	if (this->m_datetime_local.tm_sec < other.m_datetime_local.tm_sec) {
		return true;
	} else if (this->m_datetime_local.tm_sec > other.m_datetime_local.tm_sec) {
		return false;
	}

	if (this->m_milliseconds < other.m_milliseconds) {
		return true;
	} else if (this->m_milliseconds > other.m_milliseconds) {
		return false;
	}

	return false;
}

std::string Time::get_local_readable_time() const {
	if (hh_mm_ss_only) {
		std::string time_s = std::to_string(m_datetime_local.tm_hour) + ":" 
							 + std::to_string(m_datetime_local.tm_min) + ":" 
							 + std::to_string(m_datetime_local.tm_sec);
		return time_s;
	}

	std::string time = asctime(&m_datetime_local); 
	return time.erase(time.size()-1);
};

std::string Time::get_local_constructor_time() const {
	if (hh_mm_ss_only) {
		std::string time_s = std::to_string(m_datetime_local.tm_hour) + ":" 
							 + std::to_string(m_datetime_local.tm_min) + ":" 
							 + std::to_string(m_datetime_local.tm_sec);
		return time_s;
	}

	std::ostringstream oss;
	oss << std::put_time(&m_datetime_local, "%Y-%m-%d %H:%M:%S");
	RUNTIME_ASSERT(!oss.fail(), "Local time constructor has failed");
	return oss.str();
}

void Time::update_time_seconds(double seconds) {
	double total_seconds = seconds + 1000 * m_milliseconds;
	t_datetime_local += (int)seconds;

	m_datetime_local = *gmtime(&t_datetime_local);
	m_milliseconds = total_seconds - floor(total_seconds);

}

