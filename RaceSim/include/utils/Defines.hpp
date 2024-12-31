#include <stdexcept>
#include <sstream>
#include <string>
#include <ctime>

#include "spdlog/spdlog.h"

#pragma once

#define RUNTIME_EXCEPTION(cond, msg, ...)                           \
    do {                                                            \
        if (!(cond)) {                                              \
            std::ostringstream oss;                                 \
            oss << "(ERROR Encountered) => " << __func__            \
                << ", file " << __FILE__ << ", line " << __LINE__   \
                << ". Message: " << msg;                            \
            spdlog::critical(oss.str(), ##__VA_ARGS__);             \
            exit(1);                                                \
        }                                                           \
    } while (false)

#ifdef _WIN32
  #define GMTIME_SAFE(time_t_ptr, tm_ptr) gmtime_s(tm_ptr, time_t_ptr)
  #define LOCALTIME_SAFE(time_t_ptr, tm_ptr) localtime_s(tm_ptr, time_t_ptr)
  #define ASCTIME_SAFE(char_ptr, tm_ptr) asctime_s(char_ptr, 26, tm_ptr)
  #define TIMEGM(tm_ptr) _mkgmtime(tm_ptr)
#else
  #define GMTIME_SAFE(time_t_ptr, tm_ptr) gmtime_r(time_t_ptr, tm_ptr)
  #define LOCALTIME_SAFE(time_t_ptr, tm_ptr) localtime_r(time_t_ptr, tm_ptr)
  #define ASCTIME_SAFE(tm_ptr, char_ptr) asctime_r(tm_ptr, char_ptr)
  #define TIMEGM(tm_ptr) timegm(tm_ptr)
#endif
