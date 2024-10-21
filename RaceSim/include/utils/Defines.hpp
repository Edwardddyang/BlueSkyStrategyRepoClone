#include <stdexcept>
#include <sstream>
#include <string>

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
