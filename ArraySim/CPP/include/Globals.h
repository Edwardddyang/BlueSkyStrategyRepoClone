#include <stdexcept>
#include <sstream>
#include <string>

#pragma once

#define RUNTIME_ASSERT(cond, msg)                                        \
    do {                                                                \
        if (!(cond)) {                                                  \
            std::ostringstream oss;                                     \
            oss << "Assertion failed: (" #cond "), function " << __func__ \
                << ", file " << __FILE__ << ", line " << __LINE__ << ". " \
                << "Message: " << msg;                                  \
            throw std::runtime_error(oss.str());                        \
        }                                                               \
    } while (false) // Removed the backslash here
