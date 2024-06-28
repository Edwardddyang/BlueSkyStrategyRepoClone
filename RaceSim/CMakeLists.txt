# This CMake file creates two targets: 
# 1. The RaceSim dynamic library .dll and associated unit tests
# 2. The opt.exe executable

cmake_minimum_required(VERSION 3.12)
project(RaceSim VERSION 1.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -------------- Link in 3rd party libraries --------------
Include(FetchContent)

# spdlog
FetchContent_Declare(
    spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG v1.13.0
    DOWNLOAD_EXTRACT_TIMESTAMP true
)
FetchContent_MakeAvailable(spdlog)

# GoogleTest
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
  DOWNLOAD_EXTRACT_TIMESTAMP true
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "Enable installation of googletest. (Projects embedding googletest may want to turn this OFF.)" FORCE)
FetchContent_MakeAvailable(googletest)

# Yaml-cpp
FetchContent_Declare(
    yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG 0.8.0
    DOWNLOAD_EXTRACT_TIMESTAMP true
)
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "Enable testing in yaml-cpp")
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "Build yaml-cpp tools")
set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "Build yaml-cpp contrib")
FetchContent_MakeAvailable(yaml-cpp)
set(EXTERNAL_DEPS spdlog::spdlog yaml-cpp)
# ---------------------------------------------------------

# ---------------- RaceSim dynamic library ----------------
# All source files
set(MODEL_SOURCES
    src/model/base_car.cpp
    src/model/car_factory.cpp
    src/model/v1_car.cpp
)
set(CONFIG_SOURCES
    src/config/config.cpp
)
set(OPTIMIZER_SOURCES
    src/optimizer/base_opt.cpp
    src/optimizer/opt_factory.cpp
    src/optimizer/v1_opt.cpp
)
set(ROUTE_SOURCES
    src/route/route.cpp
)
set(SIMULATION_SOURCES
    src/simulation/sim.cpp
)
set(UTILS_SOURCES
    src/utils/custom_time.cpp
    src/utils/geography.cpp
    src/utils/Luts.cpp
    src/utils/utilities.cpp
)
set(RACESIM_SOURCES
    ${CONFIG_SOURCES}
    ${MODEL_SOURCES}
    ${OPTIMIZER_SOURCES}
    ${ROUTE_SOURCES}
    ${SIMULATION_SOURCES}
    ${UTILS_SOURCES}
)
add_library(RaceSim SHARED ${RACESIM_SOURCES})
target_include_directories(RaceSim PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/config
    ${CMAKE_CURRENT_SOURCE_DIR}/include/model
    ${CMAKE_CURRENT_SOURCE_DIR}/include/opt
    ${CMAKE_CURRENT_SOURCE_DIR}/include/route
    ${CMAKE_CURRENT_SOURCE_DIR}/include/sim
    ${CMAKE_CURRENT_SOURCE_DIR}/include/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/include/date # Header only third party library
)

target_compile_features(RaceSim PRIVATE cxx_std_17)

# Filesystem requires C++17 standard which might not be available on all compilers
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++17" COMPILER_SUPPORTS_CXX17)
if (COMPILER_SUPPORTS_CXX17)
    message(STATUS "Compiler supports C++17")
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
else()
    message(FATAL_ERROR "Compiler does not support C++17")
endif()

# Manually link against filesystem library for older compilers
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0)
        message(STATUS "Using GCC version ${CMAKE_CXX_COMPILER_VERSION}. Manually linking against stdc++fs.")
        target_link_libraries(RaceSim PRIVATE stdc++fs)
    endif()
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 7.0)
        message(STATUS "Using Clang version ${CMAKE_CXX_COMPILER_VERSION}. Manually linking against stdc++fs")
        target_link_libraries(RaceSim PRIVATE c++fs)
    endif()
endif()
target_link_libraries(RaceSim PRIVATE ${EXTERNAL_DEPS})
# ---------------------------------------------------------

# ------------------- opt.exe executable ------------------
add_executable(opt ./src/main.cpp)
target_link_libraries(opt PRIVATE ${EXTERNAL_DEPS} RaceSim) 
target_compile_features(opt PRIVATE cxx_std_17)
# ---------------------------------------------------------

# ----------------------- Unit Tests ----------------------
enable_testing()

# Geography functions
add_executable(GeographyTest tests/GeographyTest.cpp)
target_link_libraries(GeographyTest gtest gtest_main RaceSim ${EXTERNAL_DEPS})
add_test(GeographyTest GeographyTest)

# Custom time class
add_executable(TimeTest tests/TimeTest.cpp)
target_link_libraries(TimeTest gtest gtest_main RaceSim ${EXTERNAL_DEPS})
add_test(TimeTest TimeTest)

# Lookup Tables
add_executable(LutsTest tests/LutsTest.cpp)
target_link_libraries(LutsTest gtest gtest_main RaceSim ${EXTERNAL_DEPS})
add_test(LutsTest LutsTest)

# Gen 11.5 Car Model
add_executable(v1CarTest tests/v1CarTest.cpp)
target_link_libraries(v1CarTest gtest gtest_main RaceSim ${EXTERNAL_DEPS})
add_test(v1CarTest v1CarTest)

# Route
add_executable(RouteTest tests/RouteTest.cpp)
target_link_libraries(RouteTest gtest gtest_main RaceSim ${EXTERNAL_DEPS})
add_test(RouteTest RouteTest)
# ---------------------------------------------------------

# ---------------------- Installation ---------------------
install(TARGETS RaceSim
        RUNTIME DESTINATION ${CMAKE_SOURCE_DIR}/install/lib)
install(TARGETS opt
        RUNTIME DESTINATION ${CMAKE_SOURCE_DIR}/install/bin)
install(TARGETS GeographyTest TimeTest LutsTest v1CarTest RouteTest
        RUNTIME DESTINATION ${CMAKE_SOURCE_DIR}/install/tests/bin)
# ---------------------------------------------------------

# ------------------------- Clean -------------------------
# ---------------------------------------------------------
