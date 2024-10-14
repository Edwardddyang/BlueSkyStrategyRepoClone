# This CMake file creates two targets: 
# 1. The RaceSim dynamic library .dll and associated unit tests
# 2. The opt.exe executable

cmake_minimum_required(VERSION 3.12)
project(RaceSim VERSION 1.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

set(INSTALL_DIR "${CMAKE_SOURCE_DIR}/install")

# -------------- Link in 3rd party libraries --------------
set(EXTERNAL_DIR ${CMAKE_SOURCE_DIR}/libs)

# spdlog
add_subdirectory(${EXTERNAL_DIR}/spdlog)
list(APPEND EXTERNAL_DEPS spdlog::spdlog)

# GoogleTest
set(INSTALL_GTEST OFF)
add_subdirectory(${EXTERNAL_DIR}/googletest)
list(APPEND EXTERNAL_DEPS gtest gtest_main)

# yaml-cpp
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "Enable testing in yaml-cpp" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "Build yaml-cpp tools" FORCE)
set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "Build yaml-cpp contrib" FORCE)
add_subdirectory(${EXTERNAL_DIR}/yaml-cpp)
list(APPEND EXTERNAL_DEPS yaml-cpp)
# ---------------------------------------------------------

# ---------------- RaceSim dynamic library ----------------
# All source files
set(MODEL_SOURCES
    src/model/Car.cpp
    src/model/CarFactory.cpp
    src/model/V1Car.cpp
)
set(CONFIG_SOURCES
    src/config/Config.cpp
)
set(OPTIMIZER_SOURCES
    src/optimizer/Optimizer.cpp
    src/optimizer/OptimizerFactory.cpp
    src/optimizer/V1Optimizer.cpp
)
set(ROUTE_SOURCES
    src/route/Route.cpp
)
set(SIMULATION_SOURCES
    src/simulation/Simulator.cpp
)
set(UTILS_SOURCES
    src/utils/CustomTime.cpp
    src/utils/Geography.cpp
    src/utils/Luts.cpp
    src/utils/Utilities.cpp
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

# Don't need include directories for yaml-cpp, googletest,spdlog - they set up those with INTERFACE
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

# Simulator
add_executable(SimTest tests/SimTest.cpp)
target_link_libraries(SimTest gtest gtest_main RaceSim ${EXTERNAL_DEPS})
add_test(SimTest SimTest)

# Run tests
add_custom_target(run_tests ALL
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure || true)

# Add test for full scale simulation and match 58.csv
# Add valgrind test
# Add linter test
# ---------------------------------------------------------

# ---------------------- Installation ---------------------
install(TARGETS RaceSim
        RUNTIME DESTINATION ${CMAKE_SOURCE_DIR}/install/lib
        LIBRARY DESTINATION ${CMAKE_SOURCE_DIR}/install/lib)
install(TARGETS opt
        RUNTIME DESTINATION ${CMAKE_SOURCE_DIR}/install/bin)
install(TARGETS GeographyTest TimeTest LutsTest v1CarTest RouteTest SimTest
        RUNTIME DESTINATION ${CMAKE_SOURCE_DIR}/install/tests/bin)
# ---------------------------------------------------------

# # -------------------- Clean install --------------------
add_custom_target(clean_install
    COMMAND ${CMAKE_COMMAND} -E rm -rf ${INSTALL_DIR}
    COMMENT "Cleaning the install directory"
)
# # --------------------------------------------------------
