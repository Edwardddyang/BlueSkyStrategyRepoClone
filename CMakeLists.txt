# This CMake file creates two targets: 
# 1. The RaceSim dynamic library .dll and associated unit tests
# 2. The opt.exe executable

cmake_minimum_required(VERSION 3.12)
cmake_policy(SET CMP0091 NEW)
project(RaceSim)

find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(SimUtils REQUIRED)

# libIntegrate
set(BUILD_TESTS OFF CACHE BOOL "Disable building tests" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/libIntegrate)

# ---------------- RaceSim dynamic library ----------------
# All source files
set(MODEL_SOURCES
    src/model/Car.cpp
)
set(CONFIG_SOURCES
    src/config/Config.cpp
)
set(OPTIMIZER_SOURCES
    src/optimizer/WSCOptimizer.cpp
    # src/optimizer/FSGPOptimizer.cpp
    # src/optimizer/PopulationGenerator.cpp
    # src/optimizer/Mutation.cpp
)
set(ROUTE_SOURCES
    src/route/Route.cpp
    src/route/RacePlan.cpp
)
set(SIMULATION_SOURCES
    src/simulation/WSCSimulator.cpp
    src/simulation/ASCSimulator.cpp
    src/simulation/FSGPSimulator.cpp
    src/simulation/TelemetrySimulator.cpp
)
set(RACESIM_SOURCES
    ${CONFIG_SOURCES}
    ${MODEL_SOURCES}
    ${OPTIMIZER_SOURCES}
    ${ROUTE_SOURCES}
    ${SIMULATION_SOURCES}
)
add_library(RaceSim SHARED ${RACESIM_SOURCES})
target_compile_features(RaceSim PRIVATE cxx_std_20)
target_include_directories(RaceSim
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(RaceSim PUBLIC  yaml-cpp::yaml-cpp
                                      spdlog::spdlog
                                      nlohmann_json::nlohmann_json
                                      SimUtils::SimUtils
                                      libIntegrate)

set_target_properties(RaceSim PROPERTIES
  CXX_STANDARD 20
  POSITION_INDEPENDENT_CODE ON)
# ---------------------------------------------------------

# ------------------- opt.exe executable ------------------
# add_executable(opt ${CMAKE_SOURCE_DIR}/src/main.cpp)
# target_link_libraries(opt PRIVATE RaceSim) 
# target_compile_features(opt PRIVATE cxx_std_20)

# ---------------------------------------------------------
