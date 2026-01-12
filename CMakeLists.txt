# This CMake file creates the following targets: 
# 1. The Strategy dynamic library .dll and associated unit tests
# 2. A sample executable wsc.exe that optimizes for WSC
# 3. A sample executable fsgp.exe that optimizes for FSGP
# 4. A sample executable asc.exe that optimizes for ASC

cmake_minimum_required(VERSION 3.12)
cmake_policy(SET CMP0091 NEW)
project(Strategy)

find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(SimUtils REQUIRED)

# libIntegrate
set(BUILD_TESTS OFF CACHE BOOL "Disable building tests" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/libIntegrate)

# ---------------- Strategy dynamic library ----------------
# All source files
set(MODEL_SOURCES
    src/model/Car.cpp
)
set(CONFIG_SOURCES
    src/config/ConfigParser.cpp
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
set(STRATEGY_SOURCES
    ${CONFIG_SOURCES}
    ${MODEL_SOURCES}
    ${OPTIMIZER_SOURCES}
    ${ROUTE_SOURCES}
    ${SIMULATION_SOURCES}
)
add_library(Strategy SHARED ${STRATEGY_SOURCES})
target_compile_features(Strategy PRIVATE cxx_std_20)
target_include_directories(Strategy
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/libIntegrate/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(Strategy PUBLIC  yaml-cpp::yaml-cpp
                                      spdlog::spdlog
                                      nlohmann_json::nlohmann_json
                                      SimUtils::SimUtils
                              PRIVATE libIntegrate)

set_target_properties(Strategy PROPERTIES
  CXX_STANDARD 20
  POSITION_INDEPENDENT_CODE ON)
set_target_properties(Strategy PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)  # I'm lazy lol

# Unit Tests
option(RUN_TEST "Compile and execute unit tests" OFF)
if (RUN_TEST AND PROJECT_IS_TOP_LEVEL)
  enable_testing()
  find_package(GTest REQUIRED)
  include(GoogleTest)
  # For Windows as recommended by https://google.github.io/googletest/quickstart-cmake.html
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

  add_executable(RouteTest ${CMAKE_CURRENT_SOURCE_DIR}/tests/RouteTest.cpp)
  target_link_libraries(RouteTest PRIVATE Strategy GTest::gtest_main)
  gtest_discover_tests(RouteTest)
  target_compile_definitions(RouteTest PRIVATE LUTS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/data/luts/")
endif()

# Clang-format
find_program(CLANG_FORMAT clang-format)
file(GLOB_RECURSE FORMAT_FILES CONFIGURE_DEPENDS
  "${CMAKE_SOURCE_DIR}/src/*.[cht]pp"
  "${CMAKE_SOURCE_DIR}/include/*.[ch]pp"
  "${CMAKE_SOURCE_DIR}/tests/*.[ch]pp"
)
list(REMOVE_ITEM FORMAT_FILES
  "${CMAKE_SOURCE_DIR}/include/opt/PopulationGenerator.hpp"
  "${CMAKE_SOURCE_DIR}/src/optimizer/Mutation.cpp"
  "${CMAKE_SOURCE_DIR}/src/optimizer/PopulationGenerator.cpp"
  "${CMAKE_SOURCE_DIR}/src/optimizer/FSGPOptimizer.cpp"
  "${CMAKE_SOURCE_DIR}/include/opt/FSGPOptimizer.hpp"
  "${CMAKE_SOURCE_DIR}/src/FSGP.cpp"
  "${CMAKE_SOURCE_DIR}/src/ASC.cpp"
  "${CMAKE_SOURCE_DIR}/src/simulation/ASCSimulator.cpp"
)
add_custom_target(format
  COMMAND ${CLANG_FORMAT} -i ${FORMAT_FILES}
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

# Clang-format for github actions
find_program(CLANG_FORMAT_20 clang-format-20)
add_custom_target(format-gh
  COMMAND ${CLANG_FORMAT_20} --dry-run --Werror ${FORMAT_FILES}
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

# Clang-tidy for github actions
find_program(CLANG_TIDY_20 clang-tidy-20)
add_custom_target(tidy-gh
  COMMAND ${CLANG_TIDY_20}
          -p ${CMAKE_BINARY_DIR}
          --system-headers=false
          ${FORMAT_FILES}
)

# Clang-target
find_program(CLANG_TIDY clang-tidy)
add_custom_target(tidy
  COMMAND ${CLANG_TIDY}
          -p ${CMAKE_BINARY_DIR}
          --system-headers=false
          ${FORMAT_FILES}
)

# Strategy library dll installation
install(TARGETS Strategy
        EXPORT StrategyLib
        RUNTIME DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include)
install(EXPORT StrategyLib FILE StrategyTargets.cmake NAMESPACE Strategy:: DESTINATION lib/cmake/Strategy)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/ DESTINATION include/Strategy)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/StrategyConfigVersion.cmake"
  VERSION 1.0
  COMPATIBILITY SameMajorVersion
)
configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/StrategyConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/StrategyConfig.cmake"
  INSTALL_DESTINATION lib/cmake/Strategy
)
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/StrategyConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/StrategyConfigVersion.cmake"
  DESTINATION lib/cmake/Strategy
)

# ------------------- WSC ---------------------------
add_executable(wsc ${CMAKE_SOURCE_DIR}/src/WSC.cpp)
target_link_libraries(wsc PRIVATE Strategy) 
target_compile_features(wsc PRIVATE cxx_std_20)

# ------------------- FSGP ---------------------------
# add_executable(fsgp ${CMAKE_SOURCE_DIR}/src/FSGP.cpp)
# target_link_libraries(fsgp PRIVATE Strategy) 
# target_compile_features(fsgp PRIVATE cxx_std_20)

# ------------------- ASC ---------------------------
# add_executable(fsgp ${CMAKE_SOURCE_DIR}/src/FSGP.cpp)
# target_link_libraries(fsgp PRIVATE Strategy) 
# target_compile_features(fsgp PRIVATE cxx_std_20)

