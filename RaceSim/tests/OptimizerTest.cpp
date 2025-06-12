#include <gtest/gtest.h>
#include <stdlib.h>
#include <filesystem>
#include <utility>
#include <memory>
#include <string>
#include <vector>
#include "model/V1Car.hpp"
#include "utils/Units.hpp"
#include "sim/WSCSimulator.hpp"
#include "opt/V1Optimizer.hpp"
#include "model/CarFactory.hpp"
#include "config/Config.hpp"
#include "utils/Defines.hpp"

class OptimizerTest : public ::testing::Test {
 public:
  static std::filesystem::path strat_root_path;
 protected:
  static void SetUpTestSuite() {
    char* strat_root = std::getenv("STRAT_ROOT");
    RUNTIME_EXCEPTION(strat_root != nullptr, "No STRAT_ROOT environment variable detected."
                                            "Set it to the full path to gen12_strategy/RaceSim.");
    Config::initialize("data/config/sim_test_config.yaml", strat_root);
    strat_root_path = std::filesystem::path(strat_root);
  }
};
std::filesystem::path OptimizerTest::strat_root_path;

TEST_F(OptimizerTest, Test1) {
  /* Create a model of the car */
  std::shared_ptr<Car> car = CarFactory::get_car(Config::get_instance()->get_model());

  /* Create route */
  std::shared_ptr<Route> route = std::make_shared<Route>(Config::get_instance()->get_base_route_path(),
                                                         false,
                                                         Config::get_instance()->get_init_control_stops());

  /* Create simulator */
  std::shared_ptr<Simulator> sim = std::make_shared<WSCSimulator>(car);

  /* Create optimizer */
  std::shared_ptr<V1Optimizer> opt = std::make_shared<V1Optimizer>(sim, route);
  RacePlan viable_race_plan = opt->optimize();

  EXPECT_TRUE(viable_race_plan.is_viable());

  std::vector<std::shared_ptr<ResultsLut>> result_luts = opt->get_result_luts();

  /* Parse the logs and ensure that all values match */
  std::filesystem::path results_file = strat_root_path / "data/luts/TestData/61.csv";
  ResultsLut golden_result(results_file);

  std::shared_ptr<ResultsLut> test_result = result_luts[61 - Config::get_instance()->get_min_speed()];

  const double margin = 0.00001;
  // Test battery_energy
  std::vector<double> golden_battery_energy = golden_result.get_battery_energy();
  std::vector<double> test_battery_energy = test_result->get_battery_energy();
  EXPECT_EQ(golden_battery_energy.size(), test_battery_energy.size());
  for (size_t i = 0; i < golden_battery_energy.size(); ++i) {
    EXPECT_NEAR(golden_battery_energy[i], test_battery_energy[i], margin);
  }

  // Test accumulated_distance
  std::vector<double> golden_accumulated_distance = golden_result.get_accumulated_distance();
  std::vector<double> test_accumulated_distance = test_result->get_accumulated_distance();
  EXPECT_EQ(golden_accumulated_distance.size(), test_accumulated_distance.size());
  for (size_t i = 0; i < golden_accumulated_distance.size(); ++i) {
    EXPECT_NEAR(golden_accumulated_distance[i], test_accumulated_distance[i], margin);
  }

  // Test time
  std::vector<std::string> golden_time = golden_result.get_time();
  std::vector<std::string> test_time = test_result->get_time();
  EXPECT_EQ(golden_time.size(), test_time.size());
  for (size_t i = 0; i < golden_time.size(); ++i) {
    EXPECT_EQ(golden_time[i], test_time[i]);
  }

  // Test azimuth
  std::vector<double> golden_azimuth = golden_result.get_azimuth();
  std::vector<double> test_azimuth = test_result->get_azimuth();
  EXPECT_EQ(golden_azimuth.size(), test_azimuth.size());
  for (size_t i = 0; i < golden_azimuth.size(); ++i) {
    EXPECT_NEAR(golden_azimuth[i], test_azimuth[i], margin);
  }

  // Test elevation
  std::vector<double> golden_elevation = golden_result.get_elevation();
  std::vector<double> test_elevation = test_result->get_elevation();
  EXPECT_EQ(golden_elevation.size(), test_elevation.size());
  for (size_t i = 0; i < golden_elevation.size(); ++i) {
    EXPECT_NEAR(golden_elevation[i], test_elevation[i], margin);
  }

  // Test bearing
  std::vector<double> golden_bearing = golden_result.get_bearing();
  std::vector<double> test_bearing = test_result->get_bearing();
  EXPECT_EQ(golden_bearing.size(), test_bearing.size());
  for (size_t i = 0; i < golden_bearing.size(); ++i) {
    EXPECT_NEAR(golden_bearing[i], test_bearing[i], margin);
  }

  // Test latitude
  std::vector<double> golden_latitude = golden_result.get_latitude();
  std::vector<double> test_latitude = test_result->get_latitude();
  EXPECT_EQ(golden_latitude.size(), test_latitude.size());
  for (size_t i = 0; i < golden_latitude.size(); ++i) {
    EXPECT_NEAR(golden_latitude[i], test_latitude[i], margin);
  }

  // Test longitude
  std::vector<double> golden_longitude = golden_result.get_longitude();
  std::vector<double> test_longitude = test_result->get_longitude();
  EXPECT_EQ(golden_longitude.size(), test_longitude.size());
  for (size_t i = 0; i < golden_longitude.size(); ++i) {
    EXPECT_NEAR(golden_longitude[i], test_longitude[i], margin);
  }

  // Test altitude
  std::vector<double> golden_altitude = golden_result.get_altitude();
  std::vector<double> test_altitude = test_result->get_altitude();
  EXPECT_EQ(golden_altitude.size(), test_altitude.size());
  for (size_t i = 0; i < golden_altitude.size(); ++i) {
    EXPECT_NEAR(golden_altitude[i], test_altitude[i], margin);
  }

  // Test speed
  std::vector<double> golden_speed = golden_result.get_speed();
  std::vector<double> test_speed = test_result->get_speed();
  EXPECT_EQ(golden_speed.size(), test_speed.size());
  for (size_t i = 0; i < golden_speed.size(); ++i) {
    EXPECT_NEAR(golden_speed[i], test_speed[i], margin);
  }

  // Test array_energy
  std::vector<double> golden_array_energy = golden_result.get_array_energy();
  std::vector<double> test_array_energy = test_result->get_array_energy();
  EXPECT_EQ(golden_array_energy.size(), test_array_energy.size());
  for (size_t i = 0; i < golden_array_energy.size(); ++i) {
    EXPECT_NEAR(golden_array_energy[i], test_array_energy[i], margin);
  }

  // Test array_power
  std::vector<double> golden_array_power = golden_result.get_array_power();
  std::vector<double> test_array_power = test_result->get_array_power();
  EXPECT_EQ(golden_array_power.size(), test_array_power.size());
  for (size_t i = 0; i < golden_array_power.size(); ++i) {
    EXPECT_NEAR(golden_array_power[i], test_array_power[i], margin);
  }

  // Test motor_power
  std::vector<double> golden_motor_power = golden_result.get_motor_power();
  std::vector<double> test_motor_power = test_result->get_motor_power();
  EXPECT_EQ(golden_motor_power.size(), test_motor_power.size());
  for (size_t i = 0; i < golden_motor_power.size(); ++i) {
    EXPECT_NEAR(golden_motor_power[i], test_motor_power[i], margin);
  }

  // Test motor_energy
  std::vector<double> golden_motor_energy = golden_result.get_motor_energy();
  std::vector<double> test_motor_energy = test_result->get_motor_energy();
  EXPECT_EQ(golden_motor_energy.size(), test_motor_energy.size());
  for (size_t i = 0; i < golden_motor_energy.size(); ++i) {
    EXPECT_NEAR(golden_motor_energy[i], test_motor_energy[i], margin);
  }

  // Test aero_power
  std::vector<double> golden_aero_power = golden_result.get_aero_power();
  std::vector<double> test_aero_power = test_result->get_aero_power();
  EXPECT_EQ(golden_aero_power.size(), test_aero_power.size());
  for (size_t i = 0; i < golden_aero_power.size(); ++i) {
    EXPECT_NEAR(golden_aero_power[i], test_aero_power[i], margin);
  }

  // Test aero_energy
  std::vector<double> golden_aero_energy = golden_result.get_aero_energy();
  std::vector<double> test_aero_energy = test_result->get_aero_energy();
  EXPECT_EQ(golden_aero_energy.size(), test_aero_energy.size());
  for (size_t i = 0; i < golden_aero_energy.size(); ++i) {
    EXPECT_NEAR(golden_aero_energy[i], test_aero_energy[i], margin);
  }

  // Test rolling_power
  std::vector<double> golden_rolling_power = golden_result.get_rolling_power();
  std::vector<double> test_rolling_power = test_result->get_rolling_power();
  EXPECT_EQ(golden_rolling_power.size(), test_rolling_power.size());
  for (size_t i = 0; i < golden_rolling_power.size(); ++i) {
    EXPECT_NEAR(golden_rolling_power[i], test_rolling_power[i], margin);
  }

  // Test rolling_energy
  std::vector<double> golden_rolling_energy = golden_result.get_rolling_energy();
  std::vector<double> test_rolling_energy = test_result->get_rolling_energy();
  EXPECT_EQ(golden_rolling_energy.size(), test_rolling_energy.size());
  for (size_t i = 0; i < golden_rolling_energy.size(); ++i) {
    EXPECT_NEAR(golden_rolling_energy[i], test_rolling_energy[i], margin);
  }

  // Test gravitational_power
  std::vector<double> golden_gravitational_power = golden_result.get_gravitational_power();
  std::vector<double> test_gravitational_power = test_result->get_gravitational_power();
  EXPECT_EQ(golden_gravitational_power.size(), test_gravitational_power.size());
  for (size_t i = 0; i < golden_gravitational_power.size(); ++i) {
    EXPECT_NEAR(golden_gravitational_power[i], test_gravitational_power[i], margin);
  }

  // Test gravitational_energy
  std::vector<double> golden_gravitational_energy = golden_result.get_gravitational_energy();
  std::vector<double> test_gravitational_energy = test_result->get_gravitational_energy();
  EXPECT_EQ(golden_gravitational_energy.size(), test_gravitational_energy.size());
  for (size_t i = 0; i < golden_gravitational_energy.size(); ++i) {
    EXPECT_NEAR(golden_gravitational_energy[i], test_gravitational_energy[i], margin);
  }

  // Test electric_energy
  std::vector<double> golden_electric_energy = golden_result.get_electric_energy();
  std::vector<double> test_electric_energy = test_result->get_electric_energy();
  EXPECT_EQ(golden_electric_energy.size(), test_electric_energy.size());
  for (size_t i = 0; i < golden_electric_energy.size(); ++i) {
    EXPECT_NEAR(golden_electric_energy[i], test_electric_energy[i], margin);
  }

  // Test delta_energy
  std::vector<double> golden_delta_energy = golden_result.get_delta_energy();
  std::vector<double> test_delta_energy = test_result->get_delta_energy();
  EXPECT_EQ(golden_delta_energy.size(), test_delta_energy.size());
  for (size_t i = 0; i < golden_delta_energy.size(); ++i) {
    EXPECT_NEAR(golden_delta_energy[i], test_delta_energy[i], margin);
  }
}
