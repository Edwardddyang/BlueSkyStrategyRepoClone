#include <gtest/gtest.h>
#include <stdlib.h>
#include <string>

#include "model/V1Car.hpp"
#include "utils/Units.hpp"
#include "model/Car.hpp"
#include "config/Config.hpp"
#include "utils/Defines.hpp"

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  char* strat_root = std::getenv("STRAT_ROOT");
  RUNTIME_EXCEPTION(strat_root != nullptr, "No STRAT_ROOT environment variable detected."
                                           "Set it to the full path to gen12_strategy/RaceSim.");

  std::filesystem::path config_path("data/config/test_config.yaml");
  Config::initialize(config_path, strat_root);

  return RUN_ALL_TESTS();
}

TEST(v1CarTest, ElectricLossTest) {
  V1Car TestCar = V1Car();

  double energy_loss = TestCar.compute_electric_loss(300);
  double true_energy_loss = 0.0016667;
  EXPECT_NEAR(energy_loss, true_energy_loss, 0.0000001);

  energy_loss = TestCar.compute_electric_loss(0);
  true_energy_loss = 0;
  EXPECT_NEAR(energy_loss, true_energy_loss, 0.0000001);

  energy_loss = TestCar.compute_electric_loss(86400);
  true_energy_loss = 0.48;
  EXPECT_NEAR(energy_loss, true_energy_loss, 0.0000001);

  energy_loss = TestCar.compute_electric_loss(432000);
  true_energy_loss = 2.4;
  EXPECT_NEAR(energy_loss, true_energy_loss, 0.0000001);
}

TEST(v1CarTest, GravityLossTest) {
  V1Car TestCar = V1Car();

  // Notice: uses GRAVITY +_ACCELERATION as 9.81 as defined in Globals.h
  // If using more precise g value, difference will be larger than 0.0000001
  EnergyChange eng_change = TestCar.compute_gravitational_loss(10, 3600);
  double true_power = 8.31125;
  double true_energy = 0.00831125;
  EXPECT_NEAR(eng_change.energy, true_energy, 0.0000001);
  EXPECT_NEAR(eng_change.power, true_power, 0.00000001);

  eng_change = TestCar.compute_gravitational_loss(1400, 86400);
  true_power = 48.482291666;
  true_energy = 1.163575;
  EXPECT_NEAR(eng_change.energy, true_energy, 0.0000001);
  EXPECT_NEAR(eng_change.power, true_power, 0.00000001);

  eng_change = TestCar.compute_gravitational_loss(250, 86400);
  true_power = 8.65755208333;
  true_energy = 0.20778125;
  EXPECT_NEAR(eng_change.energy, true_energy, 0.0000001);
  EXPECT_NEAR(eng_change.power, true_power, 0.00000001);
}

TEST(v1CarTest, AeroLossTest) {
  V1Car TestCar = V1Car();

  Wind w = Wind(200, 10);
  EnergyChange eng_change = TestCar.compute_aero_loss(60, 180, w, 3600);
  double true_power = 27046;
  double true_energy = 27.046;
  EXPECT_NEAR(eng_change.energy, true_energy, 0.001);
  EXPECT_NEAR(eng_change.power, true_power, 0.5);

  w = Wind(310, 10);
  eng_change = TestCar.compute_aero_loss(80, 210, w, 86400);
  true_power = 45865;
  true_energy = 1100.76;
  EXPECT_NEAR(eng_change.energy, true_energy, 0.01);
  EXPECT_NEAR(eng_change.power, true_power, 0.5);

  w = Wind(300, 10);
  eng_change = TestCar.compute_aero_loss(60, 10, w, 400);
  true_power = 22588;
  true_energy = 2.5097881536111;
  EXPECT_NEAR(eng_change.energy, true_energy, 0.01);
  EXPECT_NEAR(eng_change.power, true_power, 0.5);
}

TEST(v1CarTest, RollingLossTest) {
  V1Car TestCar = V1Car();

  EnergyChange eng_change = TestCar.compute_rolling_loss(16.66667, 3600);
  double true_power = 230.388;
  double true_energy = 0.230388;
  EXPECT_NEAR(eng_change.energy, true_energy, 0.001);
  EXPECT_NEAR(eng_change.power, true_power, 0.5);

  eng_change = TestCar.compute_rolling_loss(30.5556, 86400);
  true_power = 562;
  true_energy = 13.5;
  EXPECT_NEAR(eng_change.energy, true_energy, 0.1);
  EXPECT_NEAR(eng_change.power, true_power, 2);

  eng_change = TestCar.compute_rolling_loss(2.7778, 1200);
  true_power = 24.039;
  true_energy = 0.0080130556;
  EXPECT_NEAR(eng_change.energy, true_energy, 0.01);
  EXPECT_NEAR(eng_change.power, true_power, 1);
}
