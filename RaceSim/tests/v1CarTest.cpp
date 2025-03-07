#include <gtest/gtest.h>
#include <stdlib.h>
#include <string>

#include "model/V1Car.hpp"
#include "utils/Units.hpp"
#include "model/Car.hpp"
#include "config/Config.hpp"
#include "utils/Defines.hpp"

class v1CarTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    char* strat_root = std::getenv("STRAT_ROOT");
    RUNTIME_EXCEPTION(strat_root != nullptr, "No STRAT_ROOT environment variable detected."
                                              "Set it to the full path to gen12_strategy/RaceSim.");
    std::filesystem::path config_path("data/config/sim_test_config.yaml");
    Config::initialize(config_path, strat_root);
  }
};

TEST_F(v1CarTest, ElectricLossTest) {
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

TEST_F(v1CarTest, GravityLossTest) {
  V1Car TestCar = V1Car();

  // Notice: uses GRAVITY +_ACCELERATION as 9.81 as defined in Globals.h
  // If using more precise g value, difference will be larger than 0.0000001
  EnergyChange eng_change = TestCar.compute_gravitational_loss(10, 3600, 0.0);
  double true_force = 0.0;
  double true_power = 0.0;
  double true_energy = 0.0;
  EXPECT_EQ(eng_change.power, true_power);
  EXPECT_EQ(eng_change.energy, true_energy);
  EXPECT_EQ(eng_change.force, true_force);

  eng_change = TestCar.compute_gravitational_loss(1400, 86400, 0.104528463268);
  true_force = 312.75438852;
  true_power = 5.06777944361;
  true_energy = 0.121626706647;
  EXPECT_NEAR(eng_change.energy, true_energy, 0.0000001);
  EXPECT_NEAR(eng_change.power, true_power, 0.00000001);
  EXPECT_NEAR(eng_change.force, true_force, 0.0000001);

  eng_change = TestCar.compute_gravitational_loss(250.0, 20.0, 0.173648177667);
  true_force = 519.564029988;
  true_power = 6494.55037485;
  true_energy = 0.0360808354158;
  EXPECT_NEAR(eng_change.energy, true_energy, 0.0000001);
  EXPECT_NEAR(eng_change.power, true_power, 0.00000001);
  EXPECT_NEAR(eng_change.force, true_force, 0.0000001);
}

TEST_F(v1CarTest, AeroLossTest) {
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

TEST_F(v1CarTest, RollingLossTest) {
  V1Car TestCar = V1Car();

  EnergyChange eng_change = TestCar.compute_rolling_loss(16.66667, 3600);
  double true_power = 161.3649843;
  double true_energy = 0.1613649843;
  EXPECT_NEAR(eng_change.energy, true_energy, 0.0001);
  EXPECT_NEAR(eng_change.power, true_power, 0.0001);

  eng_change = TestCar.compute_rolling_loss(30.5556, 86400);
  true_power = 325.8119939;
  true_energy = 7.819487855;
  EXPECT_NEAR(eng_change.energy, true_energy, 0.0001);
  EXPECT_NEAR(eng_change.power, true_power, 0.0001);

  eng_change = TestCar.compute_rolling_loss(2.7778, 1200);
  true_power = 22.14316298;
  true_energy = 0.007381054328;
  EXPECT_NEAR(eng_change.energy, true_energy, 0.0001);
  EXPECT_NEAR(eng_change.power, true_power, 0.0001);
}
