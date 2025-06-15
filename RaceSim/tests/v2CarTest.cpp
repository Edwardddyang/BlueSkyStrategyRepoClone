#include <gtest/gtest.h>
#include <stdlib.h>
#include <string>

#include "model/V2Car.hpp"
#include "utils/Units.hpp"
#include "model/Car.hpp"
#include "config/Config.hpp"
#include "utils/Defines.hpp"

class v2CarTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    char* strat_root = std::getenv("STRAT_ROOT");
    RUNTIME_EXCEPTION(strat_root != nullptr, "No STRAT_ROOT environment variable detected."
                                              "Set it to the full path to gen12_strategy/RaceSim.");
    std::filesystem::path config_path("data/config/fsgp_sim_test_config.yaml");
    Config::initialize(config_path, strat_root);
  }
};

TEST_F(v2CarTest, AccelerationTest) {
  V2Car TestCar = V2Car();

  const Coord src_coord(36.9992602, -86.3723823);
  const Coord dest_coord(36.9991661, -86.3726461);
  const double init_speed = 4.0;
  const double acceleration = 2.5;
  Time init_time("2024-08-16 10:00:00", +6.0);
  const Wind wind(5.0, 5.5);
  const Irradiance irr(800, 0.0);
  const double distance = 100.0;

  const CarUpdate result = TestCar.compute_travel_update(src_coord, dest_coord, init_speed,
                                                        acceleration, &init_time, wind, irr, distance);

  EXPECT_NEAR(result.bearing, 245.93242811073034, 0.0001);
  EXPECT_NEAR(result.az_el.Az, 240.723463989025, 0.0001);
  EXPECT_NEAR(result.delta_time, 7.486253353, 0.0001);
  EXPECT_NEAR(result.delta_distance, 100.0, 0.0001);
  EXPECT_NEAR(result.aero.energy, 0.000491823203, 0.0001);
  EXPECT_NEAR(result.rolling.energy, 0.000259202086, 0.0001);
  EXPECT_NEAR(result.motor_energy, 0.0243149, 0.0001);
}
