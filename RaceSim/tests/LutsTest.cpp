#include <stdlib.h>
#include <gtest/gtest.h>
#include <string>
#include <filesystem>
#include "utils/Luts.hpp"
#include "utils/Units.hpp"

class LutsTest : public ::testing::Test {
 public:
  static std::filesystem::path strat_root_path;
 protected:
  static void SetUpTestSuite() {
    char* strat_root = std::getenv("STRAT_ROOT");
    RUNTIME_EXCEPTION(strat_root != nullptr, "No STRAT_ROOT environment variable detected."
                                            "Set it to the full path to gen12_strategy/RaceSim.");
    strat_root_path = std::filesystem::path(strat_root);
  }
};
std::filesystem::path LutsTest::strat_root_path;

TEST_F(LutsTest, BasicLutTest) {
  std::filesystem::path power_factor_path = LutsTest::strat_root_path / "data/luts/TestData/power_factor.csv";
  BasicLut TestBaseLut = BasicLut(power_factor_path);

  double value = TestBaseLut.get_value(1, 1);
  double true_val = 0.033922205865;
  EXPECT_NEAR(value, true_val, 0.0001);

  value = TestBaseLut.get_value(73, 147);
  true_val = 0.860374110125;
  EXPECT_NEAR(value, true_val, 0.0001);

  value = TestBaseLut.get_value(38, 207);
  true_val = 0.3957201321;
  EXPECT_NEAR(value, true_val, 0.0001);
}

TEST_F(LutsTest, EffLutTest) {
  std::filesystem::path roll_res_yint_path = LutsTest::strat_root_path / "data/luts/TestData/rr1.csv";
  EffLut TestEffLut = EffLut(roll_res_yint_path);

  double value = TestEffLut.get_value(5, 0);
  double true_val = 0.002574815;
  EXPECT_NEAR(value, true_val, 0.0001);

  value = TestEffLut.get_value(5.5, 60);
  true_val = 0.002703204;
  EXPECT_NEAR(value, true_val, 0.0001);

  value = TestEffLut.get_value(6, 120);
  true_val = 0.00252506;
  EXPECT_NEAR(value, true_val, 0.0001);

  std::filesystem::path roll_res_slope_path = LutsTest::strat_root_path / "data/luts/TestData/rr2.csv";
  TestEffLut = EffLut(roll_res_slope_path);

  value = TestEffLut.get_value(5, 0);
  true_val = 0.0000318442;
  EXPECT_NEAR(value, true_val, 0.0001);

  value = TestEffLut.get_value(5.5, 60);
  true_val = 0.0000319602;
  EXPECT_NEAR(value, true_val, 0.0001);

  value = TestEffLut.get_value(6, 120);
  true_val = 0.0000330283;
  EXPECT_NEAR(value, true_val, 0.0001);
}

TEST_F(LutsTest, ForecastLutTest) {
  std::filesystem::path dni_path = LutsTest::strat_root_path / "data/luts/TestData/dni.csv";
  ForecastLut TestForecastLut = ForecastLut(dni_path);
  ForecastCoord testCoord = ForecastCoord(-12.46322, 130.84618);
  EXPECT_NEAR(testCoord.lat, -12.46322, 0.0001);

  EXPECT_NEAR(TestForecastLut.get_value(testCoord, 1697923800), 10, 0.00001);
  testCoord = ForecastCoord(-13.70958, 131.6979);

  time_t testTime = 1697923800;
  // 231021213000 actual value from dni table
  double value = TestForecastLut.get_value(testCoord, testTime);
  double true_val = 23;
  EXPECT_NEAR(value, true_val, 0.0001);

  // 231023010000 - actual value from dni table
  testTime = 1698022800;
  value = TestForecastLut.get_value(testCoord, testTime);
  true_val = 722;
  EXPECT_NEAR(value, true_val, 0.0001);

  // 231026083000 - actual value from dni table
  testTime = 1698309000;
  value = TestForecastLut.get_value(testCoord, testTime);
  true_val = 53;
  EXPECT_NEAR(value, true_val, 0.0001);

  testCoord = ForecastCoord(-25.7593, 133.28946);

  // 231022020000 - actual value from dni table
  testTime = 1697940000;
  value = TestForecastLut.get_value(testCoord, testTime);
  true_val = 1003;
  EXPECT_NEAR(value, true_val, 0.0001);

  // 231022233000 - actual value from dni table
  testTime = 1698017400;
  value = TestForecastLut.get_value(testCoord, testTime);
  true_val = 874;
  EXPECT_NEAR(value, true_val, 0.0001);

  // 231023173000 - actual value from dni table
  testTime = 1698082200;
  value = TestForecastLut.get_value(testCoord, testTime);
  true_val = 0;
  EXPECT_NEAR(value, true_val, 0.0001);

  std::filesystem::path dhi_path = LutsTest::strat_root_path / "data/luts/TestData/dhi.csv";
  TestForecastLut = ForecastLut(dhi_path);
  testCoord = ForecastCoord(-14.211745, 132.03927);

  // 231021223000 - actual value from dhi table
  testTime = 1697927400;
  value = TestForecastLut.get_value(testCoord, testTime);
  true_val = 132;
  EXPECT_NEAR(value, true_val, 0.0001);

  // 231022210000 - actual value from dhi table
  testTime = 1698008400;
  value = TestForecastLut.get_value(testCoord, testTime);
  true_val = 4;
  EXPECT_NEAR(value, true_val, 0.0001);

  // 231024033000 - actual value from dhi table
  testTime = 1698118200;
  value = TestForecastLut.get_value(testCoord, testTime);
  true_val = 263;
  EXPECT_NEAR(value, true_val, 0.0001);
}
