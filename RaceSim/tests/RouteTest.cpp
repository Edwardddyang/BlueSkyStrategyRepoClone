#include <stdlib.h>
#include <gtest/gtest.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <utility>

#include "route/Route.hpp"
#include "utils/Luts.hpp"
#include "utils/CustomTime.hpp"
#include "utils/Units.hpp"
#include "utils/Defines.hpp"

class RouteTest : public ::testing::Test {
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
std::filesystem::path RouteTest::strat_root_path;

TEST_F(RouteTest, UniformSegmentTestNewRoute) {
  std::filesystem::path base_route_path = RouteTest::strat_root_path /
                                          "data/luts/TestData/uniform_segment_testing.csv";
  Route new_route = Route(base_route_path);
  std::vector<std::pair<size_t, size_t>> segments = new_route.segment_route_uniform(new_route.get_route_length());
  int num_points = new_route.get_num_points();
  int true_num_points = 10;
  EXPECT_EQ(num_points, true_num_points);

  EXPECT_EQ(segments.size(), 1);

  segments = new_route.segment_route_uniform(200000);
  EXPECT_EQ(segments.size(), 9);

  segments = new_route.segment_route_uniform(800000);
  EXPECT_EQ(segments.size(), 3);
}

TEST_F(RouteTest, ForecastLutTest) {
  std::filesystem::path dni_path = RouteTest::strat_root_path / "data/luts/TestData/dni.csv";
  ForecastLut dni_lut(dni_path);

  const Coord starting_coord(-12.639543, 131.074720);
  const Time starting_time("2023-10-22 09:00:00", -9.5);

  std::pair<size_t, size_t> caches = dni_lut.initialize_caches(starting_coord, starting_time.get_utc_time_point());

  // The starting values appear between indices 21 and 22, but is closer to 21
  EXPECT_EQ(caches.first, 20);
  EXPECT_EQ(caches.second, 12);

  // Times exactly in between the current and next row should use the current row cache
  Coord next_coord(-12.655914, 131.077636);
  Time next_time("2023-10-22 09:15:00", -9.5);
  dni_lut.update_index_cache(&caches, next_coord, next_time.get_utc_time_point());
  EXPECT_EQ(caches.first, 21);
  EXPECT_EQ(caches.second, 12);

  // Advance time by a second, time cache should use next column
  next_time.update_time_seconds(1.0);
  dni_lut.update_index_cache(&caches, next_coord, next_time.get_utc_time_point());
  EXPECT_EQ(caches.first, 21);
  EXPECT_EQ(caches.second, 13);
}
