#include <stdlib.h>
#include <gtest/gtest.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "route/Route.hpp"
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
