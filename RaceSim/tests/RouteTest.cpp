#include <stdlib.h>
#include <gtest/gtest.h>

#include <iostream>
#include <fstream>
#include <string>

#include "route/Route.hpp"
#include "config/Config.hpp"
#include "utils/Units.hpp"

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  char* strat_root = std::getenv("STRAT_ROOT");
  RUNTIME_EXCEPTION(strat_root != nullptr, "No STRAT_ROOT environment variable detected."
                                           "Set it to the full path to gen12_strategy/RaceSim.");

  std::filesystem::path config_path("data/config/test_config.yaml");
  Config::initialize(config_path, strat_root);
  return RUN_ALL_TESTS();
}


TEST(routeTest, UniformSegmentTestNewRoute) {
  Route new_route = Route();
  int num_points = new_route.get_num_points();
  int true_num_points = 10;
  EXPECT_EQ(num_points, true_num_points);
  int num_segments = new_route.get_num_segments();
  EXPECT_EQ(num_segments, 1);

  new_route.segment_route_uniform(200000);

  num_segments = new_route.get_num_segments();
  EXPECT_EQ(num_segments, 9);

  new_route.segment_route_uniform(800000);

  num_segments = new_route.get_num_segments();
  EXPECT_EQ(num_segments, 3);
}
