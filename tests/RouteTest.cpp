#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

#include "route/Route.hpp"

TEST(WSCRouteTest, UniformSegmentTestNewRoute) {  // NOLINT
  const struct WSCRouteParams params(20.0, {1, 2, 3, 4});
  const std::filesystem::path path =
      std::string(LUTS_PATH) + "test/uniform_segment_testing.csv";
  const WSCRoute route(params, path);
  std::vector<std::pair<size_t, size_t>> segments =
      route.segment_route_uniform(route.get_route_length());

  const size_t num_points = route.get_num_points();
  const int true_num_points = 10;
  EXPECT_EQ(num_points, true_num_points);

  EXPECT_EQ(segments.size(), 1);

  segments = route.segment_route_uniform(200000);
  EXPECT_EQ(segments.size(), 9);

  segments = route.segment_route_uniform(800000);
  EXPECT_EQ(segments.size(), 3);
}
