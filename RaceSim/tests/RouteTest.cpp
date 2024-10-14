#include <gtest/gtest.h>
#include "Route.hpp"
#include "Config.hpp"
#include "Units.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>

/*
TEST(routeTest,  UniformSegmentTestBaseRoute) {
    CONFIG_FILE_PATH = "data/config/wsc_config.yaml";
    // base_route_path = "./data/luts/wsc_2023/static/baseroute.csv";
    Route new_route = Route();
    int num_points = new_route.get_num_points();
    int true_num_points = 28844;
    EXPECT_EQ(num_points, true_num_points);
    int num_segments = new_route.get_num_segments();
    EXPECT_EQ(num_segments, 1);

    new_route.segment_route_uniform(100000);
    num_segments = new_route.get_num_segments();
    
    EXPECT_NEAR(num_segments, 30, 5);
    
    //Can print all the lengths of the segments to console using
    std::vector<double> seg_lengths = new_route.get_segment_lengths();

    std::cout << "Lengths: ";
    for (auto i: seg_lengths)
        std::cout << i << ' ';
    
    
}
*/
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    
    const char* strat_root = std::getenv("STRAT_ROOT");
    RUNTIME_EXCEPTION(strat_root != nullptr, "No STRAT_ROOT environment variable detected. Set it to the full path to gen12_strategy/RaceSim.");

    Config::initialize("data/config/wsc_config_route_tests.yaml", std::string(strat_root));
    return RUN_ALL_TESTS();
}


TEST(routeTest,  UniformSegmentTestNewRoute) {
    Route new_route = Route();
    int num_points = new_route.get_num_points();
    int true_num_points = 10;
    EXPECT_EQ(num_points, true_num_points);
    int num_segments = new_route.get_num_segments();
    EXPECT_EQ(num_segments, 1);

    new_route.segment_route_uniform(200000);

    num_segments = new_route.get_num_segments();
    EXPECT_EQ(num_segments, 9);
    
    //Can print all the lengths of the segments to console using
    
    // std::vector<double> seg_lengths = new_route.get_segment_lengths();

    // for (auto i: seg_lengths)
    //     std::cout << i << ' ';

    new_route.segment_route_uniform(800000);

    num_segments = new_route.get_num_segments();
    EXPECT_EQ(num_segments, 3);
    
    //Can print all the lengths of the segments to console using
    
    // seg_lengths = new_route.get_segment_lengths();

    // for (auto i: seg_lengths)
    //     std::cout << i << ' ';
}
