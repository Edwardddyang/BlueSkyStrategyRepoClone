/** This file provides functions for crossover, mutation and initial population
 * creation */

#pragma once

#include <memory>
#include <random>
#include <stack>
#include <utility>
#include <vector>

#include "SimUtils/Logger.hpp"
#include "SimUtils/Luts.hpp"
#include "SimUtils/Types.hpp"
#include "route/RacePlan.hpp"
#include "route/Route.hpp"

// Class for creating FSGPRacePlan objects in the initial population.
// FSGPRacePlan creation as done according to the process described in. This
// class also provides functions for creating race plans on existing loops used
// for mutation in the genetic optimizer
// https://www.notion.so/blueskysolar/Race-Strategy-and-Testing-Process-1da8d1f46c3680a79056edf3c9fecd1b?pvs=4
class RacePlanCreator {
 public:
  ////////////////////////////////////////////////////////////////////////////////////////////////
  // The following structs are used in create_plan() to hold intermediate data
  // as the race plan // is being created //
  ////////////////////////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////////////////////
  ////////////////////// NOTE ABOUT TERMINOLOGY USED
  //////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////
  // "route index": Denotes an index in the route_points_array
  // "corner index": Denotes an index in the cornering_segment_bounds array of
  // the Route object "real corner": A corner where the maximum cornering speed
  // is less than the route's speed limit "segment index": An index inside
  // loop_segments, loop_segment_speeds ... "wrap-around corner": The first
  // corner of the route which the car wraps to after taking the entire route

  // Keeps track of the history of different data elements added to the race
  // plan e.g. number of segments added for each corner, speed of each corner
  // etc. This is required when rolling back to a previous corner after segment
  // generation failed
  struct PlanHistory {
    // The chosen speed for each real corner
    std::stack<double> real_corner_speeds;
    // Real corner indices for which segments were created
    std::stack<size_t> real_corner_indices;
    // Speed of the first corner of a generated loop
    std::stack<uint64_t> first_corner_speeds;

    PlanHistory() {
      real_corner_speeds.push(0.0);
      real_corner_indices.push(0);
      first_corner_speeds.push(0);
    }

    PlanHistory(double real_corner_speed, size_t real_corner_idx,
                uint64_t num_segments = 0, uint64_t first_speed = 0,
                uint64_t num_added_segments = 0) {
      real_corner_speeds.push(real_corner_speed);
      real_corner_indices.push(real_corner_idx);
      first_corner_speeds.push(first_speed);
    }
  };

  // Holds the FSGPRacePlan attributes that will be passed into the return
  // object
  struct PlanAttributes {
    // Data holders for each loop before processing when constructing a loop
    // block. For each loop block, we create one loop and modify the beginning
    // or ending segments in order to glue the loops of the block together.
    // These vectors hold the uniquely created loop for each block
    FSGPRacePlan::PlanData raw_segments;

    // FSGPRacePlan attributes
    FSGPRacePlan::PlanData all_segments;

    void add_loop(const FSGPRacePlan::LoopData& loop_data, size_t start_idx,
                  size_t end_idx);

    PlanAttributes(FSGPRacePlan::PlanData raw_loop_segments = {},
                   FSGPRacePlan::PlanData all_segments = {})
        : raw_segments(raw_loop_segments), all_segments(all_segments) {}
  };

  /** Initialize all parameters */
  RacePlanCreator(std::shared_ptr<FSGPRoute> route, unsigned speed_seed = 1,
                  unsigned loop_seed = 1, unsigned aggressive_seed = 1,
                  unsigned idx_seed = 1, unsigned acceleration_seed = 1,
                  unsigned skip_seed = 1);

  /** @brief Create a single race plan */
  FSGPRacePlan create_plan();

  /** @brief Helper to create_plan used to create a single loop block
   * @param seg_data Intermediate segment data for the latest created segment
   * @param loop_data Intermediate data for the last created loop
   * @param attributes All data for the entire race plan to the current point
   * @param route Race route
   * @param logger Logger for the race plan creation
   * @param num_loops_in_block Number of loops in the block
   * @param is_last_block Whether the block to be created is the last block of
   * the entire plan
   * @param is_first_block Whether the block to be created is the first block of
   * the entire plan
   */
  void create_loop_block(FSGPRacePlan::LoopData* loop_data,
                         int num_loops_in_block = 1, bool is_last_block = false,
                         bool is_first_block = false,
                         PlanAttributes* att = nullptr,
                         FileLogger* logger = nullptr);

  // Create only one rng at the beginning of create_plan and pass it to
  // create_segments each corner so that the state is constantly changed
  struct Gen {
    std::mt19937 speed_rng;
    std::mt19937 loop_rng;
    std::mt19937 aggressive_rng;
    std::mt19937 idx_rng;
    std::mt19937 acceleration_rng;
    std::mt19937 skip_rng;

    Gen(unsigned int speed_seed = 1, unsigned int loop_seed = 1,
        unsigned int aggressive_seed = 1, unsigned int idx_seed = 1,
        unsigned int acceleration_seed = 1, unsigned int skip_seed = 1) {
      speed_rng.seed(speed_seed);
      loop_rng.seed(loop_seed);
      aggressive_rng.seed(aggressive_seed);
      idx_rng.seed(idx_seed);
      acceleration_rng.seed(acceleration_seed);
      skip_rng.seed(skip_seed);
    }
  };

  /** @brief Helper to create_plan used to create segments for a single corner
   */
  bool create_segments(size_t corner_idx, FSGPRacePlan::LoopData* loop_data,
                       bool is_first_segment, Gen* rng, PlanHistory* history,
                       FileLogger& logger);  // NOLINT

  /** @brief Helper to create_plan used for rolling back a corner after failed
   * segment generation
   * @return Index of last real corner to which we are rolling back
   */
  size_t rollback_to_last_real_corner(size_t corner_idx, PlanHistory* history,
                                      FSGPRacePlan::LoopData* loop_data,
                                      PlanAttributes* att,
                                      FileLogger& logger);  // NOLINT

 private:
  // Route for which to create a Race Plan
  const std::shared_ptr<FSGPRoute> route;
  // Attributes about the route
  std::vector<Coord> route_points;
  std::vector<std::pair<size_t, size_t>> cornering_segment_bounds;
  std::vector<double> cornering_speed_bounds;
  BasicLut route_distances;
  double max_route_speed;
  size_t num_points;
  Gen rng_collection;

  // Maximum number of loops to create
  int max_num_loops;

  // If true, then max_num_loops will be created for any plan. Otherwise, a
  // number will be sampled
  bool fix_loops;

  // Mass of the car in kg
  double car_mass;

  // Maximum speed of the car in m/s
  double max_speed;

  // Maximum instataneous motor power draw in W
  double max_motor_power;

  // Maximum magnitude for acceleration in m/s^2. This should be positive
  double max_acceleration;

  // Maximum deceleration in m/s^2. This should be negative
  double max_deceleration;

  // Minimum acceleration to consider when sampling in m/s^2
  double min_acceleration;

  // The target average speed of the car in m/s. Speed distributions
  // will be centered around this number
  double target_average_speed;

  // Start time of the race plan (Unused)
  Time start_time;

  // End time of the race plan (Unused)
  Time end_time;

  // Sampling seeds for sampling distributions
  unsigned speed_seed;
  unsigned loop_seed;
  unsigned aggressive_seed;
  unsigned idx_seed;
  unsigned acceleration_seed;
  unsigned skip_seed;

  // Minimum ratio of the maximum cornering speed to consider
  double corner_speed_min_ratio;

  // Maximum ratio of the maximum cornering speed to choose from
  double corner_speed_max_ratio;

  // Threshold value in meters for taking a straight aggressively. If a straight
  // is longer than this this distance, then the car will accelerate, hold a
  // constant speed before decelerating
  double aggressive_straight_threshold;

  // Number of loops to repeat in a single loop block
  int num_repetitions;

  // Fraction of total motor power that can be allocated towards acceleration
  double acceleration_power_budget;

  // Amount of motor power in W that can be allocated towards acceleraiton
  double acceleration_power_allowance;

  // Number of iterations to try when sampling speeds, distances, indices,
  // accelerations etc.
  double max_iters;

  // Whether to log the segmentation process to a file. Used for debugging
  bool log;

  // Number of corners in the route
  int num_corners;
};
