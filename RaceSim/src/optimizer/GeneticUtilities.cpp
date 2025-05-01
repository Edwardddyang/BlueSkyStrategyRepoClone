#include "opt/GeneticUtilities.hpp"
#include "utils/Defines.hpp"
#include "config/Config.hpp"

RacePlan mutate_plan(RacePlan plan, const std::shared_ptr<Route> route) {
  RUNTIME_EXCEPTION(!plan.is_empty(), "RacePlan object to mutate cannot be empty");
  const std::vector<std::vector<std::pair<size_t, size_t>>> raw_segments = plan.get_orig_loop_segments();
  const std::vector<std::vector<std::pair<double, double>>> raw_speeds = plan.get_orig_loop_speeds();
  const std::vector<std::vector<bool>> raw_acceleration = plan.get_orig_loop_accelerations();
  const std::vector<std::vector<double>> raw_acceleration_value = plan.get_orig_loop_acceleration_values();
  const std::vector<std::vector<double>> raw_segment_distances = plan.get_orig_loop_segment_distances();
  std::unordered_map<size_t, size_t> corner_end_map = route->get_corner_end_map();
  std::unordered_map<size_t, size_t> corner_start_map = route->get_corner_start_map();

  if (Config::get_instance()->get_mutation_strategy() == "PreferConstantSpeed") {
    const size_t num_blocks = static_cast<int>(std::ceil(plan.get_num_loops() /
                                Config::get_instance()->get_num_repetitions()));
    RUNTIME_EXCEPTION(num_blocks == raw_segments.size(), "raw_segments vector shoud "
                      "have number of loops equal to the number of blocks {}", num_blocks);

    for (size_t block_idx=0; block_idx < num_blocks; block_idx++) {
      const size_t num_segments = raw_segments[block_idx].size();
      for (size_t seg_idx=0; seg_idx < num_segments; seg_idx++) {
        // Do nothing about segments that don't decelerate
        if (raw_acceleration_value[block_idx][seg_idx] >= 0.0) {
          continue;
        }
        const size_t seg_start = raw_segments[block_idx][seg_idx].first;
        size_t seg_end = raw_segments[block_idx][seg_idx].second;

        // We only bias towards constant speeds for "normal straights".
        // This means that the segment directly after this deceleration segment should include
        // the corner that this deceleration is for
        RUNTIME_EXCEPTION(seg_idx != num_segments - 1, "Deceleration cannot be the last segment");
        const size_t theoretical_corner_end = raw_segments[block_idx][seg_idx + 1].second;
        if (corner_end_map.find(theoretical_corner_end) == corner_end_map.end()) {
          // This is likely an aggressive straight. Don't do anything about these
          continue;
        }
        const size_t corner_idx = corner_end_map[theoretical_corner_end];
        const size_t corner_max_speed = route->get_cornering_speed_bounds()[corner_idx];
        // Sample a probability to do this at all?

        // Check if the starting speed of the deceleration is less than the maximum corner speed.
        // If it is, then we can remove the deceleration and maintain the constant speed
        if (corner_max_speed > raw_speeds[block_idx][seg_idx].first) {
          // Delete the deceleration segment and merge with the constant speed segment
        }

      }
    }
  }

  return RacePlan();
}