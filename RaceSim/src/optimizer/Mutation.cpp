#include "opt/V2Optimizer.hpp"
#include "utils/Utilities.hpp"

void V2Optimizer::mutate_plan(RacePlan* plan) {
  RUNTIME_EXCEPTION(plan != nullptr, "RacePlan object is null");
  RUNTIME_EXCEPTION(!plan->is_empty(), "RacePlan object to mutate cannot be empty");
  RacePlanCreator::Gen rng_collection;

  if (Config::get_instance()->get_mutation_strategy() == "PreferConstantSpeed") {
    constant_for_deceleration(plan, &rng_collection);
  }

  return;
}

void V2Optimizer::constant_for_deceleration(RacePlan* plan, RacePlanCreator::Gen* rng) {
  const std::vector<std::vector<std::pair<size_t, size_t>>> raw_segments = plan->get_orig_loop_segments();
  const std::vector<std::vector<std::pair<double, double>>> raw_speeds = plan->get_orig_loop_speeds();
  const std::vector<std::vector<bool>> raw_acceleration = plan->get_orig_loop_accelerations();
  const std::vector<std::vector<double>> raw_acceleration_value = plan->get_orig_loop_acceleration_values();
  const std::vector<std::vector<double>> raw_segment_distances = plan->get_orig_loop_segment_distances();
  const BasicLut route_distances = route->get_precomputed_distances();
  std::unordered_map<size_t, size_t> corner_end_map = route->get_corner_end_map();
  std::unordered_map<size_t, size_t> corner_start_map = route->get_corner_start_map();
  const size_t num_blocks = static_cast<int>(std::ceil(plan->get_num_loops() /
                              Config::get_instance()->get_num_repetitions()));
  RUNTIME_EXCEPTION(num_blocks == raw_segments.size(), "raw_segments vector shoud "
                    "have number of loops equal to the number of blocks {}", num_blocks);

  for (size_t block_idx=0; block_idx < num_blocks; block_idx++) {
    std::vector<std::pair<size_t, size_t>> loop_segments = raw_segments[block_idx];
    std::vector<std::pair<double, double>> loop_speeds = raw_speeds[block_idx];
    std::vector<bool> loop_acceleration_segments = raw_acceleration[block_idx];
    std::vector<double> loop_acceleration_values = raw_acceleration_value[block_idx];
    std::vector<double> loop_segment_distances = raw_segment_distances[block_idx];
    RacePlanCreator::LoopData loop_data(loop_segments, loop_speeds, loop_acceleration_segments,
                                        loop_acceleration_values, loop_segment_distances);
    const size_t num_segments = loop_segments.size();
    for (size_t seg_idx=0; seg_idx < num_segments; seg_idx++) {
      // Do nothing about segments that don't decelerate
      if (raw_acceleration_value[block_idx][seg_idx] >= 0.0) {
        continue;
      }
      const size_t seg_start = loop_segments[seg_idx].first;
      size_t seg_end = loop_segments[seg_idx].second;

      // We only bias towards constant speeds for "normal straights".
      // This means that the segment directly after this deceleration segment should include
      // the corner that was being decelerated towards
      RUNTIME_EXCEPTION(seg_idx != num_segments - 1, "Deceleration cannot be the last segment of a loop");
      const size_t theoretical_corner_end = loop_segments[seg_idx + 1].second;
      if (corner_end_map.find(theoretical_corner_end) == corner_end_map.end()) {
        // This is likely an aggressive straight. Don't do anything about these
        continue;
      }
      const size_t corner_idx = corner_end_map[theoretical_corner_end];
      const size_t corner_max_speed = route->get_cornering_speed_bounds()[corner_idx];
      // Sample a probability to do this at all?

      // Check if the starting speed of the deceleration is less than the maximum corner speed.
      // If it is, then we can remove the deceleration and maintain the constant speed
      if (corner_max_speed > loop_speeds[seg_idx].first) {
        // Delete the deceleration segment and merge with the constant speed segment
        // Create the new constant speed segment
        std::pair<size_t, size_t> new_segment = {loop_segments[seg_idx].first,
                                                  loop_segments[seg_idx+1].second};
        std::pair<double, double> new_segment_speeds = {loop_speeds[seg_idx].first,
                                                        loop_speeds[seg_idx].first};
        RacePlanCreator::SegmentData seg_data(
          new_segment, new_segment_speeds, false, 0.0,
          route_distances.get_value(new_segment.first, new_segment.second));

        // Replace the deceleration segment and corner segment with the new segment
        loop_data.delete_range(seg_idx, seg_idx + 2);
        loop_data.insert_segment(seg_idx, seg_data);

        // Attempt to modify the segment after the corner to be compatible with the new
        // corner speed
        if (seg_idx + 2 > num_segments) {
          // Not sure if this case can actually occur since even in the last segments, the car would
          // hold a constant speed until the end of the loop
          continue;
        }
        if (loop_acceleration_segments[seg_idx + 2]) {
          const double next_speed = loop_speeds[seg_idx + 2].second;
          if (next_speed > loop_speeds[seg_idx].first) {
            // In the case where the next segment is acceleration, we can directly replace the
            // starting speed since the acceleration magnitude will now be smaller
            new_segment = loop_segments[seg_idx + 2];
            new_segment_speeds = {next_speed, loop_speeds[seg_idx + 2].second};
            double acceleration_value = calc_acceleration(new_segment_speeds.first,
                                                          new_segment_speeds.second,
                                                          loop_segment_distances[seg_idx + 2]);
            RacePlanCreator::SegmentData next_seg_data(new_segment, new_segment_speeds,
                                                        acceleration_value != 0.0, acceleration_value,
                                                        loop_segment_distances[seg_idx + 2]);
            loop_data.insert_segment(seg_idx + 1, next_seg_data);
          } else {
            // In the case of deceleration, must respect maximum deceleration
            const double required_deceleration = calc_acceleration(loop_speeds)
          }
        } else {

        }
      }
    }
  }
}

// RacePlanCreator::LoopData V2Optimizer::legalize_loop(RacePlanCreator::LoopData* loop_data, size_t starting_idx,
//                                                   RacePlanCreator::Gen* rng) {
//   RUNTIME_EXCEPTION(loop_data != nullptr, "Loop data is null");
//   size_t corner_idx = starting_idx;
//   const size_t num_segments = loop_data->loop_segments.size();
//   RacePlanCreator::LoopData new_loop = *loop_data;
//   new_loop.slice_loop(0, corner_idx);
//   RacePlanCreator::PlanHistory history(loop_data->loop_segment_speeds.back().second,
//                                         starting_idx);
  
//   while (corner_idx < num_segments) {
//     if (RacePlanCreator::create_segments(corner_idx, &new_loop, false, rng,
//                                           history, FileLogger("dummy", false)));
//   }
// }
