#include "opt/V2Optimizer.hpp"
#include "utils/Utilities.hpp"

void V2Optimizer::mutate_plan(RacePlan* plan) {
  RUNTIME_EXCEPTION(plan != nullptr, "RacePlan object is null");
  RUNTIME_EXCEPTION(!plan->is_empty(), "RacePlan object to mutate cannot be empty");

  const std::string mutation_type = Config::get_instance()->get_mutation_strategy();
  RUNTIME_EXCEPTION(MUTATION_STRATEGIES.find(mutation_type) != MUTATION_STRATEGIES.end(),
                    "Unrecognized mutation strategy {}", mutation_type);

  if (mutation_type == "ConstantForDeceleration") {
    *plan = constant_for_deceleration(plan, &rng_collection);
  } else if (mutation_type == "GaussianNoise") {
    *plan = gaussian_noise(plan, &rng_collection);
  } else if (mutation_type == "ConstantForAcceleration") {
    *plan = constant_for_acceleration(plan, &rng_collection);
  }

  return;
}

RacePlan V2Optimizer::gaussian_noise(RacePlan* plan, RacePlanCreator::Gen* rng) {
  RUNTIME_EXCEPTION(rng != nullptr, "rng struct for gaussian noise is null");
  return *plan;
}

RacePlan V2Optimizer::constant_for_acceleration(RacePlan* plan, RacePlanCreator::Gen* rng) {
  RUNTIME_EXCEPTION(rng != nullptr, "rng struct for gaussian noise is null");
  return *plan;
}

RacePlan V2Optimizer::constant_for_deceleration(RacePlan* plan, RacePlanCreator::Gen* rng) {
  RUNTIME_EXCEPTION(rng != nullptr, "rng struct for constant_for_deceleration is null");
  // Get raw plan data
  const RacePlan::PlanData raw_plan = plan->get_orig_segments();
  RacePlan::PlanData new_raw_plan = plan->get_orig_segments();
  const BasicLut route_distances = route->get_precomputed_distances();
  RacePlanCreator::PlanAttributes att;
  std::unordered_set<size_t> corner_end_indices = route->get_corner_end_indices();
  std::unordered_map<size_t, size_t> corner_end_map = route->get_corner_end_map();
  const size_t num_blocks = static_cast<int>(std::ceil(plan->get_num_loops() /
                              Config::get_instance()->get_num_repetitions()));

  RUNTIME_EXCEPTION(num_blocks == raw_plan.size(), "raw_segments vector shoud "
                    "have number of loops equal to the number of blocks {}", num_blocks);
  for (size_t block_idx=0; block_idx < num_blocks; block_idx++) {
    RacePlan::LoopData& loop_segments = new_raw_plan[block_idx];
    mutation_logger("Mutating block " + std::to_string(block_idx) + ": ");
    mutation_logger(RacePlan::get_loop_string(loop_segments));

    const size_t num_segments = loop_segments.size();
    for (size_t seg_idx=0; seg_idx < num_segments; seg_idx++) {
      // Do nothing about segments that don't decelerate
      if (loop_segments[seg_idx].acceleration_value >= 0.0) {
        continue;
      }
      const size_t seg_start = loop_segments[seg_idx].start_idx;
      size_t seg_end = loop_segments[seg_idx].end_idx;

      // We only bias towards constant speeds for "normal straights".
      // This means that the segment directly after this deceleration segment should include
      // the corner that was being decelerated towards
      if (seg_idx == num_segments - 1) {
        continue;
      }
      const size_t theoretical_corner_end = loop_segments[seg_idx + 1].end_idx;
      if (corner_end_indices.find(theoretical_corner_end) == corner_end_indices.end()) {
        // This is likely an aggressive straight. Don't do anything about these
        continue;
      }
      const size_t corner_idx = corner_end_map[theoretical_corner_end];
      mutation_logger("Replacing straight leading to corner index " + std::to_string(corner_idx) +
                      ", Segment Index: " + std::to_string(seg_idx));
      const size_t corner_max_speed = route->get_cornering_speed_bounds()[corner_idx];

      const double probability = 0.3;
      // Pick a corner speed. If the current speed is already >= the target average speed of the course,
      // then bias towards maintaining this constant speed rather than trying to accelerate
      // bool maintain_speed = false;
      // if (last_real_corner_speed >= target_average_speed && last_real_corner_speed < max_corner_speed) {
      //   logger("Last corner speed is greater or equal to the target average speed. Sample chance to maintain"
      //           " same corner speed.");
      //   const double sample = skip_dist(rng->skip_rng);
      //   if (sample < probability) {
      //     proposed_corner_speed = last_real_corner_speed;
      //     maintain_speed = true;
      //     logger("Maintaining last corner speed of " + std::to_string(last_real_corner_speed) + "m/s");
      //   }
      // }
      // Check if the starting speed of the deceleration is less than the maximum corner speed.
      // If it is, then we can remove the deceleration and maintain the constant speed
      bool successful_mutation = false;
      if (corner_max_speed > loop_segments[seg_idx].start_speed) {
        // Create the constant speed segments for the straight AND the corner
        RacePlan::SegmentData straight_seg_data = loop_segments[seg_idx];
        const double constant_speed = straight_seg_data.start_speed;
        straight_seg_data.acceleration_value = 0.0;
        straight_seg_data.end_speed = constant_speed;

        RacePlan::SegmentData corner_seg_data = loop_segments[seg_idx + 1];
        corner_seg_data.start_speed = constant_speed;
        corner_seg_data.end_speed = constant_speed;

        RacePlan::SegmentData orig_straight_seg_data = loop_segments[seg_idx];
        RacePlan::SegmentData orig_corner_seg_data = loop_segments[seg_idx + 1];

        // Replace the deceleration segment and corner segment with the new segment
        mutation_logger("Inserting the following segment at index " + std::to_string(seg_idx)
                        + "\n" + RacePlan::get_segment_string(straight_seg_data));
        
        new_raw_plan[block_idx][seg_idx] = straight_seg_data;
        mutation_logger("Inserting the following segment at index " + std::to_string(seg_idx + 1)
                        + "\n" + RacePlan::get_segment_string(corner_seg_data));
        new_raw_plan[block_idx][seg_idx + 1] = corner_seg_data;

        if (!legalize_loop(new_raw_plan, block_idx, seg_idx+1, &mutation_logger)) {
          mutation_logger("Loop legalization failed - restoring added segments\n");
          new_raw_plan[block_idx][seg_idx] = orig_straight_seg_data;
          new_raw_plan[block_idx][seg_idx + 1] = orig_corner_seg_data;
        }
      } else {
        mutation_logger("Corner speed is greater than the desired new cruising speed. Skipping");
      }
    }
    mutation_logger("\nNew loop block " + std::to_string(block_idx));
    mutation_logger(RacePlan::get_loop_string(loop_segments));
    att.raw_segments.push_back(new_raw_plan[block_idx]);
    generator->create_loop_block(&new_raw_plan[block_idx], &route_distances, this->num_loops_in_block,
      block_idx == num_blocks - 1, block_idx == 0, &att, &mutation_logger
    );
  }

  // Create new loop plan
  RacePlan new_plan(att.all_segments, att.raw_segments, this->num_loops_in_block);
  return new_plan;
}

bool V2Optimizer::legalize_loop(RacePlan::PlanData& plan,
                                size_t loop_idx,
                                size_t seg_idx,
                                FileLogger* logger) {
  RUNTIME_EXCEPTION(logger != nullptr, "Logger is null");
  RUNTIME_EXCEPTION(loop_idx < plan.size(), "Loop index exceeds no. loops in plan");
  RUNTIME_EXCEPTION(seg_idx < plan[loop_idx].size(), "Segment index exceeds no. segments in plan");

  const RacePlan::PlanData saved_plan = plan;

  (*logger)("Legalizing loop starting with index " + std::to_string(seg_idx));
  const size_t num_loops = plan.size();

  /** @brief Resolve either an index or speed discontinuity between segment i and i+1
   * @param loop The loop to modify
   * @param i "True" segment, i+1 will be modified
   * @param starting_speed Optional parameter for the ending speed of segment i. Used when evaluating the first segment
   * of the next loop
   * @return True if discontinuity resolution was successful, False otherwise
  */
  auto resolve_next_segment = [&](RacePlan::LoopData& loop, size_t i) -> bool {
    // Discontinuity, replace i+1 segment
    RacePlan::SegmentData new_segment = loop[i+1];
    if (loop[i].end_idx != loop[i+1].start_idx) {
      // Index discontinuity
      const double new_distance = this->route_distances.get_value(loop[i].end_idx, loop[i+1].end_idx);
      new_segment.start_idx = loop[i].end_idx;
      new_segment.distance = new_distance;
      // In the case of acceleration, attempt to modify the acceleration
      const double required_acceleration = calc_acceleration(loop[i+1].start_speed,
                                                             loop[i+1].end_speed, 
                                                             new_distance);
      new_segment.acceleration_value = required_acceleration;
    } else {
      // Speed discontinuity
      new_segment.start_speed = loop[i].end_speed;
      const double required_acceleration = calc_acceleration(loop[i].end_speed,
                                                            loop[i+1].end_speed,
                                                            new_segment.distance);
      new_segment.acceleration_value = required_acceleration;
    }
    // Check acceleration constraints
    const double drawn_power = new_segment.acceleration_value * this->car_mass * loop[i+1].end_speed;
    if (new_segment.acceleration_value > 0.0) {
      if (drawn_power < this->acceleration_power_budget ||
          new_segment.acceleration_value > this->max_acceleration) {
        return false;
      }
    } else if (new_segment.acceleration_value < 0.0) {
      if (new_segment.acceleration_value < this->max_deceleration) {
        return false;
      }
    }
    plan[loop_idx][i+1] = new_segment;
    return true;
  };

  bool resolve_connection_segments = false;
  double carry_over_speed = -1;
  RacePlan::LoopData& loop = plan[loop_idx];
  const size_t num_segments = plan[loop_idx].size();

  int last_real_corner_end_idx;
  const auto corner_end = this->route->get_corner_end_indices();
  bool crossed_over = false;
  for (size_t i=loop.size() - 1; i >= 0; i--) {
    const size_t end_idx = loop[i].end_idx;
    const size_t start_idx = loop[i].start_idx;
    if (end_idx < start_idx) {
      crossed_over = true;
    }
    if (crossed_over && corner_end.find(end_idx) != corner_end.end()) {
      last_real_corner_end_idx = end_idx;
      break;
    }
  }

  int first_real_corner_end_idx = 0;
  crossed_over = false;
  for (size_t i=0; i < loop.size(); i++) {
    const size_t end_idx = loop[i].end_idx;
    const size_t start_idx = loop[i].start_idx;
    if (end_idx < start_idx || (loop_idx == 0 && start_idx == 0)) {
      crossed_over = true;
    }
    if (crossed_over && corner_end.find(end_idx) != corner_end.end()) {
      first_real_corner_end_idx = end_idx;
      break;
    }
  }
  (*logger)("End of first real corner is: " + std::to_string(first_real_corner_end_idx));

  // Do not attempt to modify the speed of the first segment
  (*logger)("Plan end of thing " + std::to_string(plan[loop_idx][seg_idx].end_idx));
  if (plan[loop_idx][seg_idx].end_idx == first_real_corner_end_idx) {
    return false;
  }

  for (size_t i=seg_idx; i < num_segments - 1; i++) {
    // Check continuity of indices and speeds. If continuous from current segment to the next,
    // return success
    bool continuous_with_next_segment = loop[i].end_idx == loop[i+1].start_idx &&
                                        loop[i].end_speed == loop[i+1].start_speed;
    if (loop[i+1].end_idx == first_real_corner_end_idx) {
      plan = saved_plan;
      return false;
    }

    if (i == num_segments - 2 && !continuous_with_next_segment && plan.size() - 1 > loop_idx) {
      (*logger)("Could not resolve wrap-around segment - giving up");
      plan = saved_plan;
      return false;
    }
    if (loop[i].end_idx == last_real_corner_end_idx && plan.size() - 1 > loop_idx && !continuous_with_next_segment) {
      // For the last corner of the loop, need to check continuity with the connecting segment of the next loop
      // and with the wrap-around segments ONLY until the first corner
      resolve_connection_segments = true;
      carry_over_speed = loop[i].end_speed;
    }
    if (continuous_with_next_segment) {
      break;
    }
    // Discontinuity, replace i+1 segment
    if (!resolve_next_segment(loop, i)) {
      plan = saved_plan;
      return false;
    }
  }

  if (resolve_connection_segments) {
    RUNTIME_EXCEPTION(loop_idx + 1 < plan.size() && carry_over_speed != -1, "Invalid parameters to resolve next loop");
    RacePlan::LoopData& next_loop = plan[loop_idx + 1];

    // Evaluate if the starting speed of the next loop can be modified
    next_loop[0].start_speed = carry_over_speed;
    next_loop[0].acceleration_value = calc_acceleration(next_loop[0].start_speed,
                                                        next_loop[0].end_speed,
                                                        next_loop[0].distance); 
    // Check acceleration constraints
    const double drawn_power = next_loop[0].acceleration_value * this->car_mass * next_loop[0].end_speed;
    if (next_loop[0].acceleration_value > 0.0) {
      if (drawn_power < this->acceleration_power_budget ||
          next_loop[0].acceleration_value > this->max_acceleration) {
        plan = saved_plan;
        return false;
      }
    } else if (next_loop[0].acceleration_value < 0.0) {
      if (next_loop[0].acceleration_value < this->max_deceleration) {
        plan = saved_plan;
        return false;
      }
    }
  }

  (*logger)("Legalization successful");
  return true;
}
