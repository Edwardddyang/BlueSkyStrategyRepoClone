#include "opt/V2Optimizer.hpp"
#include "utils/Utilities.hpp"

void V2Optimizer::mutate_plan(RacePlan* plan) {
  RUNTIME_EXCEPTION(plan != nullptr, "RacePlan object is null");
  RUNTIME_EXCEPTION(!plan->is_empty(),
                    "RacePlan object to mutate cannot be empty");

  const std::string mutation_type =
      Config::get_instance().get_mutation_strategy();
  RUNTIME_EXCEPTION(
      MUTATION_STRATEGIES.find(mutation_type) != MUTATION_STRATEGIES.end(),
      "Unrecognized mutation strategy {}", mutation_type);

  if (mutation_type == "ConstantForDeceleration") {
    constant_for_deceleration(plan, &rng_collection);
  } else if (mutation_type == "AccelerationNoise") {
    acceleration_noise(plan, &rng_collection);
  } else if (mutation_type == "ConstantNoise") {
    constant_noise(plan, &rng_collection);
  } else if (mutation_type == "Mix") {
    mix_mutation(plan, &rng_collection);
  }

  return;
}

void V2Optimizer::mix_mutation(RacePlan* plan, RacePlanCreator::Gen* rng) {
  RUNTIME_EXCEPTION(rng != nullptr, "rng struct for corner noise is null");
  RUNTIME_EXCEPTION(plan != nullptr, "Plan in mix mutation is null");
  std::uniform_int_distribution<int> mutation_dist(
      0, MUTATION_STRATEGIES.size() - 1);
  std::vector<std::string> mutations_vec(MUTATION_STRATEGIES.begin(),
                                         MUTATION_STRATEGIES.end());

  // Get raw plan
  RacePlan::PlanData new_raw_plan = plan->get_orig_segments();
  RacePlanCreator::PlanAttributes att;

  const size_t total_num_blocks = static_cast<int>(std::ceil(
      plan->get_num_loops() / Config::get_instance().get_num_repetitions()));

  const size_t num_loops = plan->get_orig_segments().size();
  for (size_t i = 0; i < num_loops; i++) {
    const size_t idx = mutation_dist(rng->idx_rng);
    // Default to ConstantForDeceleration mutation
    const std::string mutation_strategy = mutations_vec[idx] != "Mix"
                                              ? mutations_vec[idx]
                                              : "ConstantForDeceleration";
    mutation_logger("Selected " + mutation_strategy + " for block " +
                    std::to_string(i));
    if (mutation_strategy == "ConstantForDeceleration") {
      constant_for_deceleration_loop(&new_raw_plan, i, &rng_collection);
    } else if (mutation_strategy == "AccelerationNoise") {
      acceleration_noise_loop(&new_raw_plan, i, rng);
    } else if (mutation_strategy == "ConstantNoise") {
      constant_noise_loop(&new_raw_plan, i, rng);
    }
    att.raw_segments.push_back(new_raw_plan[i]);
    generator->create_loop_block(&new_raw_plan[i], this->num_loops_in_block,
                                 i == total_num_blocks - 1, i == 0, &att,
                                 &mutation_logger);
    mutation_logger("Finished creating loop block");
  }

  mutation_logger("Finished mixed mutation");
  *plan =
      RacePlan(att.all_segments, att.raw_segments, this->num_loops_in_block);
}

void V2Optimizer::constant_noise_loop(RacePlan::PlanData* new_raw_plan,
                                      size_t loop_idx,
                                      RacePlanCreator::Gen* rng) {
  RUNTIME_EXCEPTION(new_raw_plan != nullptr, "Plan is null");
  RUNTIME_EXCEPTION(new_raw_plan->size() > loop_idx,
                    "Not enough loops in the plan");
  RUNTIME_EXCEPTION(rng != nullptr, "rng collection is null");

  RacePlan::LoopData& loop_segments = (*new_raw_plan)[loop_idx];
  mutation_logger("Mutating block " + std::to_string(loop_idx) + ": ");
  mutation_logger(RacePlan::get_loop_string(loop_segments));
  std::uniform_real_distribution<double> skip_dist(0.0, 1.0);
  std::uniform_real_distribution<double> noise_dist(-1.0, 1.0);

  const size_t num_segments = loop_segments.size();
  for (size_t seg_idx = 1; seg_idx < num_segments; seg_idx++) {
    // Find a constant speed segment that is not a corner
    if (loop_segments[seg_idx].acceleration_value != 0.0) {
      continue;
    }

    const size_t seg_start = loop_segments[seg_idx].start_idx;
    size_t seg_end = loop_segments[seg_idx].end_idx;

    // Pick a corner speed. If the current speed is already >= the target
    // average speed of the course, then bias towards maintaining this constant
    // speed rather than trying to accelerate
    const double probability = 0.2;
    const double chance = skip_dist(rng->skip_rng);
    if (chance < probability) {
      mutation_logger("Sampled chance - not mutating this corner");
      continue;
    }
    // Sample a random noise to add to the speed
    // Create new constant speed segment
    const int speed_increase = noise_dist(rng->speed_rng);
    RacePlan::SegmentData orig_seg = (*new_raw_plan)[loop_idx][seg_idx];
    (*new_raw_plan)[loop_idx][seg_idx].end_speed += speed_increase;
    (*new_raw_plan)[loop_idx][seg_idx].start_speed += speed_increase;

    // Also need to modify the previous segment
    RacePlan::SegmentData prev_orig_seg =
        (*new_raw_plan)[loop_idx][seg_idx - 1];
    (*new_raw_plan)[loop_idx][seg_idx - 1].end_speed += speed_increase;
    const double new_prev_acceleration_value =
        calc_acceleration((*new_raw_plan)[loop_idx][seg_idx - 1].start_speed,
                          (*new_raw_plan)[loop_idx][seg_idx - 1].end_speed,
                          (*new_raw_plan)[loop_idx][seg_idx - 1].distance);
    (*new_raw_plan)[loop_idx][seg_idx - 1].acceleration_value =
        new_prev_acceleration_value;

    mutation_logger(
        "Trying the following at segment index " + std::to_string(seg_idx) +
        "\n" +
        RacePlan::get_segment_string((*new_raw_plan)[loop_idx][seg_idx]));
    if (!legalize_loop(new_raw_plan, loop_idx, seg_idx - 1, &mutation_logger) ||
        !legalize_loop(new_raw_plan, loop_idx, seg_idx, &mutation_logger)) {
      mutation_logger("Loop legalization failed - restoring added segments\n");
      (*new_raw_plan)[loop_idx][seg_idx] = orig_seg;
      (*new_raw_plan)[loop_idx][seg_idx - 1] = prev_orig_seg;
    }
  }
}

void V2Optimizer::constant_noise(RacePlan* plan, RacePlanCreator::Gen* rng) {
  RUNTIME_EXCEPTION(rng != nullptr, "rng struct for corner noise is null");
  RUNTIME_EXCEPTION(plan != nullptr, "Plan in constant noise mutation is null");
  // Get raw plan
  RacePlan::PlanData new_raw_plan = plan->get_orig_segments();
  RacePlanCreator::PlanAttributes att;

  const size_t total_num_blocks = static_cast<int>(std::ceil(
      plan->get_num_loops() / Config::get_instance().get_num_repetitions()));

  // if loop_idx = -1, mutating all blocks. Otherwise, mutating just one block
  RUNTIME_EXCEPTION(total_num_blocks == new_raw_plan.size(),
                    "raw_segments vector shoud "
                    "have number of loops equal to the number of blocks {}",
                    total_num_blocks);
  for (size_t block_idx = 0; block_idx < total_num_blocks; block_idx++) {
    constant_noise_loop(&new_raw_plan, block_idx, rng);
    att.raw_segments.push_back(new_raw_plan[block_idx]);
    generator->create_loop_block(&new_raw_plan[block_idx],
                                 this->num_loops_in_block,
                                 block_idx == total_num_blocks - 1,
                                 block_idx == 0, &att, &mutation_logger);
    mutation_logger("Finished creating loop block");
  }

  // Create new loop plan
  mutation_logger("Finished gaussian noise mutation");
  *plan =
      RacePlan(att.all_segments, att.raw_segments, this->num_loops_in_block);
}

void V2Optimizer::acceleration_noise_loop(RacePlan::PlanData* new_raw_plan,
                                          size_t loop_idx,
                                          RacePlanCreator::Gen* rng) {
  RUNTIME_EXCEPTION(new_raw_plan != nullptr, "Plan is null");
  RUNTIME_EXCEPTION(new_raw_plan->size() > loop_idx,
                    "Not enough loops in the plan");
  RUNTIME_EXCEPTION(rng != nullptr, "rng collection is null");

  RacePlan::LoopData& loop_segments = (*new_raw_plan)[loop_idx];
  mutation_logger("Mutating block " + std::to_string(loop_idx) + ": ");
  mutation_logger(RacePlan::get_loop_string(loop_segments));
  std::uniform_real_distribution<double> skip_dist(0.0, 1.0);
  std::uniform_real_distribution<double> noise_dist(-1.0, 1.0);

  const size_t num_segments = loop_segments.size();
  for (size_t seg_idx = 0; seg_idx < num_segments; seg_idx++) {
    // Find an acceleration/deceleration segment
    if (loop_segments[seg_idx].acceleration_value == 0.0) {
      continue;
    }
    const size_t seg_start = loop_segments[seg_idx].start_idx;
    size_t seg_end = loop_segments[seg_idx].end_idx;

    // Pick a corner speed. If the current speed is already >= the target
    // average speed of the course, then bias towards maintaining this constant
    // speed rather than trying to accelerate
    const double probability = 0.2;
    const double chance = skip_dist(rng->skip_rng);
    if (chance < probability) {
      mutation_logger("Sampled chance - not mutating this corner");
      continue;
    }
    // Sample a random noise to add to the speed
    const int speed_increase = noise_dist(rng->speed_rng);
    RacePlan::SegmentData orig_seg = (*new_raw_plan)[loop_idx][seg_idx];
    (*new_raw_plan)[loop_idx][seg_idx].end_speed += speed_increase;
    (*new_raw_plan)[loop_idx][seg_idx].acceleration_value =
        calc_acceleration((*new_raw_plan)[loop_idx][seg_idx].start_speed,
                          (*new_raw_plan)[loop_idx][seg_idx].end_speed,
                          (*new_raw_plan)[loop_idx][seg_idx].distance);
    mutation_logger(
        "Trying the following at segment index " + std::to_string(seg_idx) +
        "\n" +
        RacePlan::get_segment_string((*new_raw_plan)[loop_idx][seg_idx]));
    if (!legalize_loop(new_raw_plan, loop_idx, seg_idx, &mutation_logger)) {
      mutation_logger("Loop legalization failed - restoring added segments\n");
      (*new_raw_plan)[loop_idx][seg_idx] = orig_seg;
    }
  }
  mutation_logger("\nNew loop block " + std::to_string(loop_idx));
  mutation_logger(RacePlan::get_loop_string(loop_segments));
}

void V2Optimizer::acceleration_noise(RacePlan* plan,
                                     RacePlanCreator::Gen* rng) {
  RUNTIME_EXCEPTION(rng != nullptr, "rng struct for gaussian noise is null");
  RUNTIME_EXCEPTION(plan != nullptr,
                    "Plan in acceleration noise mutation is null");

  // Get raw plan
  RacePlan::PlanData new_raw_plan = plan->get_orig_segments();

  RacePlanCreator::PlanAttributes att;
  std::unordered_set<size_t> corner_end_indices =
      route->get_corner_end_indices();
  std::unordered_map<size_t, size_t> corner_end_map =
      route->get_corner_end_map();
  std::uniform_real_distribution<double> skip_dist(0.0, 1.0);
  std::uniform_int_distribution<int> noise_dist(-1, 1);
  const size_t total_num_blocks = static_cast<int>(std::ceil(
      plan->get_num_loops() / Config::get_instance().get_num_repetitions()));

  // if loop_idx = -1, mutating all blocks. Otherwise, mutating just one block
  RUNTIME_EXCEPTION(total_num_blocks == new_raw_plan.size(),
                    "raw_segments vector shoud "
                    "have number of loops equal to the number of blocks {}",
                    total_num_blocks);
  for (size_t block_idx = 0; block_idx < total_num_blocks; block_idx++) {
    acceleration_noise_loop(&new_raw_plan, block_idx, rng);
    att.raw_segments.push_back(new_raw_plan[block_idx]);
    generator->create_loop_block(&new_raw_plan[block_idx],
                                 this->num_loops_in_block,
                                 block_idx == total_num_blocks - 1,
                                 block_idx == 0, &att, &mutation_logger);
    mutation_logger("Finished creating loop block");
  }

  // Create new loop plan
  mutation_logger("Finished gaussian noise mutation");
  *plan =
      RacePlan(att.all_segments, att.raw_segments, this->num_loops_in_block);
}

void V2Optimizer::constant_for_deceleration(RacePlan* plan,
                                            RacePlanCreator::Gen* rng) {
  RUNTIME_EXCEPTION(rng != nullptr,
                    "rng struct for constant_for_deceleration is null");
  RUNTIME_EXCEPTION(plan != nullptr,
                    "Plan in ConstantForDeceleration mutation is null");

  // Get raw plan data
  RacePlan::PlanData new_raw_plan = plan->get_orig_segments();
  RacePlanCreator::PlanAttributes att;

  const size_t total_num_blocks = static_cast<int>(std::ceil(
      plan->get_num_loops() / Config::get_instance().get_num_repetitions()));
  RUNTIME_EXCEPTION(total_num_blocks == new_raw_plan.size(),
                    "raw_segments vector shoud "
                    "have number of loops equal to the number of blocks {}",
                    total_num_blocks);
  for (size_t block_idx = 0; block_idx < total_num_blocks; block_idx++) {
    constant_for_deceleration_loop(&new_raw_plan, block_idx, rng);
    att.raw_segments.push_back(new_raw_plan[block_idx]);
    generator->create_loop_block(&new_raw_plan[block_idx],
                                 this->num_loops_in_block,
                                 block_idx == total_num_blocks - 1,
                                 block_idx == 0, &att, &mutation_logger);
  }

  // Create new loop plan
  *plan =
      RacePlan(att.all_segments, att.raw_segments, this->num_loops_in_block);
}

void V2Optimizer::constant_for_deceleration_loop(
    RacePlan::PlanData* new_raw_plan, size_t loop_idx,
    RacePlanCreator::Gen* rng) {
  RUNTIME_EXCEPTION(new_raw_plan != nullptr, "Plan is null");
  RUNTIME_EXCEPTION(new_raw_plan->size() > loop_idx,
                    "Not enough loops in the plan");
  RUNTIME_EXCEPTION(rng != nullptr, "rng collection is null");

  const std::unordered_set<size_t>& corner_end_indices =
      route->get_corner_end_indices();
  const std::unordered_map<size_t, size_t>& corner_end_map =
      route->get_corner_end_map();
  const std::vector<double>& cornering_speed_bounds =
      route->get_cornering_speed_bounds();
  std::uniform_real_distribution<double> skip_dist(0.0, 1.0);

  RacePlan::LoopData& loop_segments = (*new_raw_plan)[loop_idx];
  mutation_logger("Mutating block " + std::to_string(loop_idx) + ": ");
  mutation_logger(RacePlan::get_loop_string(loop_segments));
  const size_t num_segments = loop_segments.size();
  for (size_t seg_idx = 0; seg_idx < num_segments; seg_idx++) {
    // Do nothing about segments that don't decelerate
    if (loop_segments[seg_idx].acceleration_value >= 0.0) {
      continue;
    }
    const size_t seg_start = loop_segments[seg_idx].start_idx;
    size_t seg_end = loop_segments[seg_idx].end_idx;

    // We only bias towards constant speeds for "normal straights".
    // This means that the segment directly after this deceleration segment
    // should include the corner that was being decelerated towards
    if (seg_idx == num_segments - 1) {
      continue;
    }
    const size_t theoretical_corner_end = loop_segments[seg_idx + 1].end_idx;
    if (corner_end_indices.find(theoretical_corner_end) ==
        corner_end_indices.end()) {
      // This is likely an aggressive straight. Don't do anything about these
      continue;
    }
    const size_t corner_idx = corner_end_map.at(theoretical_corner_end);
    mutation_logger("Replacing straight leading to corner index " +
                    std::to_string(corner_idx) +
                    ", Segment Index: " + std::to_string(seg_idx));
    const size_t corner_max_speed = cornering_speed_bounds[corner_idx];

    // Pick a corner speed. If the current speed is already >= the target
    // average speed of the course, then bias towards maintaining this constant
    // speed rather than trying to accelerate
    const double probability = 0.2;
    const double chance = skip_dist(rng->skip_rng);
    if (chance < probability) {
      mutation_logger("Sampled chance - not mutating this corner");
      continue;
    }
    // Check if the starting speed of the deceleration is less than the maximum
    // corner speed. If it is, then we can remove the deceleration and maintain
    // the constant speed
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

      // Replace the deceleration segment and corner segment with the new
      // segment
      mutation_logger("Inserting the following segment at index " +
                      std::to_string(seg_idx) + "\n" +
                      RacePlan::get_segment_string(straight_seg_data));

      (*new_raw_plan)[loop_idx][seg_idx] = straight_seg_data;
      mutation_logger("Inserting the following segment at index " +
                      std::to_string(seg_idx + 1) + "\n" +
                      RacePlan::get_segment_string(corner_seg_data));
      (*new_raw_plan)[loop_idx][seg_idx + 1] = corner_seg_data;

      if (!legalize_loop(new_raw_plan, loop_idx, seg_idx + 1,
                         &mutation_logger)) {
        mutation_logger(
            "Loop legalization failed - restoring added segments\n");
        (*new_raw_plan)[loop_idx][seg_idx] = orig_straight_seg_data;
        (*new_raw_plan)[loop_idx][seg_idx + 1] = orig_corner_seg_data;
      }
    } else {
      mutation_logger(
          "Corner speed is greater than the desired new cruising speed. "
          "Skipping");
    }
  }
  mutation_logger("\nNew loop block " + std::to_string(loop_idx));
  mutation_logger(RacePlan::get_loop_string(loop_segments));
}

bool V2Optimizer::legalize_loop(RacePlan::PlanData* plan, size_t loop_idx,
                                size_t seg_idx, FileLogger* logger) {
  RUNTIME_EXCEPTION(logger != nullptr, "Logger is null");
  RUNTIME_EXCEPTION(loop_idx < plan->size(),
                    "Loop index exceeds no. loops in plan");
  RUNTIME_EXCEPTION(seg_idx < (*plan)[loop_idx].size(),
                    "Segment index exceeds no. segments in plan");

  const RacePlan::PlanData saved_plan = *plan;

  (*logger)("Legalizing loop starting with index " + std::to_string(seg_idx));
  const size_t num_loops = plan->size();

  /** @brief Check that seg does not violate the cornering indices or speeds in
   * corners */
  auto check_cornering_constraint =
      [&](const RacePlan::SegmentData& seg,
          const std::vector<size_t>& corners) -> bool {
    for (const auto& c : seg.corners) {
      const size_t corner_start =
          this->route->get_cornering_segment_bounds()[c].first;
      if (seg.start_idx > corner_start) {
        return false;
      }
      const size_t max_corner_speed =
          this->route->get_cornering_speed_bounds()[c];
      if (seg.start_speed > max_corner_speed) {
        return false;
      }
      if (seg.acceleration_value > 0) {
        return false;
      }
    }
    return true;
  };

  /** @brief Check that the segment does not violate acceleration/deceleration
   * constraints */
  auto check_acceleration_constraint =
      [&](const RacePlan::SegmentData& seg) -> bool {
    if (seg.acceleration_value > 0.0) {
      const double drawn_power =
          seg.acceleration_value * seg.end_speed * this->car_mass;
      if (drawn_power > this->acceleration_power_budget ||
          seg.acceleration_value > this->max_acceleration) {
        (*logger)("Acceleration failed on the legalization start segment");
        return false;
      }
    } else if (seg.acceleration_value < 0.0 &&
               seg.acceleration_value < this->max_deceleration) {
      (*logger)("Deceleration failed on the legalization start segment");
      return false;
    }
    return true;
  };

  if (!check_acceleration_constraint(saved_plan[loop_idx][seg_idx])) {
    (*logger)(
        "Acceleration constraints failed on the legalization start segment");
    return false;
  }
  if (!check_cornering_constraint(saved_plan[loop_idx][seg_idx],
                                  saved_plan[loop_idx][seg_idx].corners)) {
    (*logger)("Cornering constraints failed on the legalization start segment");
    return false;
  }

  /** @brief Resolve either an index or speed discontinuity between segment i
   * and i+1
   * @param loop The loop to modify
   * @param i "True" segment, i+1 will be modified
   * @param starting_speed Optional parameter for the ending speed of segment i.
   * Used when evaluating the first segment of the next loop
   * @return True if discontinuity resolution was successful, False otherwise
   */
  auto resolve_next_segment = [&](RacePlan::LoopData& loop, size_t i) -> bool {
    // Discontinuity, replace i+1 segment
    RacePlan::SegmentData new_segment = loop[i + 1];
    if (loop[i].end_idx != loop[i + 1].start_idx) {
      // Index discontinuity
      const double new_distance =
          this->route_distances.get_value(loop[i].end_idx, loop[i + 1].end_idx);

      new_segment.start_idx = loop[i].end_idx;
      new_segment.distance = new_distance;
      // In the case of acceleration, attempt to modify the acceleration
      const double required_acceleration = calc_acceleration(
          loop[i + 1].start_speed, loop[i + 1].end_speed, new_distance);
      new_segment.corners = this->route->get_overlapping_corners(
          {new_segment.start_idx, new_segment.end_idx});
      new_segment.acceleration_value = required_acceleration;
      if (!check_cornering_constraint(new_segment, new_segment.corners)) {
        return false;
      }
    } else {
      // Speed discontinuity
      if (new_segment.corners.size() > 0 &&
          loop[i + 1].acceleration_value == 0.0) {
        // Corner segment
        new_segment.start_speed = new_segment.end_speed = loop[i].end_speed;
        if (!check_cornering_constraint(new_segment, new_segment.corners)) {
          return false;
        }
      } else {
        // Can accelerate
        const double required_acceleration = calc_acceleration(
            loop[i].end_speed, loop[i + 1].end_speed, new_segment.distance);
        new_segment.acceleration_value = required_acceleration;
        // Check cornering constraint
        new_segment.start_speed = loop[i].end_speed;
      }
    }

    if (!check_acceleration_constraint(new_segment)) {
      return false;
    }

    (*plan)[loop_idx][i + 1] = new_segment;
    return true;
  };

  // The following are necessary for resolving segments that connect the current
  // segments to the next loop
  bool resolve_connection_segments = false;
  double carry_over_speed = -1;
  RacePlan::LoopData& loop = (*plan)[loop_idx];
  const size_t num_segments = (*plan)[loop_idx].size();

  // Find the route index of the last corner whose maximum cornering speed is
  // less than the maximum route speed. This is the corner from which connection
  // segments to the next loop begin and where wrap around segments begin
  int last_real_corner_end_idx;
  const auto corner_end = this->route->get_corner_end_indices();
  bool crossed_over = false;
  for (size_t i = loop.size() - 1; i >= 0; i--) {
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

  // FInd the route index of the first corner whose maximum cornering speed is
  // less than the maximum route speed. This is the corner to which wrap-around
  // segments are applied
  int first_real_corner_end_idx = 0;
  crossed_over = false;
  for (size_t i = 0; i < loop.size(); i++) {
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

  // Do not attempt to modify the speed of the first segment
  if ((*plan)[loop_idx][seg_idx].end_idx == first_real_corner_end_idx) {
    (*logger)("Attempted to modify first real corner");
    return false;
  }

  for (size_t i = seg_idx; i < num_segments - 1; i++) {
    // Check continuity of indices and speeds. If continuous from current
    // segment to the next, return success
    bool continuous_with_next_segment =
        loop[i].end_idx == loop[i + 1].start_idx &&
        loop[i].end_speed == loop[i + 1].start_speed;
    if (loop[i + 1].end_idx == first_real_corner_end_idx) {
      *plan = saved_plan;
      (*logger)("Attempted to modify first real corner");
      return false;
    }

    if (i == num_segments - 2 && !continuous_with_next_segment &&
        plan->size() - 1 > loop_idx) {
      (*logger)("Could not resolve wrap-around segment - giving up");
      *plan = saved_plan;
      return false;
    }
    if (loop[i].end_idx == last_real_corner_end_idx &&
        plan->size() - 1 > loop_idx && !continuous_with_next_segment) {
      // For the last corner of the loop, need to check continuity with the
      // connecting segment of the next loop and with the wrap-around segments
      // ONLY until the first corner
      resolve_connection_segments = true;
      carry_over_speed = loop[i].end_speed;
    }
    if (continuous_with_next_segment) {
      break;
    }
    // Discontinuity, replace i+1 segment
    if (!resolve_next_segment(loop, i)) {
      *plan = saved_plan;
      return false;
    }
  }

  if (resolve_connection_segments) {
    // Resolve segments that connect the current loop to the next
    RUNTIME_EXCEPTION(loop_idx + 1 < plan->size() && carry_over_speed != -1,
                      "Invalid parameters to resolve next loop");
    RacePlan::LoopData& next_loop = (*plan)[loop_idx + 1];

    // Evaluate if the starting speed of the next loop can be modified
    next_loop[0].start_speed = carry_over_speed;
    next_loop[0].acceleration_value =
        calc_acceleration(next_loop[0].start_speed, next_loop[0].end_speed,
                          next_loop[0].distance);
    // Check acceleration constraints
    if (!check_acceleration_constraint(next_loop[0])) {
      (*logger)("Violated acceleration constraint in wrap-around");
      *plan = saved_plan;
      return false;
    }
  }

  (*logger)("Legalization successful");
  return true;
}
