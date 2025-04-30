#include "opt/GeneticUtilities.hpp"
#include "utils/Defines.hpp"
#include "config/Config.hpp"

RacePlan mutate_plan(RacePlan plan) {
  RUNTIME_EXCEPTION(!plan.is_empty(), "RacePlan object to mutate cannot be empty");
  std::vector<std::vector<std::pair<size_t, size_t>>> raw_segments = plan.get_orig_loop_segments();
  std::vector<std::vector<std::pair<double, double>>> speeds = plan.get_orig_loop_speeds();
  std::vector<std::vector<bool>> acceleration = plan.get_orig_loop_accelerations();
  std::vector<std::vector<double>> acceleration_value = plan.get_orig_loop_acceleration_values();
  std::vector<std::vector<double>> segment_distances = plan.get_orig_loop_segment_distances();

  if (Config::get_instance()->get_mutation_strategy() == "PreferConstantSpeed") {
    const size_t num_blocks = static_cast<int>(std::ceil(plan.get_num_loops() /
                                Config::get_instance()->get_num_repetitions()));
    size_t loop_idx = 0;
    size_t block_idx = 0;

    
  }

  return RacePlan();
}