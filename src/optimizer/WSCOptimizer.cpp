#include "opt/WSCOptimizer.hpp"

#include <filesystem>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "SimUtils/Constants.hpp"
#include "SimUtils/Defines.hpp"
#include "opt/Optimizer.hpp"
#include "route/RacePlan.hpp"
#include "route/Route.hpp"
#include "sim/WSCSimulator.hpp"
#include "spdlog/spdlog.h"

WSCRacePlan WSCOptimizer::optimize_impl() {
  /* Loop from speeds 1 -> max. speed to get the maximum viable speed */
  RUNTIME_EXCEPTION(params.max_speed >= params.min_speed,
                    "Maximum speed must be >= Minimum speed");

  // Create results folder
  if (params.save_csv) {
    std::filesystem::create_directory(params.results_dir);
  }

  // Create threads
  const int total_num_threads = params.max_speed - params.min_speed + 1;
  std::vector<std::thread> threads;
  threads.reserve(total_num_threads);

  result_luts.clear();
  race_plans.clear();

  // Create all result luts and race plans
  for (int i = params.min_speed; i <= params.max_speed; i++) {
    result_luts.emplace_back();
    const double speed = util::constants::kph2mps(i);
    const BaseSegment segment(0, route.get_num_points() - 1, speed, speed, 0.0,
                              route.get_route_length());
    const std::vector<BaseSegment> segments = {segment};
    race_plans.emplace_back(segments);
  }

  // Spin up all threads
  for (int i = 0; i < total_num_threads; i++) {
    threads.emplace_back(&WSCOptimizer::run_sim_thread_func, this,
                         &result_luts[i], &race_plans[i]);
  }

  for (int i = 0; i < total_num_threads; i++) {
    threads[i].join();
  }

  WSCRacePlan max_viable_plan;
  for (int i = 0; i < total_num_threads; i++) {
    /* Log the simulation */
    if (params.save_csv) {
      result_luts[i].export_dataset(
          (params.results_dir / (std::to_string(i + params.min_speed) + ".csv"))
              .string());
    }

    if (race_plans[i].is_viable()) {
      spdlog::info(std::to_string(i + params.min_speed) + " kph is viable.");
      max_viable_plan = race_plans[i];
    } else {
      spdlog::info(std::to_string(i + params.min_speed) +
                   " kph is not viable. Reason: " +
                   race_plans[i].get_inviability_reason());
    }
  }

  return max_viable_plan;
}

WSCOptimizer::WSCOptimizer(WSCOptimizerParams params, WSCSimulator simulator,
                           WSCRoute route)
    : Optimizer(std::move(simulator), std::move(route), params.max_num_threads),
      params(std::move(params)) {}
