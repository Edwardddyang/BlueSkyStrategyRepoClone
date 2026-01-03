#include <string>
#include <memory>
#include <semaphore>
#include <utility>
#include <thread>
#include <vector>
#include <filesystem>
#include <algorithm>

#include "spdlog/spdlog.h"
#include "opt/WSCOptimizer.hpp"
#include "route/Route.hpp"
#include "config/ConfigParser.hpp"
#include "SimUtils/Defines.hpp"

WSCRacePlan WSCOptimizer::optimize_impl() {
  /* Loop from speeds 1 -> max. speed to get the maximum viable speed */
  RUNTIME_EXCEPTION(params.max_speed >= params.min_speed, "Maximum speed must be >= Minimum speed");

  // Create results folder
  if (params.save_csv) {
    std::filesystem::create_directory(params.results_dir);
  }

  // Create threads
  const int total_num_threads = params.max_speed - params.min_speed + 1; 
  std::vector<std::thread> threads(total_num_threads);

  result_luts.clear();
  race_plans.clear();

  // Create all result luts and race plans
  for (int i=params.min_speed; i <= params.max_speed; i++) {
    result_luts.emplace_back();
    const BaseSegment segment(
      0, route.get_num_points() - 1, static_cast<double>(i), static_cast<double>(i),
      0.0, route.get_route_length()
    );
    std::vector<BaseSegment> segments = {segment};
    race_plans.emplace_back(segments);
  }

  // Spin up all threads
  for (int i=0; i < total_num_threads; i++) {
    threads.emplace_back(&WSCOptimizer::run_sim_thread_func, this, &result_luts[i], &race_plans[i]);
  }

  for (int i=0; i < total_num_threads; i++) {
    threads[i].join();
  }

  int max_viable_speed = 0;
  WSCRacePlan max_viable_plan;
  for (int i=0; i < total_num_threads; i++) {
    /* Log the simulation */
    if (params.save_csv) {
      result_luts[i].export_dataset((params.results_dir / (std::to_string(i + params.min_speed) + ".csv")).string());
    }

    if (race_plans[i].is_viable()) {
      spdlog::info(std::to_string(i + params.min_speed) + " kph is viable.");
      max_viable_plan = race_plans[i];
    } else {
      spdlog::info(std::to_string(i + params.min_speed) + " kph is not viable.");
    }
  }

  max_viable_plan.print_plan();
  return max_viable_plan;
}

WSCOptimizer::WSCOptimizer(WSCOptimizerParams params, WSCSimulator simulator, WSCRoute route) :
                           Optimizer(simulator, route, params.max_num_threads), params(std::move(params)) {}
