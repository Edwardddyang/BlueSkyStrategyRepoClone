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
#include "config/Config.hpp"
#include "SimUtils/Defines.hpp"

WSCRacePlan WSCOptimizer::optimize_impl() {
  /* Loop from speeds 1 -> max. speed to get the maximum viable speed */
  const int max_speed = static_cast<int>(Config::get_instance().get_max_speed());
  const int min_speed = static_cast<int>(Config::get_instance().get_min_speed());
  RUNTIME_EXCEPTION(max_speed >= min_speed, "Maximum speed must be >= Minimum speed");

  // Create results folder
  bool save_csv = Config::get_instance().get_save_csv();
  std::filesystem::path results_folder;
  if (save_csv) {
    const std::string strat_root = Config::get_instance().get_strat_root();
    results_folder = std::filesystem::path(strat_root) / "Results";
    std::filesystem::create_directory(results_folder);
  }

  // Create threads
  const int total_num_threads = max_speed - min_speed + 1; 
  std::vector<std::thread> threads(total_num_threads);

  result_luts.clear();
  race_plans.clear();

  // Create all result luts and race plans
  for (int i=min_speed; i <= max_speed; i++) {
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
    if (save_csv) {
      result_luts[i].export_dataset((results_folder / (std::to_string(i + min_speed) + ".csv")).string());
    }

    if (race_plans[i].is_viable()) {
      spdlog::info(std::to_string(i + min_speed) + " kph is viable.");
      max_viable_plan = race_plans[i];
    } else {
      spdlog::info(std::to_string(i + min_speed) + " kph is not viable.");
    }
  }

  max_viable_plan.print_plan();
  return max_viable_plan;
}

WSCOptimizer::WSCOptimizer(WSCSimulator simulator, WSCRoute route) :
                           Optimizer(simulator, route) {}
