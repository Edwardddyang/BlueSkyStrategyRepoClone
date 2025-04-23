#include <pthread.h>

#include <string>
#include <memory>
#include <semaphore>
#include <utility>
#include <thread>
#include <vector>
#include <filesystem>
#include <algorithm>

#include "spdlog/spdlog.h"
#include "opt/V1Optimizer.hpp"
#include "route/Route.hpp"
#include "config/Config.hpp"
#include "utils/Defines.hpp"

RacePlan V1Optimizer::optimize() {
  /* Loop from speeds 1 -> max. speed to get the maximum viable speed */
  const int max_speed = Config::get_instance()->get_max_speed();
  const int min_speed = Config::get_instance()->get_min_speed();
  RUNTIME_EXCEPTION(max_speed >= min_speed, "Maximum speed must be >= Minimum speed");

  // Create results folder
  bool save_csv = Config::get_instance()->get_save_csv();
  std::filesystem::path results_folder;
  if (save_csv) {
    const std::string strat_root = Config::get_instance()->get_strat_root();
    results_folder = std::filesystem::path(strat_root) / "Results";
    std::filesystem::create_directory(results_folder);
  }

  spdlog::info("Using {} threads for simulation", num_threads);

  // Create threads
  const int total_num_threads = max_speed - min_speed + 1;
  ThreadManager thread_manager(num_threads);
  std::vector<std::thread> threads;
  result_luts.clear();
  race_plans.clear();
  for (int i=min_speed; i <= max_speed; i++) {
    result_luts.emplace_back(std::make_shared<ResultsLut>());
    race_plans.emplace_back(RacePlan({{{0, route->get_num_points() - 1}}},
                       {{{static_cast<double>(i), static_cast<double>(i)}}}));
  }

  for (int i=0; i < total_num_threads; i++) {
    threads.emplace_back(thread_run_sim, simulator, route, result_luts[i], &race_plans[i], &thread_manager);
  }
  for (int i=0; i < total_num_threads; i++) {
    threads[i].join();
  }

  int max_viable_speed = 0;
  RacePlan max_viable_plan;
  for (int i=0; i < total_num_threads; i++) {
    /* Log the simulation */
    if (save_csv) {
      result_luts[i]->write_logs((results_folder / (std::to_string(i + min_speed) + ".csv")).string());
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

V1Optimizer::V1Optimizer(std::shared_ptr<Simulator> simulator, std::shared_ptr<Route> route)
    : num_threads(std::max(1u, static_cast<unsigned int>(
                                std::thread::hardware_concurrency() * Config::get_instance()->get_threads()))),
    Optimizer(simulator, route) {}
