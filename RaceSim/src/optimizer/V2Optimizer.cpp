#include <pthread.h>

#include <string>
#include <memory>
#include <utility>
#include <vector>
#include <filesystem>

#include "spdlog/spdlog.h"
#include "opt/V2Optimizer.hpp"
#include "route/Route.hpp"
#include "config/Config.hpp"
#include "utils/Defines.hpp"

RacePlan V2Optimizer::optimize() {
  /* Initialize segment bounds */
  /* Loop from speeds 1 -> max. speed to get the maximum viable speed */
  const int max_speed = Config::get_instance()->get_max_speed();
  const int min_speed = Config::get_instance()->get_min_speed();

  std::vector<std::pair<size_t, size_t>> segments = {{{0, route->get_num_points() - 1}}};
  RacePlan current_race_plan({{{0, route->get_num_points() - 1}}},
                            {{{static_cast<double>(min_speed), static_cast<double>(min_speed)}}});
  RacePlan last_race_plan(current_race_plan);

  bool save_csv = Config::get_instance()->get_save_csv();
  std::filesystem::path results_folder;
  if (save_csv) {
    const std::string strat_root = Config::get_instance()->get_strat_root();
    results_folder = std::filesystem::path(strat_root) / "Results";
    std::filesystem::create_directory(results_folder);
  }

  for (int i=min_speed; i <= max_speed; i++) {
    current_race_plan.set_speed_profile({{{static_cast<double>(i), static_cast<double>(i)}}});

    /* Run the simulation */
    simulator->run_sim(route, &current_race_plan);
    /* Log the simulation */
    if (save_csv) {
      simulator->write_result((results_folder / (std::to_string(i) + ".csv")).string());
    }

    if (current_race_plan.is_viable()) {
      spdlog::info(std::to_string(i) + " kph is viable.");
    } else {
      spdlog::info(std::to_string(i) + " kph is not viable.");
    }

    if (last_race_plan.is_viable() && !current_race_plan.is_viable()) {
      return last_race_plan;
    }
    last_race_plan = current_race_plan;
  }

  if (!current_race_plan.is_viable()) {
    current_race_plan.set_speed_profile({{{0.0, 0.0}}});
  }

  return current_race_plan;
}

V2Optimizer::V2Optimizer(std::shared_ptr<Simulator> simulator, std::shared_ptr<Route> route)
    : Optimizer(simulator, route) {}
