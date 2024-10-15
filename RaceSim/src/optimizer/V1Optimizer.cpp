#include <pthread.h>

#include <string>
#include <memory>
#include <utility>
#include <vector>
#include <filesystem>

#include "spdlog/spdlog.h"
#include "opt/V1Optimizer.hpp"
#include "config/Config.hpp"
#include "utils/Defines.hpp"

std::vector<double> V1Optimizer::optimize() {
  /* Loop from speeds 1 -> max. speed to get the maximum viable speed */
  std::vector<double> speed_profile_kph(1);
  bool last_speed_viability = false;
  std::vector<double> last_speed_profile_kph = speed_profile_kph;
  const int max_speed = Config::get_instance()->get_max_speed();
  const int min_speed = Config::get_instance()->get_min_speed();

  bool save_csv = Config::get_instance()->get_save_csv();
  std::filesystem::path results_folder;
  if (save_csv) {
    const char* strat_root = std::getenv("STRAT_ROOT");
    RUNTIME_EXCEPTION(strat_root != nullptr, "No STRAT_ROOT environment variable detected."
                                             "Set it to the full path to gen12_strategy/RaceSim. Exiting.");
    results_folder = std::filesystem::path(strat_root) / "Results";
    std::filesystem::create_directory(results_folder);
  }

  for (int i=min_speed; i <= max_speed; i++) {
    speed_profile_kph[0] = static_cast<double>(i);

    /* Run the simulation */
    bool current_speed_viability = simulator->run_sim(route, speed_profile_kph);
    /* Log the simulation */
    if (save_csv) {
      const int speed = static_cast<int>(speed_profile_kph[0]);
      simulator->write_result((results_folder / (std::to_string(speed) + ".csv")).string());
    }

    if (current_speed_viability) {
      spdlog::info(std::to_string(i) + " kph is viable.");
    } else {
      spdlog::info(std::to_string(i) + " kph is not viable.");
    }

    if (last_speed_viability && !current_speed_viability) {
      return last_speed_profile_kph;
    }
    last_speed_viability = current_speed_viability;
    last_speed_profile_kph = speed_profile_kph;
  }

  if (!last_speed_viability) {
    speed_profile_kph[0] = 0;
  }

  return speed_profile_kph;
}

V1Optimizer::V1Optimizer(std::unique_ptr<Simulator> simulator, std::unique_ptr<Route> route)
    : Optimizer(std::move(simulator), std::move(route)) {}
