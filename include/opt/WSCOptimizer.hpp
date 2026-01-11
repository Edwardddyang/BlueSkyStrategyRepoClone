/** Exhaustively searches for a single cruising speed for the entire duration of
 * the race */
#pragma once

#include <algorithm>
#include <filesystem>
#include <thread>
#include <utility>
#include <vector>

#include "SimUtils/Defines.hpp"
#include "SimUtils/Luts.hpp"
#include "config/ConfigParser.hpp"
#include "opt/Optimizer.hpp"
#include "route/RacePlan.hpp"
#include "route/Route.hpp"
#include "sim/WSCSimulator.hpp"

struct WSCOptimizerParams {
  unsigned int max_num_threads;
  int min_speed;                      // Minimum speed to consider in m/s
  int max_speed;                      // Maximum speed to consider in m/s
  bool save_csv;                      // Whether to save the metric log
  std::filesystem::path results_dir;  // Directory to store metric logs

  WSCOptimizerParams(unsigned int max_num_threads, int min_speed, int max_speed,
                     bool save_csv, std::filesystem::path results_dir)
      : max_num_threads(max_num_threads),
        min_speed(min_speed),
        max_speed(max_speed),
        save_csv(save_csv),
        results_dir(std::move(results_dir)) {}
};

inline WSCOptimizerParams get_wsc_optimizer_params(ConfigParser* parser) {
  RUNTIME_EXCEPTION(parser != nullptr,
                    "Parser is null when retrieving WSC Optimizer parameters");
  return WSCOptimizerParams(
      std::max<unsigned int>(
          1U, static_cast<unsigned int>(parser->get_threads() *
                                        std::thread::hardware_concurrency())),
      parser->get_min_speed(), parser->get_max_speed(), parser->get_save_csv(),
      parser->get_results_dir());
}

class WSCOptimizer
    : public Optimizer<WSCOptimizer, WSCRoute, WSCRacePlan, WSCSimulator> {
 private:
  // Race plans and their corresponding results
  std::vector<Luts::DataSet> result_luts;
  std::vector<WSCRacePlan> race_plans;

  const WSCOptimizerParams params;

 public:
  WSCOptimizer(WSCOptimizerParams params, WSCSimulator simulator,
               WSCRoute route);
  WSCRacePlan optimize_impl();  // TODO(Make this private)
  // Get result luts
  const std::vector<Luts::DataSet>& get_result_luts() const {
    return result_luts;
  }
};
