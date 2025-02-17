/** Basic optimizer that exhaustively searches for a single speed for the entire race */
#pragma once

#include <memory>
#include <vector>
#include <semaphore>

#include "opt/Optimizer.hpp"
#include "route/Route.hpp"
#include "sim/Simulator.hpp"
#include "utils/ThreadManager.hpp"

class V1Optimizer : public Optimizer {
 private:
  const unsigned int num_threads;

  // Race plans and their corresponding results
  std::vector<std::shared_ptr<ResultsLut>> result_luts;
  std::vector<RacePlan> race_plans;

 public:
  V1Optimizer(std::shared_ptr<Simulator> simulator, std::shared_ptr<Route> route);
  RacePlan optimize() override;

  // Get result luts
  inline std::vector<std::shared_ptr<ResultsLut>> get_result_luts() const { return result_luts; }
};
