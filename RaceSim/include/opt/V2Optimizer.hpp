/** Optimizer that searches for the best acceleration profile out of a random set of trials */
#pragma once

#include <memory>
#include <vector>

#include "opt/Optimizer.hpp"
#include "route/Route.hpp"
#include "sim/Simulator.hpp"

class V2Optimizer : public Optimizer {
 private:
  // Race plans and their corresponding results
  std::vector<std::shared_ptr<ResultsLut>> result_luts;
  std::vector<RacePlan> race_plans;

  // Overhead tracking
  std::vector<double> race_plan_creation;

  const unsigned int num_threads;

 public:
  V2Optimizer(std::shared_ptr<Simulator> simulator, std::shared_ptr<Route> route);
  RacePlan optimize() override;
};
