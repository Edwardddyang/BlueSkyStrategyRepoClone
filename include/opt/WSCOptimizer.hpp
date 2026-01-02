/** Exhaustively searches for a single cruising speed for the entire duration of the race */
#pragma once

#include <memory>
#include <vector>
#include <semaphore>

#include "opt/Optimizer.hpp"
#include "route/Route.hpp"
#include "route/RacePlan.hpp"
#include "sim/WSCSimulator.hpp"

class WSCOptimizer : public Optimizer<WSCOptimizer, WSCRoute, WSCRacePlan, WSCSimulator> {
 private:
  // Race plans and their corresponding results
  std::vector<Luts::DataSet> result_luts;
  std::vector<WSCRacePlan> race_plans;

 public:
  WSCOptimizer(WSCSimulator simulator, WSCRoute route);
  WSCRacePlan optimize_impl();

  // Get result luts
  inline const std::vector<Luts::DataSet>& get_result_luts() const { return result_luts; }
};
