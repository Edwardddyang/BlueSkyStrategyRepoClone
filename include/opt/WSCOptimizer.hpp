/** Exhaustively searches for a single cruising speed for the entire duration of the race */
#pragma once

#include <memory>
#include <vector>
#include <semaphore>

#include "opt/Optimizer.hpp"
#include "route/Route.hpp"
#include "sim/Simulator.hpp"

class WSCOptimizer : public Optimizer<WSCOptimizer, WSCRoute, WSCRacePlan, WSCSimulator> {
 private:
  // Semaphore with 1024 maximum count
  std::counting_semaphore<1024> thread_limiter;

  // Race plans and their corresponding results
  std::vector<Luts::DataSet> result_luts;
  std::vector<WSCRacePlan> race_plans;

  // Thread function
  void run_sim_thread_func(Luts::DataSet* result_lut, WSCRacePlan* race_plan);

 public:
  WSCOptimizer(WSCSimulator simulator, WSCRoute route);
  WSCRacePlan optimize_impl() const;

  // Get result luts
  inline const std::vector<Luts::DataSet>& get_result_luts() const { return result_luts; }
};
