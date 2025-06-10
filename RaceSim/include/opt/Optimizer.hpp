/* 
Base class for optimization algorithms
*/

#pragma once

#include <memory>
#include <vector>

#include "model/Car.hpp"
#include "route/Route.hpp"
#include "sim/Simulator.hpp"
#include "utils/ThreadManager.hpp"

// Run a simulator on a race plan using threads
void thread_run_sim(std::shared_ptr<Simulator> sim,
                    std::shared_ptr<Route> route,
                    std::shared_ptr<ResultsLut> result_lut,
                    RacePlan* race_plan,
                    ThreadManager* thread_manager);

class Optimizer {
 protected:
  /* Assess viability of proposed speed profiles */
  std::shared_ptr<Simulator> simulator;

  /* Route to optimize on */
  std::shared_ptr<Route> route;

  /* Output speed profile */
  std::vector<double> speed_profile_kph;
 public:
  Optimizer(std::shared_ptr<Simulator> sim, std::shared_ptr<Route> path);

  /* Apply the optimization algorithm, output a speed profile over
      the segments of the race
  */
  virtual RacePlan optimize() = 0;
  virtual RacePlan optimize_telem(){
    throw std::runtime_error("optimize_telem() not implemented in this optimizer");
  }
};
