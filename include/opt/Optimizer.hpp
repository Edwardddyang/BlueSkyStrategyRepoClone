/*
Base class for optimization algorithms
*/

#pragma once

#include <semaphore>

#include "SimUtils/Luts.hpp"
#include "route/RacePlan.hpp"
#include "route/Route.hpp"

template <typename Derived, RouteType RouteType, RacePlanType RacePlan,
          typename SimulatorType>
class Optimizer {
 protected:
  // Semaphore with 1024 maximum count
  std::counting_semaphore<1024> thread_limiter;

  /* Assess viability of proposed speed profiles */
  SimulatorType simulator;

  /* Route to optimize on */
  RouteType route;

  // Thread function
  void run_sim_thread_func(Luts::DataSet* result_lut, WSCRacePlan* race_plan) {
    thread_limiter.acquire();
    simulator.run_sim(route, race_plan, result_lut);
    thread_limiter.release();
  }

 public:
  Optimizer(SimulatorType sim, RouteType route, unsigned int max_num_threads)
      : thread_limiter(max_num_threads),
        simulator(std::move(sim)),
        route(std::move(route)) {}

  /* Apply the optimization algorithm, output a speed profile over
      the segments of the race
  */
  RacePlan optimize() { return static_cast<Derived*>(this)->optimize_impl(); }
};
