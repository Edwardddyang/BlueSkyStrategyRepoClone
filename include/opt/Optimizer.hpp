/* 
Base class for optimization algorithms
*/

#pragma once

#include <memory>
#include <vector>
#include <concepts>

#include "model/Car.hpp"
#include "route/Route.hpp"
#include "route/RacePlan.hpp"
#include "sim/Simulator.hpp"

template<typename Derived, RouteType RouteType, RacePlanType RacePlan, typename SimulatorType>
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
  Optimizer(SimulatorType sim, RouteType route) : simulator(std::move(sim)),
            route(std::move(route)), thread_limiter(std::max(1u, static_cast<unsigned int>(
              std::thread::hardware_concurrency() * Config::get_instance().get_threads()
            ))) {}

  /* Apply the optimization algorithm, output a speed profile over
      the segments of the race
  */
  RacePlan optimize() {
    return static_cast<const Derived*>(this)->optimize_impl();
  }
};
