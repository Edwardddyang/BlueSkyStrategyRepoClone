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
  /* Assess viability of proposed speed profiles */
  SimulatorType simulator;

  /* Route to optimize on */
  RouteType route;

 public:
  Optimizer(SimulatorType sim, RouteType route) : simulator(std::move(sim)),
            route(std::move(route)) {}

  /* Apply the optimization algorithm, output a speed profile over
      the segments of the race
  */
  RacePlan optimize() const {
    return static_cast<const Derived*>(this)->optimize_impl();
  }
};
