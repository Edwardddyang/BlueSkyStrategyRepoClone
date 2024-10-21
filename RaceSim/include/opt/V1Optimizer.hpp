/** Basic optimizer that exhaustively searches for a single speed for the entire race */
#pragma once

#include <memory>
#include <vector>

#include "opt/Optimizer.hpp"
#include "route/Route.hpp"
#include "sim/Simulator.hpp"

class V1Optimizer : public Optimizer {
 public:
  V1Optimizer(std::unique_ptr<Simulator> simulator, std::unique_ptr<Route> route);
  RacePlan optimize() override;
};
