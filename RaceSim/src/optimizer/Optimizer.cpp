#include <memory>
#include <utility>

#include "opt/Optimizer.hpp"
#include "utils/Defines.hpp"

Optimizer::Optimizer(std::shared_ptr<Simulator> sim, std::shared_ptr<Route> path) {
  RUNTIME_EXCEPTION(sim != nullptr, "Simulator is null");
  RUNTIME_EXCEPTION(path != nullptr, "Route is null");

  simulator = sim;
  route = path;
}
