#include "Optimizer.hpp"
#include "Simulator.hpp"
#include "Route.hpp"
#include "Defines.hpp"

Optimizer::Optimizer(std::unique_ptr<Simulator> sim, std::unique_ptr<Route> path) {
    RUNTIME_EXCEPTION(sim != nullptr, "Simulator is null");
    RUNTIME_EXCEPTION(path != nullptr, "Route is null");

    simulator = std::move(sim);
    route = std::move(path);
}
