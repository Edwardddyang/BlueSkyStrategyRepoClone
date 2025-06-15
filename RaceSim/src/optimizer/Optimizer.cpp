#include <memory>
#include <utility>

#include "opt/Optimizer.hpp"
#include "utils/Defines.hpp"

// Thread function for running a simulation
void thread_run_sim(std::shared_ptr<Simulator> sim,
                    std::shared_ptr<Route> route,
                    std::shared_ptr<ResultsLut> result_lut,
                    RacePlan* race_plan,
                    ThreadManager* thread_manager) {
  thread_manager->acquire();
  sim->run_sim(route, race_plan, result_lut);
  thread_manager->release();
}

Optimizer::Optimizer(std::shared_ptr<Simulator> sim, std::shared_ptr<Route> path) {
  RUNTIME_EXCEPTION(sim != nullptr, "Simulator is null");
  RUNTIME_EXCEPTION(path != nullptr, "Route is null");

  simulator = sim;
  route = path;
}
