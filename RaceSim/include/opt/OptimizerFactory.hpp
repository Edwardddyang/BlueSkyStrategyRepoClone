/* Generate optimizers */

#pragma once

#include <memory>
#include <unordered_map>
#include <string>

#include "opt/Optimizer.hpp"
#include "route/Route.hpp"
#include "sim/Simulator.hpp"

enum class Algos {
  CONSTANT = 0,
};

class OptimizerFactory {
 private:
  static std::unordered_map<std::string, Algos> config_to_optimizer;
  static const char DEFAULT_OPTIMIZER[];

 public:
  /** @brief Create an optimizer of unique_ptr type
   *
   * @param opt_type: The algorithm to use
   * @param route: Route to optimize
   * @param simulator: Validates proposed speed profiles 
   */
  static std::unique_ptr<Optimizer> get_optimizer(const std::string opt_type,
                                                  std::unique_ptr<Route> route,
                                                  std::unique_ptr<Simulator> simulator);
};
