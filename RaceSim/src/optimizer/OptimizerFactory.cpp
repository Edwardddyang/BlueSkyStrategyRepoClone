#include <string>
#include <utility>
#include <memory>
#include <unordered_map>

#include "spdlog/spdlog.h"
#include "opt/OptimizerFactory.hpp"
#include "opt/V1Optimizer.hpp"
#include "opt/V2Optimizer.hpp"
#include "opt/Optimizer.hpp"
#include "utils/Defines.hpp"

std::unordered_map<std::string, Algos> OptimizerFactory::config_to_optimizer = {
  {"Constant", Algos::CONSTANT},
  {"Acceleration", Algos::ACCELERATION}
};
const char OptimizerFactory::DEFAULT_OPTIMIZER[] = "Constant";

// std::shared_ptr<Optimizer> OptimizerFactory::get_optimizer(std::string opt_type,
//                                                            std::shared_ptr<Route> route,
//                                                            std::shared_ptr<Simulator> simulator) {
//   Algos opt;
//   if (config_to_optimizer.find(opt_type) != config_to_optimizer.end()) {
//     opt = config_to_optimizer[opt_type];
//   } else {
//     opt = config_to_optimizer[std::string(DEFAULT_OPTIMIZER)]; /* Default to constant speed */
//   }
//
//   if (opt == Algos::CONSTANT) {
//     spdlog::info("Using constant speed optimizer.");
//     return std::make_shared<V1Optimizer>(simulator, route);
//   } else if (opt == Algos::ACCELERATION) {
//     spdlog::info("Using accleration optimizer.");
//     return std::make_shared<V2Optimizer>(simulator, route);
//   } else {
//     RUNTIME_EXCEPTION(false, "No valid optimizer requested. Check the DEFAULT_OPTIMIZER variable");
//   }
//
//   return nullptr;
// }
