#include "spdlog/spdlog.h"
#include "OptimizerFactory.hpp"
#include "V1Optimizer.hpp"
#include "Optimizer.hpp"
#include "Defines.hpp"

std::unordered_map<std::string, Algos> OptimizerFactory::config_to_optimizer = {
    {"Constant", Algos::CONSTANT},
};
const std::string OptimizerFactory::DEFAULT_OPTIMIZER = "Constant";

std::unique_ptr<Optimizer> OptimizerFactory::get_optimizer(std::string opt_type,
                                                           std::unique_ptr<Route> route, 
                                                           std::unique_ptr<Simulator> simulator) {
    Algos opt;
    if (config_to_optimizer.find(opt_type) != config_to_optimizer.end()) {
        opt = config_to_optimizer[opt_type];
    } else {
        opt = config_to_optimizer[DEFAULT_OPTIMIZER]; /* Default to constant speed */
    }

    if (opt == Algos::CONSTANT) {
        spdlog::info("Using constant speed optimizer.");
        return std::make_unique<V1Optimizer>(std::move(simulator), std::move(route));
    } else {
        RUNTIME_EXCEPTION(false, "No valid optimizer selected.");
    }

    return nullptr;
}
