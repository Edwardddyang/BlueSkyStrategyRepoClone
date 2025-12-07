/* Generate optimizers */

#pragma once

#include <memory>
#include <unordered_map>
#include <string>

#include "opt/Optimizer.hpp"
#include "route/Route.hpp"
#include "sim/Simulator.hpp"
#include "opt/V1Optimizer.hpp"
#include "opt/V2Optimizer.hpp"
#include "utils/Defines.hpp"

enum class Algos {
  CONSTANT = 0,
  ACCELERATION = 1,
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

	template<class RouteType>
  static std::shared_ptr<Optimizer<RouteType>> get_optimizer(const std::string opt_type,
												  std::shared_ptr<RouteType> route,
												  std::shared_ptr<Simulator> simulator) {
		Algos opt;
		if (config_to_optimizer.find(opt_type) != config_to_optimizer.end()) {
			opt = config_to_optimizer[opt_type];
		} else {
			opt = config_to_optimizer[std::string(DEFAULT_OPTIMIZER)]; /* Default to constant speed */
		}

		if (opt == Algos::CONSTANT) {
			spdlog::info("Using constant speed optimizer.");
			return std::make_shared<V1Optimizer>(simulator, route);
		} else if (opt == Algos::ACCELERATION) {
			spdlog::info("Using accleration optimizer.");
			return std::make_shared<V2Optimizer>(simulator, route);
		} else {
			RUNTIME_EXCEPTION(false, "No valid optimizer requested. Check the DEFAULT_OPTIMIZER variable");
		}

		return nullptr;
	};
};
