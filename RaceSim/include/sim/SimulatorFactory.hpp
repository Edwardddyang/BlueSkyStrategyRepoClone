/* Generate simulators */

#pragma once

#include <memory>
#include <unordered_map>
#include <string>

#include "sim/Simulator.hpp"
#include "model/Car.hpp"

enum class SIMULATORS {
  WSC = 0,
  FSGP = 1,
};

class SimulatorFactory {
 private:
  static std::unordered_map<std::string, SIMULATORS> config_to_simulators;
  static const char DEFAULT_SIMULATOR[];

 public:
  /** @brief Create a simulator of unique_ptr type
   *
   * @param sim_type: The simulator to use
   */
  static std::shared_ptr<Simulator> get_simulator(const std::string sim_type,
                                                  std::shared_ptr<Car> model);
};
