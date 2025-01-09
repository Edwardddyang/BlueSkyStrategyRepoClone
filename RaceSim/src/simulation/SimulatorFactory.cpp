#include <string>
#include <utility>
#include <memory>
#include <unordered_map>

#include "spdlog/spdlog.h"
#include "sim/SimulatorFactory.hpp"
#include "sim/WSCSimulator.hpp"
#include "sim/FSGPSimulator.hpp"

std::unordered_map<std::string, SIMULATORS> SimulatorFactory::config_to_simulators = {
  {"FSGP", SIMULATORS::FSGP},
  {"WSC", SIMULATORS::WSC}
};
const char SimulatorFactory::DEFAULT_SIMULATOR[] = "FSGP";

std::shared_ptr<Simulator> SimulatorFactory::get_simulator(std::string sim_type,
                                                           std::shared_ptr<Car> model) {
  SIMULATORS sim;
  if (config_to_simulators.find(sim_type) != config_to_simulators.end()) {
    sim = config_to_simulators[sim_type];
  } else {
    sim = config_to_simulators[std::string(DEFAULT_SIMULATOR)]; /* Default to FSGP */
  }

  if (sim == SIMULATORS::FSGP) {
    spdlog::info("Using FSGP Simulator.");
    return std::make_shared<FSGPSimulator>(model);
  } else if (sim == SIMULATORS::WSC) {
    spdlog::info("Using WSC Simulator.");
    return std::make_shared<WSCSimulator>(model);
  } else {
    RUNTIME_EXCEPTION(false, "No valid simulator requested. Check the DEFAULT_SIMULATOR variable");
  }

  return nullptr;
}
