#include <string>
#include <utility>
#include <memory>
#include <unordered_map>

#include "spdlog/spdlog.h"
#include "sim/SimulatorFactory.hpp"
#include "model/V2Car.hpp"
#include "sim/FSGPSimulator.hpp"
#include "sim/ASCSimulator.hpp"
#include "sim/TelemetrySimulator.hpp"

std::unordered_map<std::string, SIMULATORS> SimulatorFactory::config_to_simulators = {
  {"FSGP", SIMULATORS::FSGP},
  {"WSC", SIMULATORS::WSC},
  {"ASC", SIMULATORS::ASC},
  {"TELEMETRY", SIMULATORS::TELEMETRY},
};
const char SimulatorFactory::DEFAULT_SIMULATOR[] = "FSGP";

SIMULATORS SimulatorFactory::get_sim_type(const std::string sim_type) {
  if (config_to_simulators.find(sim_type) == config_to_simulators.end()) {
    spdlog::error("Simulator {} is not registered", sim_type); 
  }
  return config_to_simulators[sim_type];
}

std::shared_ptr<Simulator> SimulatorFactory::get_simulator(std::string sim_type,
                                                           std::shared_ptr<Car> model) {
  std::transform(sim_type.begin(), sim_type.end(), sim_type.begin(), ::toupper); // normalize case

  SIMULATORS sim;
  if (config_to_simulators.find(sim_type) != config_to_simulators.end()) {
    sim = config_to_simulators[sim_type];
  } else {
    sim = config_to_simulators[std::string(DEFAULT_SIMULATOR)]; /* Default to FSGP */
  }

  // replace with switch case?
  if (sim == SIMULATORS::FSGP) {
    spdlog::info("Using FSGP Simulator.");
    if (!std::dynamic_pointer_cast<V2Car>(model)) {
      spdlog::error("FSGP Simulator must use V2Car model");
      exit(0);
    }
    return std::make_shared<FSGPSimulator>(model);
  } else if (sim == SIMULATORS::WSC) {
    spdlog::info("Using WSC Simulator.");
    return std::make_shared<WSCSimulator>(model);
  } else if (sim == SIMULATORS::ASC) {
    spdlog::info("Using ASC Simulator.");
    return std::make_shared<ASCSimulator>(model);
  } else if (sim == SIMULATORS::TELEMETRY) {
    spdlog::info("Using Telemetry Simulator.");
    if (!std::dynamic_pointer_cast<V2Car>(model)) {
      spdlog::error("Telemetry Simulator must use V2Car model");
      exit(0);
    }
    return std::make_shared<TelemetrySimulator>(model);
  } else {
    RUNTIME_EXCEPTION(false, "No valid simulator requested. Check the DEFAULT_SIMULATOR variable");
  }

  return nullptr;
}
