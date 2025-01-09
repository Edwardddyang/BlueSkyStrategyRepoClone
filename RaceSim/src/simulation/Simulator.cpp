#include <stdlib.h>
#include <string>

#include "spdlog/spdlog.h"
#include "sim/Simulator.hpp"

void Simulator::write_result(std::string csv_path) {
  results_lut->write_logs(csv_path);
  spdlog::info("Simulation data saved");
}

ResultsLut Simulator::get_results_lut() const {
  if (!results_lut) return ResultsLut();
  return *(results_lut.get());
}

Simulator::Simulator(std::shared_ptr<Car> model) :
  car(model),
  results_lut(std::make_unique<ResultsLut>()),
  wind_speed_lut(std::make_unique<ForecastLut>(Config::get_instance()->get_wind_speed_path())),
  wind_dir_lut(std::make_unique<ForecastLut>(Config::get_instance()->get_wind_direction_path())),
  dni_lut(std::make_unique<ForecastLut>(Config::get_instance()->get_dni_path())),
  dhi_lut(std::make_unique<ForecastLut>(Config::get_instance()->get_dhi_path())) {}
