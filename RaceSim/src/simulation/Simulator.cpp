#include <stdlib.h>
#include <string>

#include "spdlog/spdlog.h"
#include "sim/Simulator.hpp"

Simulator::Simulator(std::shared_ptr<Car> model) :
  car(model),
  wind_speed_lut(std::make_unique<ForecastLut>(Config::get_instance()->get_wind_speed_path())),
  wind_dir_lut(std::make_unique<ForecastLut>(Config::get_instance()->get_wind_direction_path())),
  dni_lut(std::make_unique<ForecastLut>(Config::get_instance()->get_dni_path())),
  dhi_lut(std::make_unique<ForecastLut>(Config::get_instance()->get_dhi_path())),
  ghi_lut(std::make_unique<ForecastLut>(Config::get_instance()->get_ghi_path())) {}
