#include <stdlib.h>
#include <string>

#include "spdlog/spdlog.h"
#include "sim/Simulator.hpp"

Simulator::Simulator(std::shared_ptr<Car> model) :
  car(model),
  wind_speed_lut(std::make_unique<ForecastLut>(Config::get_instance()->get_wind_speed_path())),
  wind_dir_lut(std::make_unique<ForecastLut>(Config::get_instance()->get_wind_direction_path())) {
  use_ghi = Config::get_instance()->get_use_ghi();
  if (use_ghi) {
    spdlog::info("Using GHI Solar Source");
    ghi_lut = std::make_unique<ForecastLut>(Config::get_instance()->get_ghi_path());
  } else {
    spdlog::info("Using DNI, DHI Solar Source");
    dni_lut = std::make_unique<ForecastLut>(Config::get_instance()->get_dni_path());
    dhi_lut = std::make_unique<ForecastLut>(Config::get_instance()->get_dhi_path());
  }
}
