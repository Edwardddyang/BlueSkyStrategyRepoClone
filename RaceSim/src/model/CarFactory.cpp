/* Statically generate different car models */

#include <unordered_map>
#include <string>
#include <memory>

#include "model/CarFactory.hpp"

std::unordered_map<std::string, CarModels> CarFactory::config_to_car_model = {
  {"Gen 11.5", CarModels::V1_CAR},
  {"Gen 12", CarModels::V2_CAR},
};
const char CarFactory::DEFAULT_CAR[] = "Gen 12";

std::shared_ptr<Car> CarFactory::get_car(const std::string car_type) {
  CarModels model;
  if (config_to_car_model.find(car_type) != config_to_car_model.end()) {
    model = config_to_car_model[car_type];
  } else {
    /* Default to acceleration car model */
    model = config_to_car_model[std::string(DEFAULT_CAR)];
  }

  if (model == CarModels::V1_CAR) {
    spdlog::info("Using V1 Car Model");
    return std::make_shared<V1Car>();
  } else if (model == CarModels::V2_CAR) {
    spdlog::info("Using V2 Car Model");
    return std::make_shared<V2Car>();
  } else {
    RUNTIME_EXCEPTION(false, "No valid car requested. Check the DEFAULT_CAR variable");
  }

  return nullptr;
}
