/* Statically generate different car models */

#pragma once

#include <unordered_map>
#include <string>
#include <memory>

#include "model/Car.hpp"
#include "model/V1Car.hpp"
#include "model/V2Car.hpp"

enum class CarModels {
  V1_CAR = 0,
  V2_CAR = 1,
};

class CarFactory {
 private:
  static std::unordered_map<std::string, CarModels> config_to_car_model;
  static const char DEFAULT_CAR[];

 public:
  /** @brief Return a car of unique_ptr type
   * 
   * @param car_int: A CarModels enum indicating the model to create
   */
  static std::shared_ptr<Car> get_car(const std::string car_int);
};
