/* Statically generate different car models */

#include "CarFactory.hpp"
#include "Car.hpp"
#include "V1Car.hpp"

std::unordered_map<std::string, CarModels> CarFactory::config_to_car_model = {
    {"Gen 11.5", CarModels::V1_CAR},
};
std::string CarFactory::DEFAULT_CAR = "Gen 11.5";

std::unique_ptr<Car> CarFactory::get_car(const std::string car_type) {
    CarModels model;
    if (config_to_car_model.find(car_type) != config_to_car_model.end()) {
        model = config_to_car_model[car_type];
    } else {
        /* Default to gen 11.5 car model */
        model = config_to_car_model[DEFAULT_CAR];
    }

    if (model == CarModels::V1_CAR) {
        spdlog::info("Using V1 Car Model");
        return std::make_unique<V1Car>();
    } else {
        return nullptr;
    }
}
