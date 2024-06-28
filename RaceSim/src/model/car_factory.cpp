/* Statically generate different car models */

#include <car_factory.h>
#include <base_car.h>
#include <v1_car.h>

std::unordered_map<std::string, cars> Car_Factory::config_to_car_model = {
    {"Gen 11.5", cars::V1_CAR},
};
std::string Car_Factory::DEFAULT_CAR = "Gen 11.5";

Car* Car_Factory::get_car(std::string car_type) {
    cars model;
    if (config_to_car_model.find(car_type) != config_to_car_model.end()) {
        model = config_to_car_model[car_type];
    } else {
        /* Default to gen 11.5 car model */
        model = config_to_car_model[DEFAULT_CAR];
    }

    if (model == cars::V1_CAR) {
        spdlog::info("Using V1 Car Model");
        return (Car*) new V1_Car();
    } else {
        return nullptr;
    }
}