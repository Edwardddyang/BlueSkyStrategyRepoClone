/* Statically generate different car models */

#pragma once

#include "Car.hpp"
#include "V1Car.hpp"

enum class CarModels
{   
    V1_CAR = 0,
};

class CarFactory {
private:
    static std::unordered_map<std::string, CarModels> config_to_car_model;
    static std::string DEFAULT_CAR;
public:
    /** @brief Return a car of unique_ptr type
     * 
     * @param car_int: A CarModels enum indicating the model to create
     */
    static std::unique_ptr<Car> get_car(const std::string car_int);
};
