/* Statically generate different car models */

#ifndef CAR_FACTORY_H
#define CAR_FACTORY_H

#include <base_car.h>
#include <v1_car.h>

enum class cars
{   
    V1_CAR = 0,
};

class Car_Factory {
private:
    static std::unordered_map<std::string, cars> config_to_car_model;
    static std::string DEFAULT_CAR;
public:
    static Car* get_car(std::string car_int);
    std::unordered_map<std::string, cars> get_config_to_car_models() {return config_to_car_model;}
};

#endif
