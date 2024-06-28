/* Starting point of a race simulation */

#include "spdlog/spdlog.h"
#include <stdlib.h>
#include <cstdlib>
#include <sim.h>
#include <config.h>
#include <string.h>
#include <stdio.h>
#include <car_factory.h>
#include <base_car.h>
#include <route.h>
#include <sim.h>
#include <base_opt.h>
#include <opt_factory.h>

int main(int argc, char* argv[]) {
    spdlog::set_level(spdlog::level::info);
    if (argc < 2) {
        spdlog::error("No config file supplied. Exiting");
        return 0;
    }

    const char* strat_root = std::getenv("STRAT_ROOT");
    if (strat_root == nullptr) {
        spdlog::error("No STRAT_ROOT environment variable detected. Set it to the full path to gen12_strategy/RaceSim. Exiting.");
        return 0;    
    }

    std::string config_file_path = argv[1];
    Config::initialize(config_file_path, std::string(strat_root));

    /* Create a model of the car */
    Car* car = Car_Factory::get_car(Config::get_instance()->get_model());
    
    /* Create route */
    Route route = Route();

    /* Create simulator */
    Sim sim = Sim(car);

    /* Create optimizer */
    Optimizer* opt = Opt_Factory::get_optimizer(Config::get_instance()->get_optimizer(), route, sim);
    std::vector<uint32_t> result_speed_profile_km = opt->optimize();
    spdlog::info("Viable Speed Profile: {}", result_speed_profile_km[0]);

    return 0;
}
