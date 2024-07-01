# Race Simulation 
This framework intends to provide a variety of optimization algorithms and car models in order to identify the best speed profile for the solar car

# To Use

## Create the Simulation Code
Modify the main.cpp file in order to run a simulation. This would involve creating a car model, a route, a simulation object, and running an optimizer. The following ```main.cpp``` demonstrates the method by which to do this:

```
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
```
This example file is already present in the repository.

## Building
From Gen12_Strategy/RaceSim/
```
1. mkdir build
2. cd build
3. cmake .. (May need to adjust depending on your compiler)
4. make install
5. export STRAT_ROOT=<FULL path to gen12_strategy/RaceSim>
6. Add the FULL gen12_strategy/RaceSim/install/lib to PATH environment variable
7. cd ../install/bin
8. ./opt.exe <config file relative to gen12_strategy/RaceSim>
```
Note that you will have to set STRAT_ROOT (step 5) every time you open your shell. If you're on linux, you can add it to your .bashrc in order to persist it.

Dependencies: yaml-cpp (0.8.0), spdlog (1.14.1), googletest (1.14.0)

## Config File

The .yaml configuration file specifies parameters about your car and race route. A sample configuration is given in data/config/wsc_config.yaml. Note that all LUT paths are relative to gen12_strategy/RaceSim

## Extending the Project
You can add additional car models, and optimizers by subclassing from ```Car``` and ```Optimizer``` respectively. Implement the functions and make the relevant changes to the factories in their repsective header files. If you would like to add a new configuration parameter, follow the steps in ```config.h``` in order to define a new macro for it. The parameter names in ```config.h``` and the yaml file MUST match in order for it to work.

# Scientific Units used 
All units are listed below. Any others not mentioned are also in the base metric system. The configuration file specifies units and is ultimately converted to these units

energy -> kilowatthour\
power -> Watt\
speed -> meters / second\
mass -> kilogram\
temperature -> celsius\
distance -> meters\
angles -> degrees\
force -> newton\
area -> meters squared

When naming variables representing a scientific unit, please include the units if it does not conform to the ones listed above and also is not in the base metric sytem e.g. ```double speed_kph = 70.2;```
