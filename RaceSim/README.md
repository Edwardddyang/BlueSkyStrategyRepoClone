# Race Simulation 
This framework intends to provide a variety of optimization algorithms and car models in order to identify the best speed profile for the solar car

# To Use

## Create the Simulation Code
Modify the main.cpp file in order to run a simulation. This would involve creating a car model, a route, a simulation object, and running an optimizer. The following ```main.cpp``` demonstrates the method by which to do this:

```
int main(int argc, char* argv[]) {
  spdlog::set_level(spdlog::level::debug);
  RUNTIME_EXCEPTION(argc == 2, "Exactly one argument is required for the config file path");

  char* strat_root = std::getenv("STRAT_ROOT");
  RUNTIME_EXCEPTION(strat_root != nullptr, "No STRAT_ROOT environment variable detected."
                                           "Set it to the full path to gen12_strategy/RaceSim.");

  std::filesystem::path config_file_path(argv[1]);
  Config::initialize(config_file_path, strat_root);

  /* Create a model of the car */
  std::shared_ptr<Car> car = CarFactory::get_car(Config::get_instance()->get_model());

  /* Create route */
  std::shared_ptr<Route> route = std::make_shared<Route>(Config::get_instance()->get_base_route_path());

  /* Create simulator */
  std::shared_ptr<Simulator> sim = SimulatorFactory::get_simulator(Config::get_instance()->get_simulator(), car);

  /* Create optimizer */
  std::shared_ptr<Optimizer> opt = OptimizerFactory::get_optimizer(Config::get_instance()->get_optimizer(),
                                                                   route, sim);

  /* Run optimizer */
  RacePlan viable_race_plan = opt->optimize();

  /* Print the optimal speed profile */

  return 0;
}
```
This example file is already present in the repository.

## Building

### Getting a Windows Toolchain
- Download Git (https://git-scm.com/downloads/win) which will also install the git bash terminal
- Download CMake windows installer (https://cmake.org/download/)
- Download GNU Make (https://gnuwin32.sourceforge.net/packages/make.htm). You should only need the link under the "Complete package, except sources" bullet point. Make sure the install goes to the C: directory and not to any of the 'Program Files' directories, make runs a script that seems to not like spaces in paths. After the install completes, add the `bin` folder to your PATH
- Download a MinGW64 toolchain package (https://www.mingw-w64.org/downloads/). I recommend using Msys2 (https://www.msys2.org/)
  - If you're using Msys2, open the installed MinGW64 terminal and run `pacman -S mingw-w64-x86_64-gcc`
  - Add the absolute path of msys2/mingw64/bin to PATH
- Open git bash and ensure that ```make --version``` ```gcc --version``` run without error. If you get a command not found error. Then you most likely did not set your environment variable paths correctly
- Install Python 3.13.0 (https://www.python.org/downloads/release/python-3130/)
- Run ```pip install cpplint==2.0.0```

### Building the executables and libraries

From gen12_Strategy/RaceSim/
```
1. mkdir build
2. cd build
3. cmake .. (May need to adjust depending on your compiler e.g. cmake .. -G "Unix Makefiles" for the windows toolchain above)
4. make install
5. export STRAT_ROOT=<FULL path to gen12_strategy/RaceSim>
6. Add the FULL gen12_strategy/RaceSim/install/lib to PATH environment variable
7. cd ../install/bin
8. ./opt.exe <config file relative to gen12_strategy/RaceSim>
```
Note that you will have to set STRAT_ROOT (step 5) every time you open your shell. If you're on linux, you can add it to your .bashrc in order to persist it. If you're on windows, you can add it to ~/.bash_profile in git bash. To compile with debug symbols, add `-DCMAKE_BUILD_TYPE=Debug` to step 3 and run make as normal

This has been tested on:
- Windows machine with make (GNU 3.81) with gcc (8.3), CMake (3.30)
- Ubuntu 20.04 with make (GNU 4.2) with gcc (9.4), CMake (3.16.3)

Dependencies: yaml-cpp (0.8.0), spdlog (1.14.1), googletest (1.14.0)

## Contributing
Before pushing, please ensure that ```make install``` runs with no testing or lint errors.

## Vscode Suggestions
Add the following into your .vscode/settings.json
```
"editor.insertSpaces": true
"editor.tabSize": 2,
"editor.detectIndentation": false
```

## Config File

The .yaml configuration file specifies parameters about your car and race route. A sample configuration is given in data/config/wsc_config.yaml. Note that all LUT paths are relative to gen12_strategy/RaceSim

## Extending the Project
You can add additional car models, and optimizers by subclassing from ```Car``` and ```Optimizer``` respectively. Implement the functions and make the relevant changes to the factories in their repsective header files. If you would like to add a new configuration parameter, follow the steps in ```Config.hpp``` in order to define a new macro for it. The parameter names in ```Config.hpp``` and the yaml file MUST match in order for it to work.

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

# TODOs
- Add a valgrind test
- Add acceleration model
- Profile the simulation. It's too slow
