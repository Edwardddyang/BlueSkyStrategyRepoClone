# Race Strategy
This repository provides a library comprised of solar car energy model, simulators and optimizers for the Formula Sun Grand Prix (FSGP), American Solar Challenge (ASC), World Solar Challenge (WSC).

## Toolchain
This section provides steps for installing the compiler and build toolchain

### Linux
`sudo apt-get install build-essential cmake git python3 python3-pip`

### MacOS
```
xcode-select --install
brew install cmake git python
```

### Windows
The recommended toolchain for Windows is MSVC
1. Download Visual Studio from https://visualstudio.microsoft.com/downloads/ and download the "C++ for desktop development" package after launching the installer
2. Download LLVM for Windows https://github.com/llvm/llvm-project/releases version 20.1.2
3. Download Python 3.13 or above from the Microsoft Store
4. Use the x64 Native Tools Command Prompt for VS 2022 terminal

## Build
C++20 or above is required to build the project. The generated artifact is a dynamic library. Dependencies are managed using conan, which requires a valid python installation.
```
pip install conan==2.19
conan profile detect --force
conan install . --output-folder=build --build=missing -s compiler.cppstd=20
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/build/Release/generators/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DRUN_TEST=ON
cmake --build build --config Release
```
Ensure that your conan profile detects a cpp compiler that supports standard 20 or above. You may have to modify the above steps to work with your chosen toolchain e.g. if using MSVC, you will need to add `-G Ninja` to the cmake configuration command

## Quick Start
The build command above also generates an executable called `wsc.exe` from `src/WSC.cpp`. This performs a simple constant speed optimization for WSC. Refer to this script when using the library.

### Config File

Configuration parameters e.g. route files, car characterizations, simulation settings are passed into the simulation environment using a single .yaml file. An example can be found in `data/config/wsc_config.yaml`.

### STRAT_ROOT
Almost all exported objects rely on the existence of an environment variable called `STRAT_ROOT` that holds the absolute path to the root directory of all lookup tables. As such, file paths in the configuration file will be pre-pended with `STRAT_ROOT`. This allows lookup tables to be stored in the repository itself.

## Contributing
- Code should comply with CPP Core Guidelines https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines
- All code must pass the following with 0 errors:
```
ctest --output-on-failure
cmake --build build --target format
cmake --build build --target tidy
```
- All units must be metric base e.g. grams, celsius, meters. If units are non-standard, indicate them in variable name e.g. `double speed_kph = 6.5;`
- Private class members should be appended with `_`

Before getting a review, ensure that code passes unit tests, and formatted according to the above.

## Vscode Suggestions
Add the following into your .vscode/settings.json
```
"editor.insertSpaces": true
"editor.tabSize": 2,
"editor.detectIndentation": false
```

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

# TODO
- Fix FSGP Optimization
- Finish ASC Simulator and do ASC Optimization
- Unit Tests
- Make CRTP derived methods private
- Don't print dataset for all solutions - makes things extremely slow
- Exception handling is pretty bad (next lead's problem)
