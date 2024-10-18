# Array Simulation

Requirements:
- Windows 64-bit
- OpenGL
- Boost (For CGAL)

# Getting a Windows Compiler

- Download Git (https://git-scm.com/downloads/win) which will also install the git bash terminal
- Download CMake windows installer (https://cmake.org/download/)
- Download Msys2 (https://www.msys2.org/)
- Follow the instructions on the Msys2 home page with the following modifications to the steps:
  - In step 5, instead of launching the UCRT64 environment which has proven to be buggy, use the MinGW64 environment. You can find this as one of the applications in the downloaded msys2/ folder.
  - In step 6, run `pacman -S mingw-w64-x86_64-gcc` inside the MinGW64 terminal. Then install `make` with `pacman -S mingw-w64-x86_64-make` inside the same MinGW64 terminal. After installing make, re-name the ```mingw32-make``` file in msys2/mingw64/bin to ```make```.
- Add the absolute path to msys2/mingw64/bin to your PATH environment variable
- Open git bash and ensure that ```make --version``` ```gcc --version``` run without error. If you get a command not found error, then you most likely did not set your environment variable paths correctly

# Windows Build
1. Get boost: https://archives.boost.io/release/1.86.0/source/ and download the extracted folder onto your system. 
2. ```export BOOST_ROOT=<absolute path to the Boost folder>```
3. ```export ARRAY_ROOT=<absolute path to the ArraySim folder>```
4. mkdir CPP/build
5. cd CPP/build
6. cmake .. -G "Unix Makefiles"
7. make

Add the command in step 2 and 3 to ~/.bash_profile. Otherwise, you will have to run those two commands every time you open your terminal
