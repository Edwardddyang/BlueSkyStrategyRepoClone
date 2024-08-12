cmake_minimum_required(VERSION 3.12)
project(ArraySim VERSION 1.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build GLFW and OpenGL backends
add_subdirectory(${CMAKE_SOURCE_DIR}/libs/GLFW) # glfw3
add_subdirectory(${CMAKE_SOURCE_DIR}/libs/glm) # glm
add_subdirectory(${CMAKE_SOURCE_DIR}/libs/glad) # glad
add_subdirectory(${CMAKE_SOURCE_DIR}/libs/Clipper2/CPP) # Clipper2
find_package(OpenGL REQUIRED) # OpenGL provided by Windows

# Build ImGUI
set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/libs/imgui)
set(IMGUI_HEADERS
    ${IMGUI_DIR}/
    ${IMGUI_DIR}/backends/
    ${IMGUI_DIR}/misc/cpp/
)
set(IMGUI_SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/misc/cpp/imgui_stdlib.cpp
)
add_library(imgui
    ${IMGUI_SOURCES}
)
target_link_libraries(imgui glfw OpenGL::GL)
target_include_directories(imgui PUBLIC
    ${IMGUI_HEADERS}
)

# Build project
set (PROJECT_SOURCES
    ${CMAKE_SOURCE_DIR}/src/main.cpp
    ${CMAKE_SOURCE_DIR}/src/gui/gui.cpp
    ${CMAKE_SOURCE_DIR}/libs/ImGuiFileDialog/ImGuiFileDialog.cpp # For file dialog
    ${CMAKE_SOURCE_DIR}/src/model/mesh.cpp
    ${CMAKE_SOURCE_DIR}/src/shader/Shader.cpp
    ${CMAKE_SOURCE_DIR}/src/model/model.cpp
    ${CMAKE_SOURCE_DIR}/src/camera/camera.cpp
    ${CMAKE_SOURCE_DIR}/src/irradiance/sun_plane.cpp
    ${CMAKE_SOURCE_DIR}/src/Utilities.cpp
    ${CMAKE_SOURCE_DIR}/libs/quickhull/QuickHull.cpp
    ${CMAKE_SOURCE_DIR}/src/irradiance/irradiance_csv.cpp
)

add_executable(arraySim ${PROJECT_SOURCES})
target_include_directories(arraySim PRIVATE
    ${CMAKE_SOURCE_DIR}/include/gui
    ${CMAKE_SOURCE_DIR}/include/model
    ${CMAKE_SOURCE_DIR}/include/shader
    ${CMAKE_SOURCE_DIR}/include/camera
    ${CMAKE_SOURCE_DIR}/include/irradiance
    ${CMAKE_SOURCE_DIR}/libs/ImGuiFileDialog/
    ${CMAKE_SOURCE_DIR}/libs/glm/glm
    ${CMAKE_SOURCE_DIR}/libs/glad/include
    ${CMAKE_SOURCE_DIR}/include/
    ${CMAKE_SOURCE_DIR}/libs/quickhull
)

# Filesystem requires C++17 standard which might not be available on all compilers
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++17" COMPILER_SUPPORTS_CXX17)
if (COMPILER_SUPPORTS_CXX17)
    message(STATUS "Compiler supports C++17")
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
else()
    message(FATAL_ERROR "Compiler does not support C++17")
endif()

# Manually link against filesystem library for older compilers
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0)
        message(STATUS "Using GCC version ${CMAKE_CXX_COMPILER_VERSION}. Manually linking against stdc++fs.")
        target_link_libraries(arraySim PRIVATE stdc++fs)
    endif()
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 7.0)
        message(STATUS "Using Clang version ${CMAKE_CXX_COMPILER_VERSION}. Manually linking against stdc++fs")
        target_link_libraries(arraySim PRIVATE c++fs)
    endif()
endif()

target_link_libraries(arraySim PRIVATE imgui glm glad Clipper2)
target_compile_features(arraySim PRIVATE cxx_std_17)
