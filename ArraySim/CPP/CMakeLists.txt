cmake_minimum_required(VERSION 3.12)
project(ArraySim VERSION 1.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build GLFW and OpenGL backends
add_subdirectory(${CMAKE_SOURCE_DIR}/libs/GLFW) # glfw3
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
)

add_executable(arraySim ${PROJECT_SOURCES})
target_include_directories(arraySim PRIVATE
    ${CMAKE_SOURCE_DIR}/include/gui
    ${CMAKE_SOURCE_DIR}/libs/ImGuiFileDialog/
)
target_link_libraries(arraySim PRIVATE imgui)
target_compile_features(arraySim PRIVATE cxx_std_17)
