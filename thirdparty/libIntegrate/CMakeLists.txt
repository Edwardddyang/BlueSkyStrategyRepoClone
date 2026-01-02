cmake_minimum_required(VERSION 3.20)
project(libIntegrate)

set(HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_library(libIntegrate INTERFACE)

target_include_directories(libIntegrate INTERFACE ${HEADER_DIRS})
