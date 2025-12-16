cmake_minimum_required(VERSION 3.21)

message(STATUS "CMAKE_VERSION: ${CMAKE_VERSION}")

project(bottlefishTemplates
    VERSION 0.1.0
    DESCRIPTION "C++ Template Library for Bottlefish"
    LANGUAGES CXX
)

if(EXISTS "${PROJECT_SOURCE_DIR}/local_config.cmake")
    include(local_config.cmake)
    message(STATUS "Using local_config.cmake")
endif()

include(cmake/settings.cmake)

find_package(fmt CONFIG REQUIRED)
find_package(Eigen3 3.4 CONFIG REQUIRED)
find_package(CLI11 CONFIG REQUIRED)
find_package(absl CONFIG REQUIRED)

add_subdirectory(lib)
add_subdirectory(cli)