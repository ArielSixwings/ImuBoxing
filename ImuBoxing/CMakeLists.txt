cmake_minimum_required(VERSION 3.5)
project(boxing)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

enable_testing()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")


find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

add_subdirectory(Telemetry)
add_subdirectory(Classifier)

add_executable(boxingModule main.cpp)
target_link_libraries(boxingModule Boost::system Boost::thread telemetrySource classifierSource)
