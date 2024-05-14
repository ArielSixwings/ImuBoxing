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

add_subdirectory(Classifier)
add_subdirectory(DataVisualization)
add_subdirectory(Telemetry)

add_executable(telemetry Telemetry/main.cpp)
target_link_libraries(telemetry Boost::system Boost::thread telemetrySource)

add_executable(poseClassifier Classifier/PoseClassifier.cpp)
target_link_libraries(poseClassifier Boost::system Boost::thread classifierSource)

add_executable(techniqueClassifier Classifier/TechniqueClassifier.cpp)
target_link_libraries(techniqueClassifier Boost::system Boost::thread classifierSource)

add_executable(dataVisualization DataVisualization/main.cpp)
target_link_libraries(dataVisualization Boost::system Boost::thread dataVisualizationSource)