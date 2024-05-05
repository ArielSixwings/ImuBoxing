cmake_minimum_required(VERSION 3.0)
project(VTKPointSphere)

find_package(VTK REQUIRED)

include_directories(${VTK_INCLUDE_DIRS})

add_executable(VTKPointSphere VTKPointSphere.cpp)
target_link_libraries(VTKPointSphere ${VTK_LIBRARIES})
