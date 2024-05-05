find_package(rclcpp REQUIRED)
find_package(VTK REQUIRED)

include_directories(${VTK_INCLUDE_DIRS})

add_library(dataVisualizationSource 
            GroupDisplayer.cpp )
            
target_include_directories(dataVisualizationSource PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

ament_target_dependencies(dataVisualizationSource rclcpp geometry_msgs std_srvs VTK)
