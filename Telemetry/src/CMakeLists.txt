find_package(rclcpp REQUIRED)

add_library(ImuNode ImuNode.cpp SpaceSensor.cpp)
target_include_directories(ImuNode PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

ament_target_dependencies(ImuNode rclcpp geometry_msgs std_srvs)