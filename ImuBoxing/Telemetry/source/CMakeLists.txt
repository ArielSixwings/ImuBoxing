find_package(rclcpp REQUIRED)

add_library(telemetrySource ImuNode.cpp SpaceSensor.cpp)
target_include_directories(telemetrySource PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

ament_target_dependencies(telemetrySource rclcpp geometry_msgs std_srvs)