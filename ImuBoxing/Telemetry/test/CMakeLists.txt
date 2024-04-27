include(AddCatch2)

find_package(rclcpp REQUIRED)

add_executable(TelemetryTest SpaceSensorTest.cpp)

add_executable(TelemetryMock mock/Publisher.cpp)

ament_target_dependencies(TelemetryMock rclcpp geometry_msgs std_srvs)

target_link_libraries(
  TelemetryTest PRIVATE telemetrySource Catch2::Catch2WithMain
)

include(CTest)
include(Catch)
catch_discover_tests(TelemetryTest)
