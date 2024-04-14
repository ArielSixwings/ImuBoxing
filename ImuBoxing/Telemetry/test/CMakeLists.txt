include(AddCatch2)

add_executable(
  TelemetryTest
  SpaceSensorTest.cpp
)
target_link_libraries(
  TelemetryTest PRIVATE telemetrySource Catch2::Catch2WithMain
)

include(CTest)
include(Catch)
catch_discover_tests(TelemetryTest)