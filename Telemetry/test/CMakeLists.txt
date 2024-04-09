include(AddCatch2)

add_executable(
  UnitTest
  SpaceSensorTest.cpp
)
target_link_libraries(
  UnitTest PRIVATE ImuNode Catch2::Catch2WithMain
)

include(CTest)
include(Catch)
catch_discover_tests(UnitTest)