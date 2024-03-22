include(AddCatch2)

add_executable(
  UnitTest
  MathUtilTest.cpp
)
target_link_libraries(
  UnitTest PRIVATE MathUtil Catch2::Catch2WithMain
)

include(CTest)
include(Catch)
catch_discover_tests(UnitTest)