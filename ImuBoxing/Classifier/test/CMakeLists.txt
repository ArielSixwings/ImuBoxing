include(AddCatch2)

add_executable(
  ClassifierTest
  GroupTest.cpp
  DataTest.cpp
  strategy/KnnTest.cpp
)
target_link_libraries(
  ClassifierTest PRIVATE classifierSource Catch2::Catch2WithMain
)

include(CTest)
include(Catch)
catch_discover_tests(ClassifierTest)