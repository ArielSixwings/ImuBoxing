#include "SpaceSensor.h"

#include <catch2/catch_test_macros.hpp>

// g++ -c MathUtil.cpp -o MathUtil.o
// g++ -c MathUtilTest.cpp -o MathUtilTest.o `catch2_config.cpp` -lcatch2_main
// g++ MathUtil.o MathUtilTest.o -o test

SCENARIO("SpaceSensorTest")
{

  GIVEN("Two numbers")
  {

    WHEN("they are added together")
    {

      THEN("the result should be their sum")
      {

        CHECK(true);
      }
    }
  }
}
