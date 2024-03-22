#include "MathUtil.h"

#include <catch2/catch_test_macros.hpp>

// g++ -c MathUtil.cpp -o MathUtil.o
// g++ -c MathUtilTest.cpp -o MathUtilTest.o `catch2_config.cpp` -lcatch2_main
// g++ MathUtil.o MathUtilTest.o -o test

SCENARIO("MathUtil: Adding numbers") {
  
  GIVEN("Two numbers") {
  
    double a = 2.5;
    double b = 3.1;

    WHEN("they are added together") {
      
      double result = MathUtil::Add(a, b);

      THEN("the result should be their sum") {
        
        CHECK(result == 5.6); // Approx matcher for floating-point comparison
      }
    }
  }
}
