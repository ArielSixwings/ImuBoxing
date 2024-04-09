#include "SpaceSensor.h"

#include <catch2/catch_test_macros.hpp>

SCENARIO("Should return the size of the euler angles response", "[Unit]")
{

  GIVEN("")
  {

    WHEN("EulerAngle::SizeInBytes is called")
    {
      const auto result = telemetry::SpaceSensor::EulerAngle::SizeInBytes();

      THEN("Result is 15")
      {
        CHECK(result == 15);
      }
    }
  }
}
