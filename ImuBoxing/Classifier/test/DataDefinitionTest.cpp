#include "DataDefinition.h"

#include <algorithm>
#include <random>
#include <ranges>

#include <catch2/catch_test_macros.hpp>

SCENARIO("Should compute the euclidean distance between two Data",
         "[Unit][Data][EuclideanDistance]")
{
  GIVEN("a Data at 90.0, 0.0, 45.0")
  {
    classifier::Data dataA({90.0, 0.0, 45.0}, 30);

    AND_GIVEN("A Data at 0.0, 90.0, 90.0")
    {
      classifier::Data dataB({0.0, 90.0, 90.0}, 8);

      WHEN("EuclideanDistance is called")
      {
        const auto result = dataA.EuclideanDistance(dataB);

        THEN("Resulting LabeledDistance has expected value")
        {
          classifier::LabeledDistance compare(135.0, 8);

          CHECK(result == compare);
        }
      }
    }
  }
}