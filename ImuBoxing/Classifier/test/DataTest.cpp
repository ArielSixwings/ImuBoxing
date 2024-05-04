#include "classes/Data.h"
#include "classes/LabeledDistance.h"

#include <algorithm>
#include <random>
#include <ranges>

#include <catch2/catch_test_macros.hpp>

SCENARIO("Should compute the euclidean distance between two Data",
         "[Unit][Data][EuclideanDistance]")
{
  GIVEN("a Data at 90.0, 0.0, 45.0")
  {
    classifier::classes::Data dataA({90.0, 0.0, 45.0}, 30);

    AND_GIVEN("A Data at 0.0, 90.0, 90.0")
    {
      classifier::classes::Data dataB({0.0, 90.0, 90.0}, 8);

      WHEN("EuclideanDistance is called")
      {
        const auto result = dataA.EuclideanDistance(dataB);

        THEN("Resulting LabeledDistance has expected value")
        {
          classifier::classes::LabeledDistance compare(135.0, 8);

          CHECK(result == compare);
        }
      }
    }
  }
}

SCENARIO("Should operate with Data", "[Unit][Utils][operators]")
{

  GIVEN("Two Data")
  {
    classifier::classes::Data valueA({10.1, 10.1, 10.1}, 2);
    classifier::classes::Data valueB({5.0, 10.0, 1.0}, 3);

    WHEN("operator+ is called")
    {
      const auto result = valueA + valueB;

      THEN("Result has sum of Features")
      {
        const classifier::classes::Data compare({15.1, 20.1, 11.1}, 2);

        CHECK(result == compare);
      }
    }
  }

  GIVEN("A Data")
  {
    classifier::classes::Data valueA({10.1, 10.1, 10.1}, 2);

    WHEN("operator/ is called")
    {
      const auto result = valueA / 2.0;

      THEN("Result has Features that were divided by the value")
      {
        const classifier::classes::Data compare({5.05, 5.05, 5.05}, 2);

        CHECK(result == compare);
      }
    }
  }
}