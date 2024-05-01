#include "Utils.h"
#include "classes/Data.h"

#include <algorithm>
#include <ranges>

#include <catch2/catch_test_macros.hpp>

SCENARIO("Should calculate the mean point  of a Data vector", "[Unit][Utils][Mean]")
{

    GIVEN("a vector of Data")
    {
        std::vector<std::vector<double>> features;

        for (int i = 0; i < 10; ++i)
        {

            std::vector<double> group = {90.0, 0.0, 45.0};

            std::ranges::for_each(group, [&i](auto &entry)
                                  { entry += i; });

            features.push_back(group);
        }

        std::vector<classifier::classes::Data> group;

        std::ranges::transform(features,
                               std::back_inserter(group),
                               [](const auto &feature)
                               { return classifier::classes::Data(feature, 6); });

        WHEN("Mean is called")
        {
            const auto result = classifier::Utils::Mean(group);

            THEN("Result has value")
            {

                REQUIRE(result.has_value());

                AND_THEN("Mean result has expected value")
                {
                    const classifier::classes::Data compare({94.5, 4.5, 49.5}, 6);

                    CHECK(result.value() == compare);
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