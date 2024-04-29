#include "Utils.h"
#include "DataDefinition.h"

#include <algorithm>
#include <random>
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

            std::random_device randomDevice;
            std::mt19937 generator(randomDevice());
            std::uniform_real_distribution<double> distribution(-5.0, 5.0);

            double randomValue = distribution(generator);

            std::ranges::for_each(group, [&randomValue](auto &entry)
                                  { entry += randomValue; });

            features.push_back(group);
        }

        std::vector<classifier::Data> group;

        std::ranges::transform(features,
                               std::back_inserter(group),
                               [](const auto &feature)
                               { return classifier::Data(feature, 6); });

        WHEN("Mean is called")
        {
            const auto result = classifier::Utils::Mean(group);

            THEN("Result has value")
            {

                REQUIRE(result.has_value());

                AND_THEN("Mean result has expected value")
                {
                    const classifier::Data compare({0.0, 0.0, 0.0}, 6);

                    CHECK(result.value() == compare);
                }
            }
        }
    }
}
