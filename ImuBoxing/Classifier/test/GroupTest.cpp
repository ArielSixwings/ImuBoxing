#include "Utils.h"
#include "classes/Group.h"

#include <algorithm>
#include <ranges>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

SCENARIO("Should calculate the mean point  of a Data vector", "[Unit][Group][Mean]")
{

    GIVEN("a vector of Data")
    {
        std::vector<std::vector<float>> features;

        for (int i = 0; i < 10; ++i)
        {

            std::vector<float> group = {90.0, 0.0, 45.0, 20};

            std::ranges::for_each(group, [&i](auto &entry)
                                  { entry += i; });

            features.push_back(group);
        }

        std::vector<classifier::classes::Data> points;

        std::ranges::transform(features,
                               std::back_inserter(points),
                               [](const auto &feature)
                               { return classifier::classes::Data(feature, 6); });

        WHEN("A Group objet is created")
        {
            classifier::classes::Group group(points);

            THEN("Mean and StandardDeviation has value")
            {

                const auto mean = group.GetMean();
                const auto standardDeviation = group.GetStandardDeviation();

                REQUIRE(mean.has_value());
                REQUIRE(standardDeviation.has_value());

                AND_THEN("Mean and StandardDeviation result has expected value")
                {
                    const classifier::classes::Data meanCompare({94.5, 4.5, 49.5, 20}, 6);
                    const float standardDeviationCompare = 15.7321327226;

                    CHECK(mean.value() == meanCompare);
                    CHECK(standardDeviation.value() == Catch::Approx(standardDeviationCompare));
                }
            }
        }
    }
}