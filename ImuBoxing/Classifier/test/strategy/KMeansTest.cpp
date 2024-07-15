#include "strategy/KMeans.h"
#include "classes/Data.h"
#include "Utils.h"

#include <algorithm>
#include <random>
#include <ranges>

#include <catch2/catch_test_macros.hpp>

SCENARIO("Should Classify the data according to the KMeans rule", "[Unit][strategy][KMeans][Classify]")
{

    GIVEN("a vector of Data centered at 90.0, 0.0, 45.0")
    {
        std::vector<std::vector<float>> featuresA;

        for (int i = 0; i < 10; ++i)
        {

            std::vector<float> point = {90.0, 0.0, 45.0};

            std::random_device randomDevice;
            std::mt19937 generator(randomDevice());
            std::uniform_real_distribution<float> distribution(-5.0, 5.0);

            float randomValue = distribution(generator);

            std::ranges::for_each(point, [&randomValue](auto &entry)
                                  { entry += randomValue; });

            featuresA.push_back(point);
        }

        std::vector<classifier::classes::Data> groupA;

        std::ranges::transform(featuresA,
                               std::back_inserter(groupA),
                               [](const auto &feature)
                               { return classifier::classes::Data(feature, 5); });

        AND_GIVEN("a vector of Data centered at 0.0, 90.0, 90.0")
        {

            std::vector<std::vector<float>> featuresB;
            for (int i = 0; i < 10; ++i)
            {

                std::vector<float> point = {0.0, 90.0, 90.0};

                std::random_device randomDevice;
                std::mt19937 generator(randomDevice());
                std::uniform_real_distribution<float> distribution(-5.0, 5.0);

                float randomValue = distribution(generator);

                std::ranges::for_each(point, [&randomValue](auto &entry)
                                      { entry += randomValue; });

                featuresB.push_back(point);
            }

            std::vector<classifier::classes::Data> groupB;

            std::ranges::transform(featuresB,
                                   std::back_inserter(groupB),
                                   [](const auto &feature)
                                   { return classifier::classes::Data(feature, 30); });

            AND_GIVEN("a KMeans Object")
            {

                classifier::strategy::KMeans KMeans;
                KMeans.AddGroup(groupA);
                KMeans.AddGroup(groupB);

                AND_GIVEN("A Data at 0.0, 85.0, 85.0")
                {
                    classifier::classes::Data dataPoint({0.0, 85.0, 85.0},
                                                        classifier::classes::Data::Poses::Unclassified);

                    WHEN("Classify is called")
                    {
                        const auto result = KMeans.Classify(dataPoint);

                        THEN("Resulting data is classified to group B")
                        {
                            classifier::classes::Data dataPointCompare({0.0, 85.0, 85.0}, 30);

                            CHECK(dataPoint == dataPointCompare);
                        }
                    }
                }

                AND_GIVEN("A Data at 45.0, 45.0, 45.0")
                {
                    classifier::classes::Data dataPoint({45.0, 45.0, 45.0}, classifier::classes::Data::Poses::Unclassified);

                    WHEN("Classify is called")
                    {
                        const auto result = KMeans.Classify(dataPoint);

                        THEN("Resulting data is classified to group B")
                        {
                            classifier::classes::Data dataPointCompare({45.0, 45.0, 45.0},
                                                                       classifier::classes::Data::Poses::Unknown);

                            CHECK(dataPoint == dataPointCompare);
                        }
                    }
                }
            }
        }
    }
}