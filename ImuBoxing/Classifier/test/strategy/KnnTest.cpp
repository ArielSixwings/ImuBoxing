#include "strategy/Knn.h"
#include "classes/Data.h"

#include <algorithm>
#include <random>
#include <ranges>

#include <catch2/catch_test_macros.hpp>

SCENARIO("Should add data to a Knn Object", "[Unit][strategy][Knn][AddData][GetData]")
{
    GIVEN("a knn Object")
    {

        classifier::strategy::Knn knn(3);

        AND_GIVEN("a vector of Data")
        {
            std::vector<std::vector<float>> features;

            for (int i = 0; i < 10; ++i)
            {

                std::vector<float> group = {90.0, 0.0, 45.0};

                std::random_device randomDevice;
                std::mt19937 generator(randomDevice());
                std::uniform_real_distribution<float> distribution(-5.0, 5.0);

                float randomValue = distribution(generator);

                std::ranges::for_each(group, [&randomValue](auto &entry)
                                      { entry += randomValue; });

                features.push_back(group);
            }

            std::vector<classifier::classes::Data> group;

            std::ranges::transform(features,
                                   std::back_inserter(group),
                                   [](const auto &feature)
                                   { return classifier::classes::Data(feature, 0); });

            WHEN("AddData is called")
            {
                knn.AddData(group);

                THEN("Groups where inserted in the knn")
                {
                    CHECK(knn.GetData() == group);
                }
            }
        }
    }
}

SCENARIO("Should Classify the data according to the Knn rule", "[Unit][strategy][Knn][Classify]")
{
    GIVEN("a vector of Data centered at 90.0, 0.0, 45.0")
    {
        std::vector<std::vector<float>> features1;

        for (int i = 0; i < 10; ++i)
        {

            std::vector<float> point = {90.0, 0.0, 45.0};

            std::random_device randomDevice;
            std::mt19937 generator(randomDevice());
            std::uniform_real_distribution<float> distribution(-5.0, 5.0);

            float randomValue = distribution(generator);

            std::ranges::for_each(point, [&randomValue](auto &entry)
                                  { entry += randomValue; });

            features1.push_back(point);
        }

        std::vector<classifier::classes::Data> group;

        std::ranges::transform(features1,
                               std::back_inserter(group),
                               [](const auto &feature)
                               { return classifier::classes::Data(feature, 5); });

        AND_GIVEN("a vector of Data centered at 0.0, 90.0, 90.0")
        {

            std::vector<std::vector<float>> features2;
            for (int i = 0; i < 10; ++i)
            {

                std::vector<float> point = {0.0, 90.0, 90.0};

                std::random_device randomDevice;
                std::mt19937 generator(randomDevice());
                std::uniform_real_distribution<float> distribution(-5.0, 5.0);

                float randomValue = distribution(generator);

                std::ranges::for_each(point, [&randomValue](auto &entry)
                                      { entry += randomValue; });

                features2.push_back(point);
            }

            std::ranges::transform(features2,
                                   std::back_inserter(group),
                                   [](const auto &feature)
                                   { return classifier::classes::Data(feature, 30); });

            AND_GIVEN("a knn Object")
            {

                classifier::strategy::Knn knn(3);
                knn.AddData(group);

                AND_GIVEN("A Data at 0.0, 85.0, 85.0")
                {
                    classifier::classes::Data dataPoint({0.0, 85.0, 85.0}, 100);

                    WHEN("Classify is called")
                    {
                        const auto result = knn.Classify(dataPoint);

                        THEN("Resulting data is classified to group 2")
                        {
                            classifier::classes::Data dataPointCompare({0.0, 85.0, 85.0}, 30);

                            CHECK(dataPoint == dataPointCompare);
                        }
                    }
                }
            }
        }
    }
}