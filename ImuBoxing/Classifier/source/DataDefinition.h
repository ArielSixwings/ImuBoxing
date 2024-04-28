#pragma once

#include <vector>
#include <cstdint>
#include <iostream>

namespace classifier::strategy
{

    struct LabeledDistance
    {
        LabeledDistance(const double distance, const uint8_t label)
            : Distance(distance), Label(label)
        {
        }

        bool operator<(const LabeledDistance &other) const
        {
            return Distance < other.Distance;
        }

        bool operator==(const LabeledDistance &other) const = default;

        double Distance;
        uint8_t Label;
    };

    struct Data
    {
        std::vector<double> Features;
        uint8_t Label;

        Data(const std::vector<double> &features,
             const uint8_t label) : Features(features),
                                    Label(label) {}

        bool operator==(const Data &other) const = default;
        LabeledDistance EuclideanDistance(const Data &other);
    };

}

std::ostream &operator<<(std::ostream &os, const classifier::strategy::Data &data);

std::ostream &operator<<(std::ostream &os, const classifier::strategy::LabeledDistance &other);