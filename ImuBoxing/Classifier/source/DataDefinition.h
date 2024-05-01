#pragma once

#include <vector>
#include <cstdint>
#include <iostream>

namespace classifier
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
        enum Poses
        {
            Guard = 0,
            JabEnd = 1,
            HookEnd = 2,
            UppercutEnd = 3,
            Unknown = 100
        };

        std::vector<double> Features;
        uint8_t Label;

        Data(const std::vector<double> &features,
             const uint8_t label) : Features(features),
                                    Label(label) {}

        bool operator==(const Data &other) const = default;

        LabeledDistance EuclideanDistance(const Data &other);
    };

}

std::ostream &operator<<(std::ostream &os, const classifier::Data &data);

std::ostream &operator<<(std::ostream &os, const classifier::LabeledDistance &other);