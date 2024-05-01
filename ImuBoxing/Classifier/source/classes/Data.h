#pragma once

#include "LabeledDistance.h"

#include <vector>
#include <cstdint>
#include <iostream>

namespace classifier::classes
{
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

        Data() : Features({}), Label(0) {}

        Data(const std::vector<double> &features,
             const uint8_t label) : Features(features),
                                    Label(label) {}

        bool operator==(const Data &other) const = default;

        Data operator+(const Data &other) const;
        Data operator/(double divisor) const;

        LabeledDistance EuclideanDistance(const Data &other);
    };

}

std::ostream &operator<<(std::ostream &os, const classifier::classes::Data &data);