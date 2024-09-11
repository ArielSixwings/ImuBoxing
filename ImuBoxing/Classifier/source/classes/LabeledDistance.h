#pragma once

#include <vector>
#include <cstdint>
#include <iostream>

namespace classifier::classes
{
    struct LabeledDistance
    {
        LabeledDistance(const float distance, const uint8_t label)
            : Distance(distance), Label(label)
        {
        }

        bool operator<(const LabeledDistance &other) const
        {
            return Distance < other.Distance;
        }

        bool operator==(const LabeledDistance &other) const = default;

        float Distance;
        uint8_t Label;
    };

}

std::ostream &operator<<(std::ostream &os, const classifier::classes::LabeledDistance &other);