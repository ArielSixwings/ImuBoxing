#pragma once

#include <chrono>

namespace classifier::classes
{
    struct Technique
    {
        enum BoxingTechnique
        {
            OnGuard = 0,
            Jab = 1,
            Hook = 2,
            Uppercut = 3,
            Moving = 4,
            Unknown = 100,
            Unclassified = 255
        };

        uint8_t Label = BoxingTechnique::Unclassified;
        std::chrono::milliseconds Duration;

        bool operator==(const Technique &other) const = default;
    };
}
