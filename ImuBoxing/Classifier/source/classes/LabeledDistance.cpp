#include "LabeledDistance.h"

#include <iostream>

std::ostream &operator<<(std::ostream &os, const classifier::classes::LabeledDistance &other)
{
    os << "LabeledDistance { Distance: " << other.Distance << ", Label: " << static_cast<int>(other.Label) << " }";
    return os;
}