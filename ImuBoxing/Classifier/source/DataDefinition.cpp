#include "DataDefinition.h"

#include <cmath>
#include <iostream>

namespace classifier::strategy
{

    LabeledDistance Data::EuclideanDistance(const Data &other)
    {
        if (Features.size() != other.Features.size())
        {
            throw std::invalid_argument("Data Features must be of the same size.");
        }

        double sumSquaredDiff = 0.0;

        for (size_t i = 0; i < Features.size(); i++)
        {
            sumSquaredDiff += std::pow(Features[i] - other.Features[i], 2);
        }

        return LabeledDistance(std::sqrt(sumSquaredDiff), other.Label);
    }
}

std::ostream &operator<<(std::ostream &os, const classifier::strategy::Data &data)
{
    os << "Features: [";
    for (size_t i = 0; i < data.Features.size(); ++i)
    {
        os << data.Features[i];
        if (i != data.Features.size() - 1)
        {
            os << ", ";
        }
    }
    os << "], Label: " << static_cast<int>(data.Label);
    return os;
}

std::ostream &operator<<(std::ostream &os, const classifier::strategy::LabeledDistance &other)
{
    os << "LabeledDistance { Distance: " << other.Distance << ", Label: " << static_cast<int>(other.Label) << " }";
    return os;
}