#include "Data.h"

#include <algorithm>
#include <iterator>
#include <cmath>
#include <iostream>
#include <numeric>
#include <ranges>

namespace classifier::classes
{
    LabeledDistance Data::EuclideanDistance(const Data &other)
    {
        if (Features.size() != other.Features.size())
        {
            throw std::invalid_argument("Data Features must be of the same size");
        }

        float sumSquaredDiff = 0.0;

        for (size_t i = 0; i < Features.size(); i++)
        {
            sumSquaredDiff += std::pow(Features[i] - other.Features[i], 2);
        }

        return LabeledDistance(std::sqrt(sumSquaredDiff), other.Label);
    }

    Data Data::operator+(const Data &other) const
    {
        Data result;

        result.Label = Label;
        std::transform(Features.begin(), Features.end(), other.Features.begin(),
                       std::back_inserter(result.Features), std::plus<float>());
        return result;
    }

    Data Data::operator/(float divisor) const
    {
        if (divisor == 0)
        {

            throw std::invalid_argument("Division by zero");
        }
        Data result;
        result.Label = Label;
        std::ranges::transform(Features, std::back_inserter(result.Features),
                               [divisor](float feature)
                               { return feature / divisor; });
        return result;
    }
}

std::ostream &operator<<(std::ostream &os, const classifier::classes::Data &data)
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
