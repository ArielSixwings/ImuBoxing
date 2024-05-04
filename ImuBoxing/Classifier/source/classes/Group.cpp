#include "Group.h"

#include <algorithm>
#include <iterator>
#include <ranges>
#include <cmath>

namespace classifier::classes
{
    Group::Group(const std::vector<Data> &points) : m_points(points)
    {
        m_mean = Mean();
        m_standardDeviation = StandardDeviation();
    }

    std::optional<Data> Group::Mean()
    {
        if (m_points.empty())
        {
            return {};
        }

        const auto numFeatures = m_points[0].Features.size();

        auto valid = true;

        std::ranges::for_each(m_points, [&numFeatures, &valid](const auto point)
                              { if(point.Features.size() != numFeatures) valid = false; });

        const auto label = m_points[0].Label;

        std::ranges::for_each(m_points, [&label, &valid](const auto point)
                              { if(point.Label != label) valid = false; });

        if (not valid)
        {
            return {};
        }

        classes::Data sumOfData(std::vector<double>(numFeatures, 0.0), label);

        std::ranges::for_each(m_points, [&sumOfData](const auto point)
                              { sumOfData = sumOfData + point; });

        const auto mean = sumOfData / static_cast<double>(m_points.size());

        return mean;
    }

    std::optional<double> Group::StandardDeviation()
    {
        if (m_points.empty())
        {
            return {};
        }

        if (not m_mean.has_value())
        {
            return {};
        }

        std::vector<LabeledDistance> distances;

        std::ranges::transform(m_points,
                               std::back_inserter(distances),
                               [&](const auto &dataPoint)
                               { return m_mean.value().EuclideanDistance(dataPoint); });

        double squareSum = 0.0;

        std::ranges::for_each(distances, [&squareSum](const auto distance)
                              { squareSum += std::pow(distance.Distance, 2); });

        const auto standardDeviation = std::sqrt(squareSum);

        return standardDeviation;
    }

    std::optional<Data> Group::GetMean()
    {
        return m_mean;
    }
    std::optional<double> Group::GetStandardDeviation()
    {
        return m_standardDeviation;
    }
}