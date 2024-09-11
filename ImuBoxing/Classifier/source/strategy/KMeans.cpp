#include "KMeans.h"

#include <algorithm>
#include <iterator>
#include <ranges>
#include <cmath>

namespace classifier::strategy
{
    void KMeans::AddGroup(const classes::Group &group)
    {
        m_groups.push_back(group);
    }

    void KMeans::AddGroup(const std::vector<classes::Data> &points)
    {
        classes::Group group(points);
        m_groups.push_back(group);
    }

    float KMeans::GetLastMinimumDistance()
    {
        return m_lastMinimumDistance;
    }

    classes::Data KMeans::Classify(classes::Data &data)
    {

        std::vector<Candidate> candidates;

        std::ranges::transform(m_groups,
                               std::back_inserter(candidates),
                               [&data](auto &group)
                               { return Candidate(data.EuclideanDistance(group.GetMean().value()),
                                                  group.GetStandardDeviation().value()); });

        std::ranges::sort(candidates, [](const Candidate &a, const Candidate &b)
                          { return a.Point.Distance < b.Point.Distance; });

        ((2 * candidates.front().Point.Distance) < candidates.front().Threshold)
            ? data.Label = candidates.front().Point.Label
            : data.Label = classes::Data::Poses::Unknown;

        return data;
    }
}
