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

    classes::Data KMeans::Classify(classes::Data &data)
    {

        std::ranges::transform(m_groups,
                               std::back_inserter(m_meanDistances),
                               [&data](auto &group)
                               { return data.EuclideanDistance(group.GetMean().value()); });

        std::ranges::sort(m_meanDistances, [](const classes::LabeledDistance &a, const classes::LabeledDistance &b)
                          { return a.Distance < b.Distance; });

        data.Label = m_meanDistances.front().Label;
        return data;
    }

}