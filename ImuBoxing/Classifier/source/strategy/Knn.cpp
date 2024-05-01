#include "Knn.h"

#include <algorithm>
#include <iterator>
#include <ranges>
#include <cmath>

namespace classifier::strategy
{

    void Knn::AddData(const std::vector<classes::Data> &data)
    {
        m_data.insert(m_data.end(), data.begin(), data.end());
    }

    std::vector<classes::Data> Knn::GetData()
    {
        return m_data;
    }

    std::vector<classes::LabeledDistance> Knn::GetNeighbors()
    {
        return m_neighbors;
    }

    classes::Data Knn::Classify(classes::Data &data)
    {

        std::ranges::transform(m_data,
                               std::back_inserter(m_neighbors),
                               [&data](const auto &dataPoint)
                               { return data.EuclideanDistance(dataPoint); });

        std::ranges::sort(m_neighbors, [](const classes::LabeledDistance &a, const classes::LabeledDistance &b)
                          { return a.Distance < b.Distance; });

        std::vector<classes::LabeledDistance> candidates;

        candidates.insert(candidates.begin(),
                          m_neighbors.begin(),
                          std::next(m_neighbors.begin(), std::min(m_neighbors.size(), static_cast<size_t>(m_k))));

        std::vector<uint8_t> labels(m_k);

        std::ranges::transform(candidates, labels.begin(),
                               [](const classes::LabeledDistance &neighbor)
                               { return neighbor.Label; });

        std::unordered_map<uint8_t, size_t> labelCounts;

        for (uint8_t label : labels)
        {
            labelCounts[label]++;
        }

        uint8_t mostFrequentLabel = labels[0];

        size_t maxCount = labelCounts[mostFrequentLabel];

        for (const auto &[label, count] : labelCounts)
        {
            if (count > maxCount)
            {
                maxCount = count;
                mostFrequentLabel = label;
            }
        }

        data.Label = mostFrequentLabel;
        return data;
    }

}