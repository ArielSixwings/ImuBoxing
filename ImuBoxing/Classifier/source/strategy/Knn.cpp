#include "Knn.h"

#include <algorithm>
#include <iterator>
#include <ranges>
#include <cmath>

namespace classifier::strategy
{
    Knn::LabeledDistance Knn::Data::EuclideanDistance(const Data &other)
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

    void Knn::AddData(const std::vector<Data> &data)
    {
        m_data.insert(m_data.end(), data.begin(), data.end());
    }

    std::vector<Knn::Data> Knn::GetData()
    {
        return m_data;
    }

    std::vector<Knn::LabeledDistance> Knn::GetNeighbors()
    {
        return m_neighbors;
    }

    Knn::Data Knn::Classify(Data &data)
    {

        std::ranges::transform(m_data,
                               std::back_inserter(m_neighbors),
                               [&data](const auto &dataPoint)
                               { return data.EuclideanDistance(dataPoint); });

        std::ranges::sort(m_neighbors, [](const LabeledDistance &a, const LabeledDistance &b)
                          { return a.Distance < b.Distance; });

        std::vector<LabeledDistance> candidates;

        candidates.insert(candidates.begin(),
                          m_neighbors.begin(),
                          std::next(m_neighbors.begin(), std::min(m_neighbors.size(), static_cast<size_t>(m_k))));

        std::vector<uint8_t> labels(m_k);

        std::ranges::transform(candidates, labels.begin(),
                               [](const LabeledDistance &neighbor)
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

std::ostream &operator<<(std::ostream &os, const classifier::strategy::Knn::Data &data)
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

std::ostream &operator<<(std::ostream &os, const classifier::strategy::Knn::LabeledDistance &other)
{
    os << "LabeledDistance { Distance: " << other.Distance << ", Label: " << static_cast<int>(other.Label) << " }";
    return os;
}