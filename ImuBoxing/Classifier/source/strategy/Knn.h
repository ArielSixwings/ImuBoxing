#pragma once

#include <vector>
#include <cstdint>
#include <iostream>

namespace classifier::strategy
{
    class Knn
    {
    public:
        struct LabeledDistance
        {
            LabeledDistance(const double distance, const uint8_t label)
                : Distance(distance), Label(label)
            {
            }

            bool operator<(const LabeledDistance &other) const
            {
                return Distance < other.Distance;
            }

            bool operator==(const LabeledDistance &other) const = default;

            double Distance;
            uint8_t Label;
        };

        struct Data
        {
            std::vector<double> Features;
            uint8_t Label;

            Data(const std::vector<double> &features,
                 const uint8_t label) : Features(features),
                                        Label(label) {}

            bool operator==(const Data &other) const = default;
            LabeledDistance EuclideanDistance(const Data &other);
        };

        Knn(const uint8_t k) : m_k(k) {}

        void AddData(const std::vector<Data> &data);

        std::vector<Data> GetData();
        std::vector<LabeledDistance> GetNeighbors();

        Data Classify(Data &data);

    private:
        uint8_t m_k;

        std::vector<LabeledDistance> m_neighbors;
        std::vector<Data> m_data;
    };

}

std::ostream &operator<<(std::ostream &os, const classifier::strategy::Knn::Data &data);

std::ostream &operator<<(std::ostream &os, const classifier::strategy::Knn::LabeledDistance &other);