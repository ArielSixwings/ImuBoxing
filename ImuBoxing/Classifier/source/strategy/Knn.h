#pragma once

#include <vector>

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

            double Distance;
            uint8_t Label;
        };

        struct Data
        {
            std::vector<double> Features;
            uint8_t Label;

            LabeledDistance EuclideanDistance(const Data &other);
        };

        Knn(const uint8_t k);
        void AddData(const std::vector<Data> &data);

        Data Classify(Data &data);

    private:
        uint8_t m_k;

        std::vector<Data> m_data;
        std::vector<LabeledDistance> m_neighbors;
    };

}