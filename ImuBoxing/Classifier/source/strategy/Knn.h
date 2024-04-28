#pragma once

#include "DataDefinition.h"

#include <vector>
#include <cstdint>
#include <iostream>

namespace classifier::strategy
{
    class Knn
    {
    public:
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