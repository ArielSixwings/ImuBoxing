#pragma once

#include "classes/Data.h"
#include "classes/LabeledDistance.h"

#include <vector>
#include <cstdint>
#include <iostream>

namespace classifier::strategy
{
    class Knn
    {
    public:
        Knn() : m_k(3) {}
        Knn(const uint8_t k) : m_k(k) {}

        void AddData(const std::vector<classes::Data> &data);

        std::vector<classes::Data> GetData();
        std::vector<classes::LabeledDistance> GetNeighbors();

        classes::Data Classify(classes::Data &data);

    private:
        uint8_t m_k;

        std::vector<classes::LabeledDistance> m_neighbors;
        std::vector<classes::Data> m_data;
    };

}