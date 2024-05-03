#pragma once

#include "classes/Group.h"
#include "classes/Data.h"
#include "classes/LabeledDistance.h"

#include <vector>
#include <cstdint>
#include <iostream>

namespace classifier::strategy
{
    class KMeans
    {
    public:
        KMeans() {}

        void AddGroup(const classes::Group &group);
        void AddGroup(const std::vector<classes::Data> &points);

        classes::Data Classify(classes::Data &data);

    private:
        std::vector<classes::Group> m_groups;
        std::vector<classes::LabeledDistance> m_meanDistances;
    };

}