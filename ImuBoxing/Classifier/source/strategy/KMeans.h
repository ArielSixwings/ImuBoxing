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
        struct Candidate
        {
            Candidate(const classes::LabeledDistance &point, double threshold)
                : Point(point), Threshold(threshold) {}

            classes::LabeledDistance Point;
            double Threshold;
        };

        KMeans() {}

        void AddGroup(const classes::Group &group);
        void AddGroup(const std::vector<classes::Data> &points);

        classes::Data Classify(classes::Data &data);

        double GetLastMinimumDistance();

    private:
        double m_lastMinimumDistance;
        std::vector<classes::Group> m_groups;
    };

}