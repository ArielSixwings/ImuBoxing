#pragma once

#include "classes/Data.h"

#include <vector>
#include <cstdint>
#include <iostream>
#include <optional>

namespace classifier
{
    class Utils
    {
    public:
        static std::optional<classes::Data> Mean(const std::vector<classes::Data> &data);

        // static Double StandardDeviation(const classes::Data &mean, const std::vector<classes::Data> &data);

        static void PrintProgressBar(double percentage);

        static std::vector<classes::Data> ReadCSV(const std::string &filename,
                                                  const int label);
    };

}