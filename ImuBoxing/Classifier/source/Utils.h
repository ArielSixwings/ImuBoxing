#pragma once

#include "DataDefinition.h"

#include <vector>
#include <cstdint>
#include <iostream>
#include <optional>

namespace classifier
{
    class Utils
    {
    public:
        static std::optional<Data> Mean(const std::vector<Data> &data);

        static void PrintProgressBar(double percentage);

        static std::vector<Data> ReadCSV(const std::string &filename,
                                         const int label);
    };

}