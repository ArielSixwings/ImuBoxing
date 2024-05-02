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
        static void PrintProgressBar(double percentage);

        static std::vector<classes::Data> ReadCSV(const std::string &filename,
                                                  const int label);
    };

}