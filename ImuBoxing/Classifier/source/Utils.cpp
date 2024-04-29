#include "Utils.h"

#include <algorithm>
#include <iomanip>
namespace classifier
{

    std::optional<Data> Utils::Mean(const std::vector<Data> &data)
    {
        if (data.empty())
        {
            return {};
        }

        const auto numFeatures = data[0].Features.size();

        auto valid = true;

        std::ranges::for_each(data, [&numFeatures, &valid](const auto point)
                              { if(point.Features.size() != numFeatures) valid = false; });

        const auto label = data[0].Label;

        std::ranges::for_each(data, [&label, &valid](const auto point)
                              { if(point.Label != label) valid = false; });

        if (not valid)
        {
            return {};
        }

        std::vector<double> means(numFeatures, 0.0);

        for (const auto &point : data)
        {
            for (size_t i = 0; i < numFeatures; ++i)
            {
                means[i] += point.Features[i];
            }
        }

        for (size_t i = 0; i < numFeatures; ++i)
        {
            means[i] /= data.size();
        }

        return Data(means, label);
    }

    void Utils::PrintProgressBar(double percentage)
    {
        const int barWidth = 70; // Width of the progress bar in characters

        std::cout << "[";
        int pos = static_cast<int>(barWidth * percentage);
        for (int i = 0; i < barWidth; ++i)
        {
            if (i < pos)
                std::cout << "=";
            else if (i == pos)
                std::cout << ">";
            else
                std::cout << " ";
        }
        std::cout << "] " << int(percentage * 100.0) << " %\r";
        std::cout.flush(); // Important to use when updating the same line
    }
}