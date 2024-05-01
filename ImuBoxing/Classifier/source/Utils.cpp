#include "Utils.h"

#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <ranges>
#include <sstream>
#include <vector>

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

        Data sumOfData(std::vector<double>(numFeatures, 0.0), label);

        std::ranges::for_each(data, [&sumOfData](const auto point)
                              { sumOfData = sumOfData + point; });

        const auto mean = sumOfData / static_cast<double>(data.size());

        return mean;
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

    std::vector<Data> Utils::ReadCSV(const std::string &filename, const int label)
    {
        std::ifstream file(filename);
        std::vector<Data> data;

        if (not file.is_open())
        {
            std::cerr << "Error: Could not open file: " << filename << std::endl;
            return data;
        }

        std::string line;

        size_t sizeToRead = 100;

        for (size_t i = 0; (i < sizeToRead) and std::getline(file, line); i++)
        {
            std::stringstream lineStream(line);
            std::vector<double> row;
            double value;

            size_t count = 0;

            while (lineStream >> value)
            {
                row.push_back(value);
                count++;

                if (count == 3)
                    break;

                if (lineStream.peek() == ',')
                    lineStream.ignore();
            }

            if (count != 3)
            {
                std::cerr << "Warning: Row does not contain exactly three values. Skipping..." << std::endl;
                continue;
            }

            Data feature(row, label);

            data.push_back(feature);
        }

        for (size_t i = 0; i < data.size(); i++)
        {
            std::cout << data[i] << std::endl;
        }

        file.close();
        return data;
    }

}