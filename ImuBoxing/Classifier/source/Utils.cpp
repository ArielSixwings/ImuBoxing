#include "Utils.h"

#include "rclcpp/rclcpp.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
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

    std::vector<Data> Utils::ReadCSV(const std::string &filename, const int label)
    {
        std::ifstream file(filename);
        std::vector<Data> data;

        if (not file.is_open())
        {
            std::cerr << "Error: Could not open file: " << filename << std::endl;
            return data; // Return empty data vector if file cannot be opened
        }

        std::string line;

        size_t sizeToRead = 100;

        for (size_t i = 0; (i < sizeToRead) and std::getline(file, line); i++)
        {
            std::stringstream lineStream(line);
            std::vector<double> row;
            double value;

            size_t count = 0; // Counter for the number of values read

            // Read comma-separated values and convert to doubles
            while (lineStream >> value)
            {
                row.push_back(value);
                count++;

                if (count == 3) // Ensure we only read three values per row
                    break;

                if (lineStream.peek() == ',') // Skip the comma
                    lineStream.ignore();
            }

            // If the row doesn't contain exactly three values, skip it
            if (count != 3)
            {
                std::cerr << "Warning: Row does not contain exactly three values. Skipping..." << std::endl;
                continue;
            }

            Data feature(row, label);

            // Add the row of doubles to the data vector
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