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

    std::vector<classes::Data> Utils::ReadCSV(const std::string &filename, const int label)
    {
        std::ifstream file(filename);
        std::vector<classes::Data> data;

        if (not file.is_open())
        {
            std::cerr << "Error: Could not open file: " << filename << std::endl;
            return data;
        }

        std::string line;

        for (size_t i = 0; std::getline(file, line); i++)
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

            classes::Data feature(row, label);

            data.push_back(feature);
        }

        file.close();
        return data;
    }

}