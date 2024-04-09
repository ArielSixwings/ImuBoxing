#include "SpaceSensor.h"

#include <sstream>
#include <cstring>

namespace telemetry
{
    std::string SpaceSensor::CreateImuCommand(int logicalId,
                                              int commandNumber,
                                              const std::vector<int> &arguments)
    {
        std::ostringstream command;

        command << ">" << logicalId << "," << commandNumber;

        if (not arguments.empty())
        {
            command << ",";

            for (auto argument : arguments)
            {
                command << argument;
            }

            command << ",";
        }

        command << "\n";
        return command.str();
    }

    std::vector<float> SpaceSensor::EulerAngle::Parse(std::vector<char> &buffer)
    {

        buffer.erase(std::remove(buffer.begin(), buffer.end(), ' '), buffer.end());
        buffer.erase(std::remove_if(buffer.begin(), buffer.end(),
                                    [](char c)
                                    { return not isalnum(c) and c != '.' and c != ','; }),
                     buffer.end());

        std::vector<float> angles;
        std::string str(buffer.begin(), buffer.end());
        size_t pos = 0;

        while ((pos = str.find(',')) != std::string::npos)
        {
            angles.push_back(std::stof(str.substr(0, pos)));
            str.erase(0, pos + 1);
        }

        return angles;
    }
}