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

        if (buffer.empty() or buffer.size() < 15)
        {
            return {};
        }

        if (buffer[0] != 0x01)
        {
            return {};
        }

        std::vector<float> angles;

        const auto headerSize = 2 * sizeof(int8_t);

        float pitch, yaw, roll;
        std::memcpy(&pitch, buffer.data() + headerSize, sizeof(float));
        std::memcpy(&yaw, buffer.data() + headerSize + sizeof(float), sizeof(float));
        std::memcpy(&roll, buffer.data() + headerSize + (2 * sizeof(float)), sizeof(float));

        angles.push_back(pitch);
        angles.push_back(yaw);
        angles.push_back(roll);

        return angles;
    }
}