#include "SpaceSensor.h"

#include <algorithm>
#include <cstring>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <iostream>

namespace telemetry
{
    SpaceSensor::BinaryResponse::BinaryResponse(const std::vector<uint8_t> &buffer,
                                                const ResponsesSizes responseSize)
    {
        std::memcpy(&Status, buffer.data(), sizeof(uint8_t));

        if (Status != 0x00)
        {
            return;
        }

        std::memcpy(&LogicalId, buffer.data() + sizeof(uint8_t), sizeof(uint8_t));
        std::memcpy(&DataLength, buffer.data() + 2 * sizeof(uint8_t), sizeof(uint8_t));

        if (DataLength != responseSize)
        {
            return;
        }

        std::ranges::copy(buffer.begin() + (ResponsesSizes::Header * sizeof(uint8_t)), buffer.end(),
                          std::back_inserter(ResponseData));

        m_isValid = true;
    }

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

    std::vector<uint8_t> SpaceSensor::BinaryCommand::Get()
    {
        std::vector<uint8_t> commandBuffer{StartOfPacket, LogicalId, Command};

        std::ranges::copy(CommandData.begin(), CommandData.end(),
                          std::back_inserter(commandBuffer));

        commandBuffer.push_back(CheckSum);

        return commandBuffer;
    }

    std::vector<float> SpaceSensor::ParseEulerAngle(const std::vector<uint8_t> &responseData)
    {

        std::vector<float> angles;

        if (responseData.size() < ResponsesSizes::EulerAngle)
        {
            return angles;
        }

        float roll;
        float pitch;
        float yaw;

        std::memcpy(&roll, responseData.data(), sizeof(float));
        std::memcpy(&pitch, responseData.data() + sizeof(float), sizeof(float));
        std::memcpy(&yaw, responseData.data() + 2 * sizeof(float), sizeof(float));

        angles.push_back(roll);
        angles.push_back(pitch);
        angles.push_back(yaw);

        return angles;
    }
}
