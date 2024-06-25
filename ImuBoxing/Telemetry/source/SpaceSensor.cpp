#include "SpaceSensor.h"

#include <algorithm>
#include <cstring>
#include <iomanip>
#include <ranges>
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

        if (LogicalId != 0x03)
        {
            return;
        }

        std::memcpy(&DataLength, buffer.data() + 2 * sizeof(uint8_t), sizeof(uint8_t));

        if (DataLength != responseSize)
        {
            return;
        }

        std::ranges::copy(buffer.begin() + (ResponsesSizes::Header * sizeof(uint8_t)), buffer.end(),
                          std::back_inserter(ResponseData));

        m_isValid = true;
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

        angles = Parser(responseData, ResponsesSizes::EulerAngle);

        return angles;
    }

    std::vector<float> SpaceSensor::ParseQuaternion(const std::vector<uint8_t> &responseData)
    {
        std::vector<float> angles;

        if (responseData.size() < ResponsesSizes::Quaternion)
        {
            return angles;
        }

        angles = Parser(responseData, ResponsesSizes::Quaternion);

        return angles;
    }

    std::vector<float> SpaceSensor::Parser(const std::vector<uint8_t> &responseData, const uint8_t dataSize)
    {
        std::vector<float> angles;
        const auto steps = dataSize / 4;

        std::ranges::for_each(std::views::iota(0, steps),
                              [&angles, &responseData](const auto step)
                              {
                                  float data;

                                  std::memcpy(&data, responseData.data() + (step * sizeof(float)), sizeof(float));
                                  std::reverse(reinterpret_cast<uint8_t *>(&data), reinterpret_cast<uint8_t *>(&data) + sizeof(float));

                                  angles.push_back(data);
                              });

        return angles;
    }
}
