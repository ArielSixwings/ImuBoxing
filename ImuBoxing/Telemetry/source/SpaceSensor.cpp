#include "SpaceSensor.h"

#include <algorithm>
#include <cstring>
#include <iomanip>
#include <iterator>
#include <sstream>

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

    std::vector<uint8_t> SpaceSensor::BinaryCommand::Get()
    {
        std::vector<uint8_t> commandBuffer{StartOfPacket, LogicalId, Command};

        std::ranges::copy(CommandData.begin(), CommandData.end(),
                          std::back_inserter(commandBuffer));

        commandBuffer.push_back(CheckSum);

        return commandBuffer;
    }

    std::vector<float> SpaceSensor::EulerAngle::Parse(std::vector<char> &buffer)
    {

        buffer.erase(std::remove(buffer.begin(), buffer.end(), ' '), buffer.end());
        buffer.erase(std::remove_if(buffer.begin(), buffer.end(),
                                    [](char byte)
                                    { return not isalnum(byte) and byte != '.' and byte != ','; }),
                     buffer.end());

        std::vector<float> angles;
        std::string str(buffer.begin(), buffer.end());
        size_t pos = 0;

        while ((pos = str.find(',')) != std::string::npos)
        {
            try
            {
                angles.push_back(std::stof(str.substr(0, pos)));
                str.erase(0, pos + 1);
            }
            catch (const std::exception &e)
            {
                // std::cerr << e.what() << '\n';
            }
        }

        return angles;
    }
}

std::ostream &operator<<(std::ostream &os, const telemetry::SpaceSensor::BinaryCommand &binaryCommand)
{
    os << "BinaryCommand:" << std::endl;
    os << "  StartOfPacket: 0x" << std::hex << std::setfill('0') << std::setw(2) << (int)binaryCommand.StartOfPacket << std::endl;
    os << "  LogicalId: 0x" << std::hex << std::setfill('0') << std::setw(2) << (int)binaryCommand.LogicalId << std::endl;
    os << "  Command: 0x" << std::hex << std::setfill('0') << std::setw(2) << (int)binaryCommand.Command << std::endl;

    if (not binaryCommand.CommandData.empty())
    {
        os << "  CommandData: ";
        for (uint8_t data : binaryCommand.CommandData)
        {
            os << "0x" << std::hex << std::setfill('0') << std::setw(2) << (int)data << " ";
        }
        os << std::endl;
    }

    os << "  Checksum: 0x" << std::hex << std::setfill('0') << std::setw(2) << (int)binaryCommand.CheckSum << std::endl;
    return os;
}