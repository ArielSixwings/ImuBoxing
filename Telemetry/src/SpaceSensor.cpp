#include "SpaceSensor.h"

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
}