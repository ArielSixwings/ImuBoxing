#include "SpaceSensor.h"

#include <sstream>

namespace telemetry
{
    std::string SpaceSensor::CreateImuCommand(int logicalId,
                                        int commandNumber,
                                        const std::vector<int>& arguments ) 
    {

        std::ostringstream command;

        command << ">" << logicalId << "," << commandNumber;

        if (not arguments.empty()) {
            
            command << ",";

            for (size_t i = 0; i < arguments.size(); ++i) {
                command << arguments[i];
                if (i < arguments.size() - 1) {
                    command << ",";
                }
            }
        }

        command << "\n";
        return command.str();
    }
}