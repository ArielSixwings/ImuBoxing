#pragma once

#include <string>
#include <vector>

namespace telemetry
{
    class SpaceSensor
    {
    public:
        enum Commands
        {
            StartStreaming = 85,
            StopStreaming = 86,
            TareWithCurrentOrientation = 96,
            TareWithQuaternion = 97
        };

        static std::string CreateImuCommand( int logicalId,
                                            int commandNumber,
                                            const std::vector<int>& arguments = {}); 
    };
}

