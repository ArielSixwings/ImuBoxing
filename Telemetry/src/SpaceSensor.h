#pragma once

#include <string>
#include <vector>

namespace telemetry
{
    class SpaceSensor
    {

    public:
        static std::string CreateImuCommand( int logicalId,
                                            int commandNumber,
                                            const std::vector<int>& arguments = {}); 
    };
}

