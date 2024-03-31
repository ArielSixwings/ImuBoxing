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
            OffsetWithCurrentOrientation = 19,
            SetBaseOffsetWithCurrentOrientation = 22,
            SetStreamingSlots = 80,
            SetStreamingTiming = 82,
            StartStreaming = 85,
            StopStreaming = 86,
            TareWithCurrentOrientation = 96,
            TareWithQuaternion = 97
        };

        static std::string CreateImuCommand(int logicalId,
                                            int commandNumber,
                                            const std::vector<int> &arguments = {});
    };
}
