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
            SetEulerAngleDecompositionOrder = 16,
            OffsetWithCurrentOrientation = 19,
            SetBaseOffsetWithCurrentOrientation = 22,
            SetStreamingSlots = 80,
            SetStreamingTiming = 82,
            StartStreaming = 85,
            StopStreaming = 86,
            TareWithCurrentOrientation = 96,
            TareWithQuaternion = 97,
            SetCompassEnabled = 109
        };

        enum StreamingCommand
        {
            ReadTaredOrientationAsQuaternion = 0,
            ReadTaredOrientationAsEulerAngles = 1,
            ReadTaredOrientationAsRotationMatrix = 2,
            ReadTaredOrientationAsAxisAngle = 3,
            ReadTaredOrientationAsTwoVector = 4,
            ReadDifferenceQuaternion = 5,
            ReadUntaredOrientationAsQuaternion = 6,
            ReadUntaredOrientationAsEulerAngles = 7,
            ReadUntaredOrientationAsRotationMatrix = 8,
            ReadUntaredOrientationAsAxisAngle = 9,
            ReadUntaredOrientationAsTwoVector = 10,
            ReadTaredTwoVectorInSensorFrame = 11,
            ReadUntaredTwoVectorInSensorFrame = 12,
            ReadAllNormalizedComponentSensorData = 32,
            ReadNormalizedGyroscopeVector = 33,
            ReadNormalizedAccelerometerVector = 34,
            ReadNormalizedCompassVector = 35,
            ReadAllCorrectedComponentSensorData = 37,
            ReadCorrectedGyroscopeVector = 38,
            ReadCorrectedAccelerometerVector = 39,
            ReadCorrectedCompassVector = 40,
            ReadCorrectedLinearAcceleration = 41,
            ReadTemperatureC = 43,
            ReadTemperatureF = 44,
            ReadConfidenceFactor = 45,
            ReadAllRawComponentSensorData = 64,
            ReadRawGyroscopeVector = 65,
            ReadRawAccelerometerVector = 66,
            ReadRawCompassVector = 67,
            ReadBatteryVoltage = 201,
            ReadBatteryPercentage = 202,
            ReadBatteryStatus = 203,
            ReadButtonState = 250,
            NoCommand = 255
        };

        struct EulerAngle
        {
            int8_t expectedFirstByte;

            int8_t StartOfPacket;
            int8_t Command;
            float Pitch;
            float Yaw;
            float Roll;
            int8_t Checksum;

            EulerAngle(int8_t expectedFirstByte)
                : StartOfPacket(expectedFirstByte)
            {
            }

            EulerAngle &operator=(const EulerAngle &other) = default;
            bool operator==(const EulerAngle &other) const = default;

            static constexpr size_t SizeInBytes()
            {
                return sizeof(StartOfPacket) + sizeof(Command) + sizeof(Pitch) + sizeof(Yaw) + sizeof(Roll) + sizeof(Checksum);
            }

            std::vector<float> Parse(std::vector<char> &buffer);
        };

        static std::string CreateImuCommand(int logicalId,
                                            int commandNumber,
                                            const std::vector<int> &arguments = {});
    };
}
