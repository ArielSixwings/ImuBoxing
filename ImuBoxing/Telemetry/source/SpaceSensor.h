#pragma once

#include <string>
#include <vector>
#include <cstddef>
#include <array>
#include <numeric>

namespace telemetry
{
    class SpaceSensor
    {
    public:
        enum Commands
        {
            SetEulerAngleDecompositionOrder = 0x10,
            OffsetWithCurrentOrientation = 0x13,
            SetBaseOffsetWithCurrentOrientation = 0x16,
            SetStreamingSlots = 0x50,
            SetStreamingTiming = 0x52,
            StartStreaming = 0x55,
            StopStreaming = 0x56,
            TareWithCurrentOrientation = 0x60,
            TareWithQuaternion = 0x61,
            SetCompassEnabled = 0x6D
        };

        enum StreamingCommand
        {
            ReadTaredOrientationAsQuaternion = 0x00,
            ReadTaredOrientationAsEulerAngles = 0x01,
            ReadTaredOrientationAsRotationMatrix = 0x02,
            ReadTaredOrientationAsAxisAngle = 0x03,
            ReadTaredOrientationAsTwoVector = 0x04,
            ReadDifferenceQuaternion = 0x05,
            ReadUntaredOrientationAsQuaternion = 0x06,
            ReadUntaredOrientationAsEulerAngles = 0x07,
            ReadUntaredOrientationAsRotationMatrix = 0x08,
            ReadUntaredOrientationAsAxisAngle = 0x09,
            ReadUntaredOrientationAsTwoVector = 0x0A,
            ReadTaredTwoVectorInSensorFrame = 0x0B,
            ReadUntaredTwoVectorInSensorFrame = 0x0C,
            ReadAllNormalizedComponentSensorData = 0x20,
            ReadNormalizedGyroscopeVector = 0x21,
            ReadNormalizedAccelerometerVector = 0x22,
            ReadNormalizedCompassVector = 0x23,
            ReadAllCorrectedComponentSensorData = 0x25,
            ReadCorrectedGyroscopeVector = 0x26,
            ReadCorrectedAccelerometerVector = 0x27,
            ReadCorrectedCompassVector = 0x28,
            ReadCorrectedLinearAcceleration = 0x29,
            ReadTemperatureC = 0x2B,
            ReadTemperatureF = 0x2C,
            ReadConfidenceFactor = 0x2D,
            ReadAllRawComponentSensorData = 0x40,
            ReadRawGyroscopeVector = 0x41,
            ReadRawAccelerometerVector = 0x42,
            ReadRawCompassVector = 0x43,
            ReadBatteryVoltage = 0xC9,
            ReadBatteryPercentage = 0xCA,
            ReadBatteryStatus = 0xCB,
            ReadButtonState = 0xFA,
            NoCommand = 0xFF
        };

        enum ValidateMode
        {
            Simple = 0xf8,
            PrependResponseHeader = 0xfa
        };

        struct BinaryCommand
        {
            uint8_t StartOfPacket = ValidateMode::Simple;
            uint8_t LogicalId = 0x03; // Imu available for use.
            uint8_t Command;
            std::vector<uint8_t> CommandData;
            uint8_t CheckSum;

            BinaryCommand(uint8_t startOfPacket,
                          uint8_t logicalId,
                          uint8_t command,
                          std::vector<uint8_t> commandData = {})
                : StartOfPacket(startOfPacket),
                  LogicalId(logicalId),
                  Command(command),
                  CommandData(commandData)
            {
                CheckSum = Command + std::accumulate(commandData.begin(), commandData.end(), 0);
            }

            std::vector<uint8_t> Get();

            bool operator==(const BinaryCommand &other) const = default;
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
