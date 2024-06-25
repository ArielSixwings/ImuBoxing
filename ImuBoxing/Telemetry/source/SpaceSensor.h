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

        enum ResponsesSizes
        {
            Header = 3,
            EulerAngle = 12,
            Quaternion = 16
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
                CheckSum = (LogicalId + Command + std::accumulate(commandData.begin(), commandData.end(), 0)) % 256;
            }

            std::vector<uint8_t> Get();

            bool operator==(const BinaryCommand &other) const = default;
        };

        struct BinaryResponse
        {
            int8_t Status;
            uint8_t LogicalId;
            uint8_t DataLength = 0x00;
            std::vector<uint8_t> ResponseData;

            BinaryResponse(const std::vector<uint8_t> &buffer,
                           const ResponsesSizes responseSize);

            bool operator==(const BinaryResponse &other) const = default;

            bool IsValid() { return m_isValid; }

        private:
            bool m_isValid = false;
        };

        static std::vector<float> ParseEulerAngle(const std::vector<uint8_t> &responseData);
    };
}