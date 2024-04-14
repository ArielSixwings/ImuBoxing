#include "SpaceSensor.h"

#include <catch2/catch_test_macros.hpp>

using namespace telemetry;

SCENARIO("Should return the size of the euler angles response", "[Unit][EulerAngle]")
{

  WHEN("EulerAngle::SizeInBytes is called")
  {
    const auto result = SpaceSensor::EulerAngle::SizeInBytes();

    THEN("Result is 15")
    {
      CHECK(result == 15);
    }
  }
}

SCENARIO("Should create a valid command.", "[Unit][BinaryCommand]")
{
  GIVEN("A BinaryCommand created defining StartOfPacket, LogicalId and Command")
  {
    SpaceSensor::BinaryCommand binaryCommand(SpaceSensor::ValidateMode::Simple,
                                             0x03,
                                             SpaceSensor::Commands::StartStreaming);

    WHEN("Get method is called")
    {
      const auto bufferResult = binaryCommand.Get();

      THEN("Resulting vector has defined values and CheckSum equals to the LogicalId + Command provided")
      {
        const std::vector<uint8_t> bufferCompare = {0xf8, 0x03, 0x55, 0x58};

        CHECK(bufferResult == bufferCompare);
      }
    }
  }

  GIVEN("A BinaryCommand created defining StartOfPacket, LogicalId, Command and CommandData")
  {
    SpaceSensor::BinaryCommand binaryCommand(SpaceSensor::ValidateMode::Simple,
                                             0x03,
                                             SpaceSensor::Commands::SetStreamingSlots,
                                             {SpaceSensor::StreamingCommand::ReadTaredOrientationAsEulerAngles,
                                              SpaceSensor::StreamingCommand::NoCommand,
                                              SpaceSensor::StreamingCommand::NoCommand,
                                              SpaceSensor::StreamingCommand::NoCommand,
                                              SpaceSensor::StreamingCommand::NoCommand,
                                              SpaceSensor::StreamingCommand::NoCommand,
                                              SpaceSensor::StreamingCommand::NoCommand,
                                              SpaceSensor::StreamingCommand::NoCommand});

    WHEN("Get method is called")
    {
      const auto bufferResult = binaryCommand.Get();

      THEN("Resulting vector has defined values and CheckSum equals to "
           "(LogicalId + the Command provided + CommandData) modulus 256")
      {
        const std::vector<uint8_t> bufferCompare = {0xf8, 0x03, 0x50, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x4D};

        CHECK(bufferResult == bufferCompare);
      }
    }
  }
}