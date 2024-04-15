#include "SpaceSensor.h"

#include <algorithm>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace telemetry;

SCENARIO("Should create a valid command.", "[Unit][SpaceSensor][BinaryCommand]")
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

SCENARIO("Should parse a response data to an Euler Angle", "[Unit][SpaceSensor]")
{
  GIVEN("A vector of bytes representing angles 90.0, 45.0, 10.10")
  {
    const std::vector<uint8_t> responseData = {0x00, 0x00, 0xb4, 0x42, 0x00, 0x00, 0x34, 0x42, 0x9a, 0x99, 0x21, 0x41};

    WHEN("ParseEulerAngle is called")
    {
      const auto angles = SpaceSensor::ParseEulerAngle(responseData);

      THEN("Resulting angles is a std::vector<float> with 90.0, 45.0, 10.10")
      {
        const std::vector<float> anglesCompare = {90.0, 45.0, 10.10};

        CHECK(angles == anglesCompare);
      }
    }
  }

  GIVEN("A vector of bytes representing angles 1.5708, 0.7854, 0.17628 in rad")
  {
    const std::vector<uint8_t> responseData = {0xdb, 0x0f, 0xc9, 0x3f, 0xdb, 0x0f, 0x49, 0x3f, 0x49, 0x82, 0x34, 0x3e};

    WHEN("ParseEulerAngle is called")
    {
      auto angles = SpaceSensor::ParseEulerAngle(responseData);

      THEN("Resulting angles is a std::vector<float> with 1.5708, 0.7854, 0.17628")
      {
        const std::vector<float> anglesCompare = {1.5708, 0.7854, 0.17628};

        CHECK(std::ranges::equal(angles, anglesCompare, [](float angle, float angleCompare)
                                 { return angle == Catch::Approx(angleCompare); }));
      }
    }
  }
}

SCENARIO("Should validate a BinaryResponse", "[Unit][SpaceSensor][BinaryResponse]")
{
  GIVEN("A buffer of bytes representing angles 90.0, 45.0, 10.10 with headers")
  {
    const std::vector<uint8_t> buffer = {0x00, 0x03, 0x0C, 0x00, 0x00, 0xb4, 0x42, 0x00, 0x00, 0x34, 0x42, 0x9a, 0x99, 0x21, 0x41};

    WHEN("A BinaryResponse is constructed with the buffer and EulerAngle size")
    {
      SpaceSensor::BinaryResponse binaryResponse(buffer, SpaceSensor::ResponsesSizes::EulerAngle);

      THEN("BinaryResponse is valid")
      {

        CHECK(binaryResponse.IsValid());

        AND_THEN("Resulting BinaryResponse has expected header")
        {
          CHECK(binaryResponse.Status == 0x00);
          CHECK(binaryResponse.LogicalId == 0x03);
          CHECK(binaryResponse.DataLength == 0x0C);

          AND_THEN("ResponseData of BinaryResponse has expected size and value")
          {
            CHECK(binaryResponse.ResponseData.size() == static_cast<size_t>(binaryResponse.DataLength));

            const std::vector<uint8_t> responseDataCompare = {0x00, 0x00, 0xb4, 0x42, 0x00, 0x00, 0x34, 0x42, 0x9a, 0x99, 0x21, 0x41};

            CHECK(binaryResponse.ResponseData == responseDataCompare);
          }
        }
      }
    }
  }
}