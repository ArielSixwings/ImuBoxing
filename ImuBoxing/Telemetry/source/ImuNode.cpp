#include "ImuNode.h"

#include <sstream>
#include <thread>
#include <ranges>
#include <sys/ioctl.h>

namespace telemetry
{
    ImuNode::ImuNode() : Node("imuNode"), m_io()
    {
        try
        {
            m_serialPort = std::make_shared<boost::asio::serial_port>(m_io, "/dev/ttyACM0");
        }
        catch (const boost::system::system_error &e)
        {
            RCLCPP_ERROR(get_logger(), "Failed to open serial port: %s", e.what());
            throw;
        }

        m_serialPort->set_option(boost::asio::serial_port_base::baud_rate(115200));

        m_publisher = create_publisher<geometry_msgs::msg::Vector3>("imu/Angles", 1);

        m_startService = create_service<std_srvs::srv::Empty>(
            "imu/start", std::bind(&ImuNode::StartStreaming, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default);

        m_stopService = create_service<std_srvs::srv::Empty>(
            "imu/stop", std::bind(&ImuNode::StopStreaming, this, std::placeholders::_1, std::placeholders::_2));

        m_serviceTare = create_service<std_srvs::srv::Empty>(
            "imu/tare", std::bind(&ImuNode::TareSensor, this, std::placeholders::_1, std::placeholders::_2));

        m_serviceTareQuaternion = create_service<std_srvs::srv::Empty>(
            "imu/tareQuaternion", std::bind(&ImuNode::TareSensorQuaternion, this, std::placeholders::_1, std::placeholders::_2));

        m_serviceOffset = create_service<std_srvs::srv::Empty>(
            "imu/offset", std::bind(&ImuNode::OffsetWithCurrentOrientation, this, std::placeholders::_1, std::placeholders::_2));

        m_serviceBaseOffset = create_service<std_srvs::srv::Empty>(
            "imu/baseOffset", std::bind(&ImuNode::SetBaseOffsetWithCurrentOrientation, this, std::placeholders::_1, std::placeholders::_2));

        SetStreamingSlots({SpaceSensor::StreamingCommand::ReadTaredOrientationAsEulerAngles,
                           SpaceSensor::StreamingCommand::NoCommand,
                           SpaceSensor::StreamingCommand::NoCommand,
                           SpaceSensor::StreamingCommand::NoCommand,
                           SpaceSensor::StreamingCommand::NoCommand,
                           SpaceSensor::StreamingCommand::NoCommand,
                           SpaceSensor::StreamingCommand::NoCommand,
                           SpaceSensor::StreamingCommand::NoCommand});

        SetStreamingTiming(5);

        SetCompassEnabledToZero();
        SetEulerAngleDecompositionOrder();

        m_timer = create_wall_timer(std::chrono::milliseconds(5),
                                    std::bind(&ImuNode::LoopCallback, this));
    }

    void ImuNode::ApplyCommand(SpaceSensor::BinaryCommand &binaryCommand, bool showResponse)
    {
        m_serialPort->write_some(boost::asio::buffer(binaryCommand.Get()));

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (showResponse)
        {
            std::vector<uint8_t> responseBuffer(128);

            int bytesRead = m_serialPort->read_some(boost::asio::buffer(responseBuffer));

            if (bytesRead > 0)
            {
                std::ranges::for_each(responseBuffer, [&](const auto byte)
                                      { RCLCPP_INFO(get_logger(), ">> %X", byte); });
            }
        }
    }

    void ImuNode::StartStreaming([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                 [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {
        if (m_streaming)
        {
            RCLCPP_WARN(get_logger(), "Already streaming");

            return;
        }

        RCLCPP_INFO(get_logger(), "Start Streaming");

        SpaceSensor::BinaryCommand binaryCommand(SpaceSensor::ValidateMode::Simple,
                                                 0x03,
                                                 SpaceSensor::Commands::StartStreaming);
        ApplyCommand(binaryCommand);

        m_streaming = true;
    }

    void ImuNode::StopStreaming([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {
        if (not m_streaming)
        {
            RCLCPP_WARN(get_logger(), "Already not streaming");

            return;
        }

        RCLCPP_INFO(get_logger(), "Stop Streaming");

        SpaceSensor::BinaryCommand binaryCommand(SpaceSensor::ValidateMode::Simple,
                                                 0x03,
                                                 SpaceSensor::Commands::StopStreaming);
        ApplyCommand(binaryCommand);

        m_streaming = false;
    }

    void ImuNode::TareSensor([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                             [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Response> response)

    {
        RCLCPP_INFO(get_logger(), "Tare Sensor");

        SpaceSensor::BinaryCommand binaryCommand(SpaceSensor::ValidateMode::Simple,
                                                 0x03,
                                                 SpaceSensor::Commands::TareWithCurrentOrientation);
        ApplyCommand(binaryCommand);
    }

    void ImuNode::TareSensorQuaternion([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                       [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Response> response) // TODO, implement quaternion parameter
    {

        RCLCPP_INFO(get_logger(), "Tare Sensor Quaternion");

        SpaceSensor::BinaryCommand binaryCommand(SpaceSensor::ValidateMode::Simple,
                                                 0x03,
                                                 SpaceSensor::Commands::TareWithQuaternion);

        ApplyCommand(binaryCommand);
    }

    void ImuNode::OffsetWithCurrentOrientation([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                               [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {

        RCLCPP_INFO(get_logger(), "Offset with current orientation");

        SpaceSensor::BinaryCommand binaryCommand(SpaceSensor::ValidateMode::Simple,
                                                 0x03,
                                                 SpaceSensor::Commands::OffsetWithCurrentOrientation);
        ApplyCommand(binaryCommand);
    }

    void ImuNode::SetBaseOffsetWithCurrentOrientation([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                                      [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {

        RCLCPP_INFO(get_logger(), "Offset with current orientation");

        SpaceSensor::BinaryCommand binaryCommand(SpaceSensor::ValidateMode::Simple,
                                                 0x03,
                                                 SpaceSensor::Commands::SetBaseOffsetWithCurrentOrientation);

        ApplyCommand(binaryCommand);
    }

    void ImuNode::SetStreamingSlots(const std::vector<uint8_t> &commandData)
    {

        RCLCPP_INFO(get_logger(), "Set Streaming Slots");

        SpaceSensor::BinaryCommand binaryCommand(SpaceSensor::ValidateMode::Simple,
                                                 0x03,
                                                 SpaceSensor::Commands::SetStreamingSlots,
                                                 commandData);

        ApplyCommand(binaryCommand);
    }

    void ImuNode::SetCompassEnabledToZero()
    {

        RCLCPP_INFO(get_logger(), "Set Compass Enabled to 0");

        SpaceSensor::BinaryCommand binaryCommand(SpaceSensor::ValidateMode::Simple,
                                                 0x03,
                                                 SpaceSensor::Commands::SetCompassEnabled,
                                                 {0x00});
        ApplyCommand(binaryCommand);
    }

    void ImuNode::SetEulerAngleDecompositionOrder()
    {

        RCLCPP_INFO(get_logger(), "Set Euler Angle Decomposition Order");

        SpaceSensor::BinaryCommand binaryCommand(SpaceSensor::ValidateMode::Simple,
                                                 0x03,
                                                 SpaceSensor::Commands::SetEulerAngleDecompositionOrder,
                                                 {0x05});
        ApplyCommand(binaryCommand);
    }

    void ImuNode::SetStreamingTiming([[maybe_unused]] const int frequency)
    {

        RCLCPP_INFO(get_logger(), "Set Streaming timing");

        SpaceSensor::BinaryCommand binaryCommand(SpaceSensor::ValidateMode::Simple,
                                                 0x03,
                                                 SpaceSensor::Commands::SetStreamingTiming,
                                                 {0x00, 0x00, 0x27, 0x10, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00});
        ApplyCommand(binaryCommand);
    }

    void ImuNode::LoopCallback()
    {
        if (not m_streaming)
        {
            RCLCPP_INFO(get_logger(), "Not streaming");
            std::this_thread::sleep_for(std::chrono::seconds(1));
            return;
        }

        int bytesAvailable;

        if (ioctl(m_serialPort->native_handle(), FIONREAD, &bytesAvailable) == -1)
        {
            RCLCPP_ERROR(get_logger(), "native_handle failed");
            return;
        }

        constexpr int eulerAngleBinaryResponseSize = (SpaceSensor::ResponsesSizes::Header + SpaceSensor::ResponsesSizes::EulerAngle);

        if (bytesAvailable < eulerAngleBinaryResponseSize)
        {
            RCLCPP_INFO(get_logger(), "Not enough bytes available");
            return;
        }

        std::vector<uint8_t> responseBuffer(15);

        const auto bytesRead = m_serialPort->read_some(boost::asio::buffer(responseBuffer,
                                                                           eulerAngleBinaryResponseSize));

        if (bytesRead <= 0)
        {
            RCLCPP_INFO(get_logger(), "No bytes read");
            return;
        }

        SpaceSensor::BinaryResponse binaryResponse(responseBuffer, SpaceSensor::ResponsesSizes::EulerAngle);

        if (not binaryResponse.IsValid())
        {

            m_serialPort->read_some(boost::asio::buffer(responseBuffer,
                                                        1));

            return;
        }

        for (size_t i = 0; i < responseBuffer.size(); ++i)
        {
            std::cout << "0x" << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(responseBuffer[i]);
            if (i != responseBuffer.size() - 1)
                std::cout << ", ";
        }

        std::cout << std::endl;

        const auto angles = SpaceSensor::ParseEulerAngle(binaryResponse.ResponseData);

        geometry_msgs::msg::Vector3 message;
        message.x = angles[0];
        message.y = angles[1];
        message.z = angles[2];

        m_publisher->publish(message);
    }
}
