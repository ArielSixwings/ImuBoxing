#include "ImuNode.h"

#include "SpaceSensor.h"

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

        m_publisher = create_publisher<geometry_msgs::msg::Vector3>("imu/angles", 1);

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

        SetStreamingTiming(100);

        SetCompassEnabledToZero();
        SetEulerAngleDecompositionOrder();

        m_timer = create_wall_timer(std::chrono::milliseconds(5),
                                    std::bind(&ImuNode::LoopCallback, this));
    }

    void ImuNode::ApplyCommand(const std::string &command, bool showResponse)
    {
        m_serialPort->write_some(boost::asio::buffer(command));

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (showResponse)
        {
            std::vector<char> responseBuffer(128);

            size_t bytesRead = m_serialPort->read_some(boost::asio::buffer(responseBuffer));

            if (bytesRead > 0)
            {
                std::string response(responseBuffer.begin(), responseBuffer.begin() + bytesRead);
                RCLCPP_INFO(get_logger(), ">> %s", response.c_str());
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
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

        std::vector<int> imuNumbers = {3};

        for (auto id : imuNumbers)
        {
            auto command = SpaceSensor::CreateImuCommand(id, SpaceSensor::Commands::StartStreaming);
            ApplyCommand(command);
        }

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

        std::vector<int> imuNumbers = {3};

        for (auto id : imuNumbers)
        {
            auto command = SpaceSensor::CreateImuCommand(id, SpaceSensor::Commands::StopStreaming);
            ApplyCommand(command);
        }

        m_streaming = false;
    }

    void ImuNode::TareSensor([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                             [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {

        RCLCPP_INFO(get_logger(), "Tare Sensor");

        std::vector<int> imuNumbers = {3};

        for (auto id : imuNumbers)
        {
            auto command = SpaceSensor::CreateImuCommand(id, SpaceSensor::Commands::TareWithCurrentOrientation);
            ApplyCommand(command);
        }
    }

    void ImuNode::TareSensorQuaternion([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                       [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {

        RCLCPP_INFO(get_logger(), "Tare Sensor Quaternion");

        std::vector<int> imuNumbers = {3};

        for (auto id : imuNumbers)
        {
            auto command = SpaceSensor::CreateImuCommand(id, SpaceSensor::Commands::TareWithQuaternion);
            ApplyCommand(command);
        }
    }

    void ImuNode::OffsetWithCurrentOrientation([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                               [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {

        RCLCPP_INFO(get_logger(), "Offset with current orientation");

        std::vector<int> imuNumbers = {3};

        for (auto id : imuNumbers)
        {
            auto command = SpaceSensor::CreateImuCommand(id, SpaceSensor::Commands::OffsetWithCurrentOrientation);
            ApplyCommand(command);
        }
    }

    void ImuNode::SetBaseOffsetWithCurrentOrientation([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                                      [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {

        RCLCPP_INFO(get_logger(), "Offset with current orientation");

        std::vector<int> imuNumbers = {3};

        for (auto id : imuNumbers)
        {
            auto command = SpaceSensor::CreateImuCommand(id, SpaceSensor::Commands::SetBaseOffsetWithCurrentOrientation);
            ApplyCommand(command);
        }
    }

    void ImuNode::SetStreamingSlots(const std::vector<int> &arguments)
    {

        RCLCPP_INFO(get_logger(), "Set Streaming Slots");

        std::vector<int> imuNumbers = {3};

        for (auto id : imuNumbers)
        {
            ApplyCommand(SpaceSensor::CreateImuCommand(id,
                                                       SpaceSensor::Commands::SetStreamingSlots,
                                                       arguments));
        }
    }

    void ImuNode::SetCompassEnabledToZero()
    {

        RCLCPP_INFO(get_logger(), "Set Compass Enabled to 0");

        std::vector<int> imuNumbers = {3};

        for (auto id : imuNumbers)
        {
            ApplyCommand(SpaceSensor::CreateImuCommand(id,
                                                       SpaceSensor::Commands::SetCompassEnabled,
                                                       {0}));
        }
    }

    void ImuNode::SetEulerAngleDecompositionOrder()
    {

        RCLCPP_INFO(get_logger(), "Set Euler Angle Decomposition Order");

        std::vector<int> imuNumbers = {3};

        for (auto id : imuNumbers)
        {
            ApplyCommand(SpaceSensor::CreateImuCommand(id,
                                                       SpaceSensor::Commands::SetEulerAngleDecompositionOrder,
                                                       {5}));
        }
    }

    void ImuNode::SetStreamingTiming(const int frequency)
    {

        RCLCPP_INFO(get_logger(), "Set Streaming timing");

        std::vector<int> imuNumbers = {3};

        const auto usedFrequency = frequency > 0 ? (1000000 / frequency) : 0;

        for (auto id : imuNumbers)
        {
            ApplyCommand(SpaceSensor::CreateImuCommand(id,
                                                       SpaceSensor::Commands::SetStreamingTiming,
                                                       {usedFrequency, -1, 0}));
        }
    }

    bool ImuNode::LoopCallback()
    {
        if (not m_streaming)
        {
            RCLCPP_DEBUG(get_logger(), "Not streaming");
            std::this_thread::sleep_for(std::chrono::seconds(1));
            return false;
        }

        int bytesAvailable;

        if (ioctl(m_serialPort->native_handle(), FIONREAD, &bytesAvailable) == -1)
        {
            RCLCPP_ERROR(get_logger(), "native_handle failed");
            return false;
        }

        if (static_cast<size_t>(bytesAvailable) < SpaceSensor::EulerAngle::SizeInBytes())
        {
            RCLCPP_DEBUG(get_logger(), "No bytes available ");
            return true;
        }

        std::vector<char> responseBuffer(128);

        size_t bytesRead = m_serialPort->read_some(boost::asio::buffer(responseBuffer,
                                                                       bytesAvailable));

        if (bytesRead <= 0)
        {
            RCLCPP_DEBUG(get_logger(), "No bytes read ");
            return true;
        }

        SpaceSensor::EulerAngle eulerAngle(0);

        const auto angles = eulerAngle.Parse(responseBuffer);

        geometry_msgs::msg::Vector3 message;
        message.x = angles[0];
        message.y = angles[1];
        message.z = angles[2];

        m_publisher->publish(message);

        return true;
    }
}
