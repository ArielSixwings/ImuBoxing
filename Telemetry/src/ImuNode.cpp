#include "ImuNode.h"

#include "SpaceSensor.h"

#include <sstream>
#include <thread>
#include <ranges>

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

        m_publisher = create_publisher<geometry_msgs::msg::Vector3>("imu/angles", 1);

        auto callBackGroup = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

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

        ManualFlush();

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

    void ImuNode::ManualFlush()
    {
        std::vector<char> buffer(128);

        auto bytesRead = m_serialPort->read_some(boost::asio::buffer(buffer));

        for (size_t i = 0; (i < 20 and bytesRead > 0); i++)
        {
            bytesRead = m_serialPort->read_some(boost::asio::buffer(buffer));
        }
        RCLCPP_INFO(get_logger(), "Flushed data");
    }

    bool ImuNode::LoopCallback()
    {
        if (not m_streaming)
        {
            RCLCPP_INFO(get_logger(), "Not streaming");
            std::this_thread::sleep_for(std::chrono::seconds(1));
            return false;
        }

        std::vector<char> responseBuffer(256);

        size_t bytesRead = m_serialPort->read_some(boost::asio::buffer(responseBuffer));

        if (bytesRead <= 0)
        {
            RCLCPP_INFO(get_logger(), "No bytes to read");
            return true;
        }

        std::string data(responseBuffer.begin(), responseBuffer.begin() + bytesRead);

        if (data.empty())
        {
            RCLCPP_ERROR(get_logger(), "Data is empty");
            return false;
        }

        // if (static_cast<unsigned char>(data[0]) != 0)
        // {
        //     RCLCPP_ERROR(get_logger(), "Data acquired is invalid: %s", data.c_str());

        //     std::this_thread::sleep_for(std::chrono::milliseconds(500));
        //     return false;
        // }

        if (data.length() <= 3)
        {
            RCLCPP_ERROR(get_logger(), "Data size is invalid");
            return false;
        }

        std::replace(data.begin(), data.end(), '\r', ' ');
        std::replace(data.begin(), data.end(), '\n', ' ');

        std::istringstream iss(data);
        std::vector<std::string> dataList((std::istream_iterator<std::string>(iss)),
                                          std::istream_iterator<std::string>());

        if (dataList.empty())
        {
            RCLCPP_ERROR(get_logger(), "Data List is empty");
            return false;
        }

        auto &lastElement = dataList.back();
        std::vector<std::string> eulerVectorString;

        std::string eulerString = lastElement.substr(3);
        std::istringstream eulerStream(eulerString);
        std::string segment;

        while (std::getline(eulerStream, segment, ','))
        {
            eulerVectorString.push_back(segment);
        }

        if (eulerVectorString.size() < 3)
        {
            RCLCPP_ERROR(get_logger(), "Result vector is too short");
            return false;
        }

        geometry_msgs::msg::Vector3 message;
        message.x = std::stod(eulerVectorString[0]);
        message.y = std::stod(eulerVectorString[1]);
        message.z = std::stod(eulerVectorString[2]);

        m_publisher->publish(message);
        return true;
    }
}
