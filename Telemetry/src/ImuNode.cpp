#include "ImuNode.h"

#include "SpaceSensor.h"

#include <sstream>
#include <thread>

namespace telemetry
{
    ImuNode::ImuNode() : Node("imuNode"), m_io() 
    {
        try {
            m_serialPort = std::make_shared<boost::asio::serial_port>(m_io, "/dev/ttyACM0");
        } catch (const boost::system::system_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
            throw;
        }

        m_startService = this->create_service<std_srvs::srv::Empty>(
            "imu/start", std::bind(&ImuNode::StartStreaming, this, std::placeholders::_1, std::placeholders::_2));
        
        m_stopService = this->create_service<std_srvs::srv::Empty>(
            "imu/stop", std::bind(&ImuNode::StopStreaming, this, std::placeholders::_1, std::placeholders::_2));

        m_serviceTare = this->create_service<std_srvs::srv::Empty>(
            "imu/tare", std::bind(&ImuNode::TareSensor, this, std::placeholders::_1, std::placeholders::_2));

        m_serviceTareQuaternion = this->create_service<std_srvs::srv::Empty>(
            "imu/tareQuaternion", std::bind(&ImuNode::TareSensorQuaternion, this, std::placeholders::_1, std::placeholders::_2));

        m_serviceOffset = this->create_service<std_srvs::srv::Empty>(
            "imu/offset", std::bind(&ImuNode::OffsetWithCurrentOrientation, this, std::placeholders::_1, std::placeholders::_2));

        m_serviceBaseOffset = this->create_service<std_srvs::srv::Empty>(
            "imu/baseOffset", std::bind(&ImuNode::SetBaseOffsetWithCurrentOrientation, this, std::placeholders::_1, std::placeholders::_2));

        auto timerCallback = [this]() -> void {
            // Placeholder for timer callback functionality
        };
        
        m_timer = this->create_wall_timer(std::chrono::milliseconds(5), timerCallback);
    }

    void ImuNode::CreateAnglesPublisher()
    {
        m_publisher = this->create_publisher<geometry_msgs::msg::Vector3>("imu/angles", 1);
    }

    void ImuNode::StartStreaming([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Response> response)  
    {
        if (m_streaming) {
            RCLCPP_WARN(this->get_logger(), "Already streaming");

            return;
        }

        RCLCPP_INFO(this->get_logger(), "Start Streaming");

        std::vector<int> imuNumbers = {3};

        for (auto id : imuNumbers) {
            auto command = SpaceSensor::CreateImuCommand(id, SpaceSensor::Commands::StartStreaming);
            ApplyCommand(command);
        }

        m_streaming = true;
    }

    void ImuNode::StopStreaming([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Response> response)  
    {
        if (not m_streaming) {
            RCLCPP_WARN(this->get_logger(), "Already not streaming");

            return;
        }

        RCLCPP_INFO(this->get_logger(), "Stop Streaming");

        std::vector<int> imuNumbers = {3};

        for (auto id : imuNumbers) {
            auto command = SpaceSensor::CreateImuCommand(id, SpaceSensor::Commands::StopStreaming);
            ApplyCommand(command);
        }

        m_streaming = false;
    }

    void ImuNode::TareSensor([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Response> response) 
    {
        
        RCLCPP_INFO(this->get_logger(), "Tare Sensor");

        std::vector<int> imuNumbers = {3};

        for (auto id : imuNumbers) {
            auto command = SpaceSensor::CreateImuCommand(id, SpaceSensor::Commands::TareWithCurrentOrientation);
            ApplyCommand(command);
        }

    }

    void ImuNode::TareSensorQuaternion([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                       [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Response> response) 
    {
        
        RCLCPP_INFO(this->get_logger(), "Tare Sensor Quaternion");

        std::vector<int> imuNumbers = {3};

        for (auto id : imuNumbers) {
            auto command = SpaceSensor::CreateImuCommand(id, SpaceSensor::Commands::TareWithQuaternion);
            ApplyCommand(command);
        }

    }

    void ImuNode::OffsetWithCurrentOrientation([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                               [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Response> response) 
    {
        
        RCLCPP_INFO(this->get_logger(), "Offset with current orientation");

        std::vector<int> imuNumbers = {3};

        for (auto id : imuNumbers) {
            auto command = SpaceSensor::CreateImuCommand(id, SpaceSensor::Commands::OffsetWithCurrentOrientation);
            ApplyCommand(command);
        }

    }

    void ImuNode::SetBaseOffsetWithCurrentOrientation([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                                      [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Response> response) 
    {
        
        RCLCPP_INFO(this->get_logger(), "Offset with current orientation");

        std::vector<int> imuNumbers = {3};

        for (auto id : imuNumbers) {
            auto command = SpaceSensor::CreateImuCommand(id, SpaceSensor::Commands::SetBaseOffsetWithCurrentOrientation);
            ApplyCommand(command);
        }

    }

    void ImuNode::ApplyCommand(const std::string& command, bool showResponse) 
    {
        m_serialPort->write_some(boost::asio::buffer(command));

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (showResponse) 
        {
            std::vector<char> response_buffer(128);

            size_t bytes_read = m_serialPort->read_some(boost::asio::buffer(response_buffer));

            if (bytes_read > 0) 
            {
                std::string response(response_buffer.begin(), response_buffer.begin() + bytes_read);
                RCLCPP_INFO(this->get_logger(), ">> %s", response.c_str());
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}