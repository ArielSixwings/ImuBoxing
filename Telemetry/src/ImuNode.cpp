#include "ImuNode.h"

ImuNode::ImuNode() : Node("imuNode"), m_io() 
{
    try {
        m_serialPort = std::make_shared<boost::asio::serial_port>(m_io, "/dev/ttyACM0");
    } catch (const boost::system::system_error& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
        throw;
    }

    auto timerCallback = [this]() -> void {
        // Placeholder for timer callback functionality
    };
    
    m_timer = this->create_wall_timer(std::chrono::milliseconds(5), timerCallback);
}

void ImuNode::CreateAnglesPublisher()
{
    m_publisher = this->create_publisher<geometry_msgs::msg::Vector3>("imu/angles", 1);
}
