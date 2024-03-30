#include "ImuNode.h"

ImuNode::ImuNode() : Node("imuNode"), io() {
    try {
        serialPort = std::make_shared<boost::asio::serial_port>(io, "/dev/ttyACM0");
    } catch (const boost::system::system_error& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
        throw;
    }

    m_publisher = this->create_publisher<geometry_msgs::msg::Vector3>("imu/angles", 1);

    auto timerCallback = [this]() -> void {
        // Placeholder for timer callback functionality
    };
    
    m_timer = this->create_wall_timer(std::chrono::milliseconds(5), timerCallback);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<ImuNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("ImuNode"), "Exception thrown: %s", e.what());
    } catch (...) {
        RCLCPP_ERROR(rclcpp::get_logger("ImuNode"), "Unknown exception thrown");
    }
    rclcpp::shutdown();

    return 0;
}
