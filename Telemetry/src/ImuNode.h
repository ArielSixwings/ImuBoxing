#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_srvs/srv/empty.hpp"
#include <boost/asio.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

class ImuNode : public rclcpp::Node {
public:
    ImuNode();

private:
    int imuNumber = 3;
    bool streaming = false;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr m_publisher;
    std::shared_ptr<boost::asio::serial_port> serialPort;
    boost::asio::io_service io; // Declare io object
    rclcpp::TimerBase::SharedPtr m_timer;
};
