#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <boost/asio.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace mock
{
    class Publisher : public rclcpp::Node
    {
    public:
        Publisher(const std::string &topic);

        void LoopCallback();

    private:
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr m_publisher;
        rclcpp::TimerBase::SharedPtr m_timer;
    };
}
