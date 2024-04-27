
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"

namespace classifier
{

    class ImuSubscriber : public rclcpp::Node
    {
    public:
        ImuSubscriber();

        std::vector<std::vector<float>> GetMessages();

    private:
        void topic_callback(const geometry_msgs::msg::Vector3::SharedPtr message);

        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr m_subscription;

        std::vector<std::vector<float>> m_messages;
    };
}
