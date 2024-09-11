#pragma once

#include "strategy/Knn.h"
#include "strategy/KMeans.h"

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include "std_msgs/msg/int32.hpp"

namespace classifier
{
    class PoseClassification : public rclcpp::Node
    {
    public:
        PoseClassification();

    private:
        void topicCallback(const std_msgs::msg::Float32MultiArray::SharedPtr message);

        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_subscription;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr m_publisher;

        strategy::Knn m_knn;
        strategy::KMeans m_kMeans;
        int m_lastPose = 100;
    };
}
