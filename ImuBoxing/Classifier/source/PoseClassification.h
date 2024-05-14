#pragma once

#include "strategy/Knn.h"
#include "strategy/KMeans.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/int32.hpp"

namespace classifier
{

    class PoseClassification : public rclcpp::Node
    {
    public:
        PoseClassification();

    private:
        void topicCallback(const geometry_msgs::msg::Vector3::SharedPtr message);

        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr m_subscription;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr m_publisher;

        strategy::Knn m_knn;
        strategy::KMeans m_kMeans;
        int m_lastPose = 100;
    };
}
