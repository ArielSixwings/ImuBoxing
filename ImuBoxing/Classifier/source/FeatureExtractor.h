
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"

namespace classifier
{

    class FeatureExtractor : public rclcpp::Node
    {
    public:
        FeatureExtractor(const std::string &fileName,
                         const bool inTest = false);

        void SaveCsv(const std::vector<double> &features);

    private:
        void topicCallback(const geometry_msgs::msg::Vector3::SharedPtr message);

        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr m_subscription;

        size_t m_count = 0;
        std::string m_fileName;
    };
}
