
#pragma once

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>

namespace classifier
{
    class FeatureExtractor : public rclcpp::Node
    {
    public:
        FeatureExtractor(const std::string &fileName,
                         const size_t numberOfEntries,
                         const bool inTest = false);

        void SaveCsv(const std::vector<float> &features);

    private:
        void topicCallback(const std_msgs::msg::Float32MultiArray::SharedPtr message);

        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_subscription;

        size_t m_count = 0;
        size_t m_numberOfEntries;
        std::string m_fileName;
    };
}
