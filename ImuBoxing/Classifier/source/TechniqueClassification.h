#pragma once

#include "classes/Data.h"
#include "classes/Technique.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

namespace classifier
{
    class TechniqueClassification : public rclcpp::Node
    {
    public:
        TechniqueClassification();

    private:
        void topicCallback(const std_msgs::msg::Int32 message);

        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_subscription;

        uint8_t LastStablePose = classes::Data::Poses::Unclassified;
        size_t MovingCount = 0;

        classes::Technique CurrentTechnique;
    };
}
