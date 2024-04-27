#include "Subscriber.h"

#include <memory>

namespace classifier
{

    ImuSubscriber::ImuSubscriber() : Node("imu_subscriber")
    {
        m_subscription = create_subscription<geometry_msgs::msg::Vector3>(
            "imu/angles", 10, std::bind(&ImuSubscriber::topic_callback, this, std::placeholders::_1));
    }

    void ImuSubscriber::topic_callback(const geometry_msgs::msg::Vector3::SharedPtr message)
    {
        RCLCPP_INFO(get_logger(), "Received IMU angles: x: '%f', y: '%f', z: '%f'",
                    message->x, message->y, message->z);

        std::vector<float> dataMessage = {message->x, message->y, message->z};

        m_messages.push_back(dataMessage);
    }

    std::vector<std::vector<float>> ImuSubscriber::GetMessages()
    {
        return m_messages;
    }
};
