#include "Subscriber.h"

#include <memory>

namespace classifier
{

    ImuSubscriber::ImuSubscriber() : Node("imu_subscriber")
    {
        m_subscription = create_subscription<geometry_msgs::msg::Vector3>(
            "imu/angles", 10, std::bind(&ImuSubscriber::topic_callback, this, std::placeholders::_1));
    }

    void ImuSubscriber::topic_callback(const geometry_msgs::msg::Vector3::SharedPtr message) const
    {
        RCLCPP_INFO(get_logger(), "Received IMU angles: x: '%f', y: '%f', z: '%f'",
                    message->x, message->y, message->z);
    }
};
