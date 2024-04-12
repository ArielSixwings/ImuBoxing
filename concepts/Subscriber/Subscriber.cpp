#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"

class ImuSubscriber : public rclcpp::Node
{
public:
    ImuSubscriber() : Node("imu_subscriber")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "imu/angles", 10, std::bind(&ImuSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Received IMU angles: x: '%f', y: '%f', z: '%f'", msg->x, msg->y, msg->z);
    }
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscription_;
};
