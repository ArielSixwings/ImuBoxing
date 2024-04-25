
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"

namespace classifier
{

    class ImuSubscriber : public rclcpp::Node
    {
    public:
        ImuSubscriber();

    private:
        void topic_callback(const geometry_msgs::msg::Vector3::SharedPtr message) const;

        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr m_subscription;
    };
}
