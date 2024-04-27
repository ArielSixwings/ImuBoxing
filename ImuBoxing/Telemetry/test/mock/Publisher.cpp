#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;

namespace mock
{

    class Publisher : public rclcpp::Node
    {
    public:
        Publisher()
            : Node("minimal_publisher")
        {

            m_publisher = create_publisher<geometry_msgs::msg::Vector3>("topic", 10);

            m_timer = create_wall_timer(500ms, std::bind(&Publisher::timer_callback, this));
        }

    private:
        void timer_callback()
        {
            geometry_msgs::msg::Vector3 message;
            message.x = m_count;
            message.y = m_count;
            message.z = m_count;

            ++m_count;

            m_publisher->publish(message);
        }

        rclcpp::TimerBase::SharedPtr m_timer;
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr m_publisher;

        float m_count = 0.0;
    };
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mock::Publisher>());
    rclcpp::shutdown();

    return 0;
}