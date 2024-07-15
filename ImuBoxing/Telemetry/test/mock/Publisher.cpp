#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
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
            : Node("minimalPublisher")
        {

            RCLCPP_INFO(get_logger(), "Starting Publisher mock");

            m_publisher = create_publisher<std_msgs::msg::Float32MultiArray>("mock/Angles", 10);

            m_timer = create_wall_timer(500ms, std::bind(&Publisher::timer_callback, this));
        }

    private:
        void timer_callback()
        {
            std_msgs::msg::Float32MultiArray message;
            message.data = {m_count, m_count, m_count, m_count};

            ++m_count;

            m_publisher->publish(message);
        }

        rclcpp::TimerBase::SharedPtr m_timer;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr m_publisher;

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