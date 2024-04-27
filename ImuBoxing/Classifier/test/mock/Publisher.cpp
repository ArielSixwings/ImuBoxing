#include "Publisher.h"

#include <sstream>
#include <thread>
#include <ranges>
#include <sys/ioctl.h>

namespace mock
{
    Publisher::Publisher(const std::string &topic) : Node("imuNodeMock")
    {

        m_publisher = create_publisher<geometry_msgs::msg::Vector3>(topic, 1);

        m_timer = create_wall_timer(std::chrono::milliseconds(5),
                                    std::bind(&Publisher::LoopCallback, this));
    }

    void Publisher::LoopCallback()
    {

        geometry_msgs::msg::Vector3 message;
        message.x = 5.5;
        message.y = 5.5;
        message.z = 5.5;

        m_publisher->publish(message);
    }
}
