#include "src/ImuNode.h"

#include <thread>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<telemetry::ImuNode>();

        std::thread spin_thread([node]()
                                { rclcpp::spin(node); });

        spin_thread.join();
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ImuNode"), "Exception thrown: %s", e.what());
    }
    catch (...)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ImuNode"), "Unknown exception thrown");
    }

    rclcpp::shutdown();

    return 0;
}
