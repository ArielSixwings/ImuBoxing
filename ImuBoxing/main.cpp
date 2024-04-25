#include "Telemetry/source/ImuNode.h"
#include "Classifier/source/Subscriber.h"

#include <thread>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try
    {

        auto node = std::make_shared<telemetry::ImuNode>();

        auto subscriber = std::make_shared<classifier::ImuSubscriber>();

        std::thread spin_thread([node]()
                                { rclcpp::spin(node); });

        rclcpp::spin(subscriber);

        // spin_thread.join();
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception thrown: %s", e.what());
    }
    catch (...)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Unknown exception thrown");
    }

    rclcpp::shutdown();

    return 0;
}
