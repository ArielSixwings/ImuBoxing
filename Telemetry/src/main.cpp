#include "ImuNode.h"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<ImuNode>();
        rclcpp::spin(node);
        node->CreateAnglesPublisher();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("ImuNode"), "Exception thrown: %s", e.what());
    } catch (...) {
        RCLCPP_ERROR(rclcpp::get_logger("ImuNode"), "Unknown exception thrown");
    }
    
    rclcpp::shutdown();

    return 0;
}