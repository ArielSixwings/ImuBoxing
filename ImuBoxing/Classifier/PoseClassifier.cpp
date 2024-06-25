#include "source/PoseClassification.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<classifier::PoseClassification>());
    rclcpp::shutdown();

    return 0;
}