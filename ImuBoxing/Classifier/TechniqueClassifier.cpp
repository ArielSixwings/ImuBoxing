#include "source/TechniqueClassification.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<classifier::TechniqueClassification>());
    rclcpp::shutdown();

    return 0;
}