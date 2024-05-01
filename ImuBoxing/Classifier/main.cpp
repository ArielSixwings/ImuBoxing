#include "source/PoseClassification.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // const std::string fileName = path + "dataTest.csv";
    // const std::string fileName = path + "guard.csv";
    // const std::string fileName = path + "jabEnd.csv";
    // const std::string fileName = path + "hookEnd.csv";
    rclcpp::spin(std::make_shared<classifier::PoseClassification>());
    rclcpp::shutdown();

    return 0;
}