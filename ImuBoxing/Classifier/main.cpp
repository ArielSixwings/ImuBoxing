#include "source/FeatureExtractor.h"

int main(int argc, char *argv[])
{
    const std::string path = "ImuBoxing/data/";

    rclcpp::init(argc, argv);

    const std::string fileName = path + "dataTest.csv";
    // const std::string fileName = path + "guard.csv";
    // const std::string fileName = path + "jabEnd.csv";
    // const std::string fileName = path + "hookEnd.csv";
    // const std::string fileName = path + "uppercutEnd.csv";
    rclcpp::spin(std::make_shared<classifier::FeatureExtractor>(fileName, true));
    rclcpp::shutdown();

    return 0;
}