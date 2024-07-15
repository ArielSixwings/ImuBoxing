#include "source/FeatureExtractor.h"

int main(int argc, char *argv[])
{
    const std::string path = "ImuBoxing/data/";

    // const std::string fileName = path + "dataTest.csv";
    // const std::string fileName = path + "guardQuaternion.csv";
    // const std::string fileName = path + "jabEndQuaternion.csv";
    // const std::string fileName = path + "hookEndQuaternion.csv";
    const std::string fileName = path + "uppercutEndQuaternion.csv";

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<classifier::FeatureExtractor>(fileName, 2048));
    rclcpp::shutdown();

    return 0;
}