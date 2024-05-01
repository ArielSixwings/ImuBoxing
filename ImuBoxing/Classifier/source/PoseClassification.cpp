#include "PoseClassification.h"
#include "Utils.h"

#include <memory>
#include <fstream>

namespace classifier
{

    PoseClassification::PoseClassification() : Node("PoseClassification")
    {

        m_subscription = create_subscription<geometry_msgs::msg::Vector3>(
            "imu/Angles", 10, std::bind(&PoseClassification::topicCallback, this, std::placeholders::_1));

        // m_knn(5);

        const std::string path = "ImuBoxing/data/";

        // const std::string fileName = path + "guard.csv";
        // const std::string fileName = path + "jabEnd.csv";
        // const std::string fileName = path + "hookEnd.csv";
        // const std::string fileName = path + "uppercutEnd.csv";

        const auto guardData = Utils::ReadCSV(path + "guard.csv", classifier::Data::Poses::Guard);
        const auto jabEndData = Utils::ReadCSV(path + "jabEnd.csv", classifier::Data::Poses::JabEnd);
        const auto hookEndData = Utils::ReadCSV(path + "hookEnd.csv", classifier::Data::Poses::HookEnd);
        const auto uppercutEndData = Utils::ReadCSV(path + "uppercutEnd.csv", classifier::Data::Poses::UppercutEnd);

        m_knn.AddData(guardData);
        m_knn.AddData(jabEndData);
        m_knn.AddData(hookEndData);
        m_knn.AddData(uppercutEndData);
    }

    void PoseClassification::topicCallback(const geometry_msgs::msg::Vector3::SharedPtr message)
    {

        std::vector<double> angles = {message->x, message->y, message->z};

        Data dataPoint(angles, classifier::Data::Poses::Unknown);

        const auto result = m_knn.Classify(dataPoint);

        switch (result.Label)
        {
        case classifier::Data::Poses::Guard:

            if (m_lastPose != classifier::Data::Poses::Guard)
            {

                RCLCPP_INFO(get_logger(), "GUARD");
            }

            m_lastPose = classifier::Data::Poses::Guard;
            break;

        case classifier::Data::Poses::JabEnd:

            if (m_lastPose != classifier::Data::Poses::JabEnd)
            {

                RCLCPP_INFO(get_logger(), "JAB END");
            }

            m_lastPose = classifier::Data::Poses::JabEnd;
            break;

        case classifier::Data::Poses::HookEnd:

            if (m_lastPose != classifier::Data::Poses::HookEnd)
            {

                RCLCPP_INFO(get_logger(), "HOOK END");
            }

            m_lastPose = classifier::Data::Poses::HookEnd;
            break;

        case classifier::Data::Poses::UppercutEnd:

            if (m_lastPose != classifier::Data::Poses::UppercutEnd)
            {

                RCLCPP_INFO(get_logger(), "UPPERCUT END");
            }

            m_lastPose = classifier::Data::Poses::UppercutEnd;
            break;

        default:
            break;
        }
    }

};