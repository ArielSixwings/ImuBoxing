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

        const std::string path = "ImuBoxing/data/";

        const auto guardData = Utils::ReadCSV(path + "guard.csv", classifier::classes::Data::Poses::Guard);
        const auto jabEndData = Utils::ReadCSV(path + "jabEnd.csv", classifier::classes::Data::Poses::JabEnd);
        const auto hookEndData = Utils::ReadCSV(path + "hookEnd.csv", classifier::classes::Data::Poses::HookEnd);
        const auto uppercutEndData = Utils::ReadCSV(path + "uppercutEnd.csv", classifier::classes::Data::Poses::UppercutEnd);

        m_knn.AddData(guardData);
        m_knn.AddData(jabEndData);
        m_knn.AddData(hookEndData);
        m_knn.AddData(uppercutEndData);

        classifier::classes::Group guardGroup(guardData);
        classifier::classes::Group jabEndGroup(jabEndData);
        classifier::classes::Group hookEndGroup(hookEndData);
        classifier::classes::Group uppercutEndGroup(uppercutEndData);

        m_kMeans.AddGroup(guardGroup);
        m_kMeans.AddGroup(jabEndGroup);
        m_kMeans.AddGroup(hookEndGroup);
        m_kMeans.AddGroup(uppercutEndGroup);
    }

    void PoseClassification::topicCallback(const geometry_msgs::msg::Vector3::SharedPtr message)
    {

        // RCLCPP_INFO(get_logger(), "%lf %lf %lf", message->x, message->y, message->z);

        std::vector<double> angles = {message->x, message->y, message->z};

        classes::Data dataPoint(angles, classifier::classes::Data::Poses::Unclassified);

        // const auto result = m_knn.Classify(dataPoint);
        const auto result = m_kMeans.Classify(dataPoint);

        switch (result.Label)
        {
        case classifier::classes::Data::Poses::Guard:

            if (m_lastPose != classifier::classes::Data::Poses::Guard)
            {

                RCLCPP_INFO(get_logger(), "GUARD");
            }

            m_lastPose = classifier::classes::Data::Poses::Guard;
            break;

        case classifier::classes::Data::Poses::JabEnd:

            if (m_lastPose != classifier::classes::Data::Poses::JabEnd)
            {

                RCLCPP_INFO(get_logger(), "JAB END");
            }

            m_lastPose = classifier::classes::Data::Poses::JabEnd;
            break;

        case classifier::classes::Data::Poses::HookEnd:

            if (m_lastPose != classifier::classes::Data::Poses::HookEnd)
            {

                RCLCPP_INFO(get_logger(), "HOOK END");
            }

            m_lastPose = classifier::classes::Data::Poses::HookEnd;
            break;

        case classifier::classes::Data::Poses::UppercutEnd:

            if (m_lastPose != classifier::classes::Data::Poses::UppercutEnd)
            {

                RCLCPP_INFO(get_logger(), "UPPERCUT END");
            }

            m_lastPose = classifier::classes::Data::Poses::UppercutEnd;
            break;

        case classifier::classes::Data::Poses::Unknown:

            if (m_lastPose != classifier::classes::Data::Poses::Unknown)
            {

                RCLCPP_INFO(get_logger(), "UNKNOWN");
            }

            m_lastPose = classifier::classes::Data::Poses::Unknown;
            break;

        default:
            break;
        }
    }

};