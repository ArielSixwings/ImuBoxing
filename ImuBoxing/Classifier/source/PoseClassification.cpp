#include "PoseClassification.h"
#include "Utils.h"

#include <memory>
#include <fstream>

namespace classifier
{
    PoseClassification::PoseClassification() : Node("PoseClassification")
    {

        m_subscription = create_subscription<std_msgs::msg::Float32MultiArray>(
            "imu/Angles", 10, std::bind(&PoseClassification::topicCallback, this, std::placeholders::_1));

        m_publisher = create_publisher<std_msgs::msg::Int32>("pose", 10);

        const std::string path = "ImuBoxing/data/";

        const auto guardData = Utils::ReadCSV(path + "guardQuaternion.csv", classes::Data::Poses::Guard);
        const auto jabEndData = Utils::ReadCSV(path + "jabEndQuaternion.csv", classes::Data::Poses::JabEnd);
        const auto hookEndData = Utils::ReadCSV(path + "hookEndQuaternion.csv", classes::Data::Poses::HookEnd);
        const auto uppercutEndData = Utils::ReadCSV(path + "uppercutEndQuaternion.csv", classes::Data::Poses::UppercutEnd);

        m_knn.AddData(guardData);
        m_knn.AddData(jabEndData);
        m_knn.AddData(hookEndData);
        m_knn.AddData(uppercutEndData);

        classes::Group guardGroup(guardData);
        classes::Group jabEndGroup(jabEndData);
        classes::Group hookEndGroup(hookEndData);
        classes::Group uppercutEndGroup(uppercutEndData);

        m_kMeans.AddGroup(guardGroup);
        m_kMeans.AddGroup(jabEndGroup);
        m_kMeans.AddGroup(hookEndGroup);
        m_kMeans.AddGroup(uppercutEndGroup);
    }

    void PoseClassification::topicCallback(const std_msgs::msg::Float32MultiArray::SharedPtr message)
    {

        std::vector<float> angles = message->data;

        classes::Data dataPoint(angles, classes::Data::Poses::Unclassified);

        // const auto result = m_knn.Classify(dataPoint);
        const auto result = m_kMeans.Classify(dataPoint);

        switch (result.Label)
        {
        case classes::Data::Poses::Guard:

            if (m_lastPose != classes::Data::Poses::Guard)
            {

                RCLCPP_INFO(get_logger(), "GUARD");
            }

            m_lastPose = classes::Data::Poses::Guard;
            break;

        case classes::Data::Poses::JabEnd:

            if (m_lastPose != classes::Data::Poses::JabEnd)
            {

                RCLCPP_INFO(get_logger(), "JAB END");
            }

            m_lastPose = classes::Data::Poses::JabEnd;
            break;

        case classes::Data::Poses::HookEnd:

            if (m_lastPose != classes::Data::Poses::HookEnd)
            {

                RCLCPP_INFO(get_logger(), "HOOK END");
            }

            m_lastPose = classes::Data::Poses::HookEnd;
            break;

        case classes::Data::Poses::UppercutEnd:

            if (m_lastPose != classes::Data::Poses::UppercutEnd)
            {

                RCLCPP_INFO(get_logger(), "UPPERCUT END");
            }

            m_lastPose = classes::Data::Poses::UppercutEnd;
            break;

        case classes::Data::Poses::Unknown:

            if (m_lastPose != classes::Data::Poses::Unknown)
            {

                RCLCPP_INFO(get_logger(), "UNKNOWN");
            }

            m_lastPose = classes::Data::Poses::Unknown;
            break;

        default:
            break;
        }

        std_msgs::msg::Int32 pose;

        pose.data = m_lastPose;

        m_publisher->publish(pose);
    }
}
