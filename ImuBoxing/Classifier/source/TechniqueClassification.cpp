#include "TechniqueClassification.h"

namespace classifier
{
    TechniqueClassification::TechniqueClassification() : Node("TechniqueClassification")
    {

        m_subscription = create_subscription<std_msgs::msg::Int32>(
            "pose", 10, std::bind(&TechniqueClassification::topicCallback, this, std::placeholders::_1));
    }

    void TechniqueClassification::topicCallback(const std_msgs::msg::Int32 message)
    {
        if ((message.data == classes::Data::Poses::Guard) and
            (LastStablePose != classes::Data::Poses::Guard))
        {
            RCLCPP_INFO(get_logger(), "ON GUARD");

            LastStablePose = classes::Data::Poses::Guard;
            MovingCount = 0;
            return;
        }

        if (LastStablePose == classes::Data::Poses::Guard)
        {
            switch (message.data)
            {
            case classes::Data::Poses::JabEnd:

                RCLCPP_INFO(get_logger(), "JAB");
                LastStablePose = classes::Data::Poses::JabEnd;

                break;

            case classes::Data::Poses::HookEnd:

                RCLCPP_INFO(get_logger(), "HOOK");
                LastStablePose = classes::Data::Poses::HookEnd;

                break;

            case classes::Data::Poses::UppercutEnd:

                RCLCPP_INFO(get_logger(), "UPPERCUT");
                LastStablePose = classes::Data::Poses::UppercutEnd;

                break;

            case classes::Data::Poses::Unknown:
                MovingCount++;
                break;
            default:
                break;
            }
        }
    }
}
