#include "FeatureExtractor.h"
#include "Utils.h"

#include <memory>
#include <fstream>

namespace classifier
{
    FeatureExtractor::FeatureExtractor(const std::string &fileName,
                                       const size_t numberOfEntries,
                                       const bool inTest) : Node("FeatureExtractor"),
                                                            m_numberOfEntries(numberOfEntries),
                                                            m_fileName(fileName)
    {
        const auto topic = inTest ? "mock/Angles" : "imu/Angles";

        m_subscription = create_subscription<std_msgs::msg::Float32MultiArray>(
            topic, 10, std::bind(&FeatureExtractor::topicCallback, this, std::placeholders::_1));
    }

    void FeatureExtractor::topicCallback(const std_msgs::msg::Float32MultiArray::SharedPtr message)
    {
        ++m_count;

        auto percentage = static_cast<float>(m_count) / static_cast<float>(m_numberOfEntries);

        Utils::PrintProgressBar(percentage);

        if (m_count >= m_numberOfEntries)
        {
            rclcpp::shutdown();
        }

        std::vector<float> angles = message->data;
        SaveCsv(angles);
    }

    void FeatureExtractor::SaveCsv(const std::vector<float> &features)
    {

        std::ofstream csvFile(m_fileName, std::ios::app);

        if (not csvFile.is_open())
        {
            RCLCPP_ERROR(get_logger(), "Error: Could not open file %s", m_fileName.c_str());
            return;
        }

        std::string csvData;
        for (const float &feature : features)
        {
            csvData += std::to_string(feature) + ',';
        }

        if (not csvData.empty())
        {
            csvData.pop_back();
        }

        csvData += '\n';

        csvFile << csvData;

        RCLCPP_INFO(get_logger(), "Features saved to %s", m_fileName.c_str());
    }
};
