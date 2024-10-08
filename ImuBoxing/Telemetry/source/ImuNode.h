#pragma once

#include "SpaceSensor.h"

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <boost/asio.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace telemetry
{
    class ImuNode : public rclcpp::Node
    {
    public:
        ImuNode();

        void StartStreaming(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                            std::shared_ptr<std_srvs::srv::Empty::Response> response);

        void StopStreaming(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                           std::shared_ptr<std_srvs::srv::Empty::Response> response);

        void TareSensor(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                        std::shared_ptr<std_srvs::srv::Empty::Response> response);

        void TareSensorQuaternion(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                  std::shared_ptr<std_srvs::srv::Empty::Response> response);

        void OffsetWithCurrentOrientation(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                          std::shared_ptr<std_srvs::srv::Empty::Response> response);

        void SetBaseOffsetWithCurrentOrientation(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                                 std::shared_ptr<std_srvs::srv::Empty::Response> response);

        void SetStreamingSlots(const std::vector<uint8_t> &commandData);

        void SetStreamingTiming(const int frequency);

        void ApplyCommand(SpaceSensor::BinaryCommand &binaryCommand, bool showResponse = false);

        void SetCompassEnabledToZero();

        void SetEulerAngleDecompositionOrder();

        void LoopCallback();

    private:
        int imuNumber = 3;
        bool m_streaming = false;

        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr m_publisher;

        std::shared_ptr<boost::asio::serial_port> m_serialPort;
        boost::asio::io_service m_io;
        rclcpp::TimerBase::SharedPtr m_timer;

        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr m_startService;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr m_stopService;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr m_serviceTare;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr m_serviceTareQuaternion;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr m_serviceOffset;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr m_serviceBaseOffset;
    };
}
