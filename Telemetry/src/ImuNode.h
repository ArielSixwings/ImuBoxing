#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_srvs/srv/empty.hpp"
#include <boost/asio.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

class ImuNode : public rclcpp::Node {
public:
    ImuNode();

    void CreateAnglesPublisher();

    void StartStreaming(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                        std::shared_ptr<std_srvs::srv::Empty::Response> response);

    void StopStreaming(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                        std::shared_ptr<std_srvs::srv::Empty::Response> response);

    void ApplyCommand(const std::string& command, bool show_response = false);

    static std::string CreateImuCommand( int logicalId,
                                         int commandNumber,
                                         const std::vector<int>& arguments = {});


private:
    int imuNumber = 3;
    bool m_streaming = false;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr m_publisher;
    std::shared_ptr<boost::asio::serial_port> m_serialPort;
    boost::asio::io_service m_io; // Declare io object
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr m_startService;
    rclcpp::TimerBase::SharedPtr m_timer;
};
