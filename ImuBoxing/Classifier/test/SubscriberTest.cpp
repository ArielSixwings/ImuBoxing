#include "Subscriber.h"
#include "mock/Publisher.h"

#include <algorithm>
#include <iostream>

#include <catch2/catch_test_macros.hpp>

SCENARIO("Should acquire messages published to given topic.", "[Unit][Subscriber][GetMessages]")
{
  int argc = 0;
  char **argv = nullptr;

  rclcpp::init(argc, argv);

  GIVEN("A subscriber and A publisher mock")
  {

    const auto topic = "imu/angles";

    auto publisher = std::make_shared<mock::Publisher>(topic);

    auto subscriber = std::make_shared<classifier::ImuSubscriber>();

    // Spin the publisher asynchronously
    std::thread spin_publisher_thread([publisher]()
                                      { rclcpp::spin_some(publisher); });

    // Spin the subscriber asynchronously
    std::thread spin_subscriber_thread([subscriber]()
                                       { rclcpp::spin_some(subscriber); });

    // Wait for a moment to ensure nodes have started spinning
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Publish messages

    std::cout << "Published messages." << std::endl;

    // Wait for a moment to ensure messages have been processed
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Increase wait time

    // Join the publisher spin thread
    spin_publisher_thread.join();

    // Join the subscriber spin thread
    spin_subscriber_thread.join();

    WHEN("GetMessages method is called")
    {
      const auto result = subscriber->GetMessages();

      THEN("Resulting vector has defined values")
      {
        std::vector<std::vector<float>> messages = {
            {10.5, 9.2, 0.0},
            {-43.0, -5.0, 1.0},
            {-61.0, -70.0, 86.0},
            {62.0, 61.0, -35.0},
            {-68.0, -22.0, -66.0},
            {-69.0, -77.0, -30.0},
            {-31.0, 36.0, -20.0},
            {-68.0, 1.0, 70.0},
            {20.0, 81.0, 11.0},
            {-30.0, -90.0, -53.0}};

        CHECK(result == messages);
      }
    }
  }

  rclcpp::shutdown();
}
