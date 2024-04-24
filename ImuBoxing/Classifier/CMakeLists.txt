project(Classifier)

find_package(ament_cmake REQUIRED)
find_package(Shark REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(${SHARK_INCLUDE_DIRS})  # This line adds Shark include directory

add_executable(knn_classifier_node KnnNode.cpp)
target_link_libraries(knn_classifier_node ${SHARK_LIBRARIES} rclcpp geometry_msgs std_srvs)
