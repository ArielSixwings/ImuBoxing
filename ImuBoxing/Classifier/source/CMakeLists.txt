find_package(rclcpp REQUIRED)

add_library(classifierSource 
            strategy/Knn.cpp 
            Utils.cpp 
            DataDefinition.cpp 
            FeatureExtractor.cpp )
            
target_include_directories(classifierSource PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

ament_target_dependencies(classifierSource rclcpp geometry_msgs std_srvs)