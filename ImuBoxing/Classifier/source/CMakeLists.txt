find_package(rclcpp REQUIRED)

add_library(classifierSource 
            classes/Data.cpp
            classes/Group.cpp
            classes/LabeledDistance.cpp
            FeatureExtractor.cpp 
            PoseClassification.cpp
            TechniqueClassification.cpp
            strategy/KMeans.cpp 
            strategy/Knn.cpp 
            Utils.cpp )
            
target_include_directories(classifierSource PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

ament_target_dependencies(classifierSource rclcpp geometry_msgs std_srvs)