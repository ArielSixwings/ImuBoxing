cmake_minimum_required(VERSION 3.14)
project(KnnSharkProject)

# Set C++ standard
set(CMAKE_CXX_STANDARD 23)

# Include FetchContent module
include(FetchContent)

# Declare the Shark library
FetchContent_Declare(
  shark
  GIT_REPOSITORY https://github.com/Shark-ML/Shark.git
  GIT_TAG        master  # Optionally, specify a particular tag or commit
)

# Check if Shark has been downloaded; if not, do so now
FetchContent_GetProperties(shark)
if(NOT shark_POPULATED)
  FetchContent_Populate(shark)
  # Optionally, you can add any build configuration needed for Shark here
  add_subdirectory(${shark_SOURCE_DIR} ${shark_BINARY_DIR})
endif()

# Find Boost (required by Shark)
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

# Define the executable
add_executable(Knn_Shark main.cpp)

# Include directories from Shark
target_include_directories(Knn_Shark PRIVATE ${shark_SOURCE_DIR}/include)

# Link against Shark (adjust the target name based on Shark's configuration if needed)
target_link_libraries(Knn_Shark PRIVATE Shark)
