cmake_minimum_required(VERSION 3.0)

project(mapzing_slam)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED 1)

find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(ORB_SLAM2 REQUIRED)


add_executable(${PROJECT_NAME} mapzing_slam.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE realsense2::realsense2 ${OpenCV_LIBRARIES} ::ORB_SLAM2)
