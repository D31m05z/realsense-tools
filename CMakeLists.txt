cmake_minimum_required(VERSION 3.0)
set(CMAKE_CXX_VERSION 14)
set(CMAKE_CXX_VERSION_REQUIRED 1)

project(mapzing)

find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

add_executable(mapzing main.cpp)
target_link_libraries(mapzing PRIVATE realsense2::realsense2 ${OpenCV_LIBRARIES})

add_executable(mapzing_record record/record.cpp)
target_link_libraries(mapzing_record PRIVATE realsense2::realsense2 ${OpenCV_LIBRARIES})

add_subdirectory(mapzing_slam)
