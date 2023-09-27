cmake_minimum_required(VERSION 3.2)
project(trajectory_saver)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(PCL_DIR /usr/lib/x86_64-linux-gnu/cmake/pcl)

find_package(catkin REQUIRED COMPONENTS
    PCL REQUIRED
    roscpp
    sensor_msgs
    std_msgs
    nav_msgs
    rosbag
    pcl_ros
)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


## PCL library
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(save_trajectory_node 
    src/save_trajectory.cpp
)

target_link_libraries(save_trajectory_node ${LIVOX_LIDAR_SDK_LIBRARY} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
