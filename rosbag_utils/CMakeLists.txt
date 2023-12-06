cmake_minimum_required(VERSION 3.2)
project(rosbag_utils)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(PCL_DIR /usr/lib/x86_64-linux-gnu/cmake/pcl)

SET(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
    PCL REQUIRED
    roscpp
    sensor_msgs
    std_msgs
    nav_msgs
    rosbag
    pcl_ros
    livox_ros_driver2
)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

## PCL library
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


## replay a rosbag controled by keys
add_executable(rosbag_player 
    src/rosbag_player.cpp
)
target_link_libraries(rosbag_player ${LIVOX_LIDAR_SDK_LIBRARY} ${catkin_LIBRARIES} ${PCL_LIBRARIES})


## merge several rosbag.
add_executable(rosbag_merge 
    src/rosbag_merge.cpp
)
target_link_libraries(rosbag_merge ${LIVOX_LIDAR_SDK_LIBRARY} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

## output a rosbag messages' timestamp into csv file
add_executable(rosbag_timestamp2csv
    src/rosbag_timestamp2csv.cpp
)
target_link_libraries(rosbag_timestamp2csv ${LIVOX_LIDAR_SDK_LIBRARY} ${catkin_LIBRARIES} ${PCL_LIBRARIES})


## output a rosbag IMU and chassis into csv file
add_executable(rosbag_exportChassisIMU
    src/rosbag_export_chassis_imu.cpp
)
target_link_libraries(rosbag_exportChassisIMU ${LIVOX_LIDAR_SDK_LIBRARY} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
