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
target_link_libraries(rosbag_player  ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# convert a 128 lines rosbag to 32 lines
add_executable(ouster_converter 
    src/ouster128to32.cpp
)
target_link_libraries(ouster_converter  ${catkin_LIBRARIES} ${PCL_LIBRARIES})


## change a frame id for /
add_executable(change_frameid 
    src/change_frameid.cpp
)
target_link_libraries(change_frameid  ${catkin_LIBRARIES} ${PCL_LIBRARIES})


## merge NCD bags and clean
add_executable(process_NCD
    src/process_NCD.cpp
)
target_link_libraries(process_NCD  ${catkin_LIBRARIES} ${PCL_LIBRARIES})
