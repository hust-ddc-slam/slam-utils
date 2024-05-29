cmake_minimum_required(VERSION 3.0.2)
project(map_compare)

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
link_directories(/usr/lib/x86_64-linux-gnu)
add_definitions(${PCL_DEFINITIONS})

add_executable(map_compare_node src/main.cpp)

target_link_libraries(map_compare_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

