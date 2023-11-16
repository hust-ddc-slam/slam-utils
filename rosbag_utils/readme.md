
# rosbag_utils

This package include some rosbag process code.

`rosbag_play` generate lidar scan each time of a rosbag.
`rosbag_output` write some rosbag data to a txt for further analysize (TODO).

## rosbag_play

### Usage
```bash
roslaunch rosbag_utils rosbag_player
```
Then type "Enter" key in the terminal to generate a lidar scan with corresponding IMUs.  
Type 's' or 'q' and then type "Enter" to stop the program.  

### How to modify
Check the `rosbag_play.launch` file:
- `bag_file`: which rosbag to load.
- `lidar_topic`/`imu_topic`: lidar and imu's topic in rosbag.


The defaut Lidar message format is: `livox_ros_driver2::CustomMsg`, and IMU message format is: `sensor_msgs::Imu`.  
You need to modify the source code if lidar/imu are not those format.  
(Attention: you should pay attention if you're using 'livox_ros_driver' or 'livox_ros_driver2')


## rosbag_merge

Merge rosbag index from '0.bag' to 'N.bag' in one folder.  
Maybe need "/scripts/rename.bash" to change all rosbag into 0~N order.  
Output a merged rosbag.


## rosbag_timestamp2csv

This read a rosbag and write topic's timestamp into a csv.  



