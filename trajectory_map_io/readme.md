
# trajectory_map_io

This package save or load(TODO) trajectory & map.


## Usage

### Saving trajectory and map:

Since some slam-code publish odom (not path), and registered laser scan (not full map),  
this code just save odometry and scan when running, and save to file when received the command.

Run this node:
```bash
# subscribe the odom and map data.
roslaunch trajectory_saver save_trajectory.launch
# in a new terminal type the following line to save data.
rostopic pub /cmd std_msgs/String "s" -1   
```

