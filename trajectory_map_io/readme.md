
# trajectory_map_io

This package save or load(TODO) trajectory & map.


## Usage

### Saving trajectory and map:

Since some slam-code publish odom (not path), and registered laser scan (not full map),  
this code just save odometry and scan when running, and save to file when received the command.

Trajectory codes can be saved to a `txt` or `csv` file, and map can be saved as `ply` or `pcd` file.

Run this node:
```bash
# subscribe the odom and map data.
roslaunch trajectory_saver save_trajectory.launch
# in a new terminal type the following line to save data.
rostopic pub /cmd std_msgs/String "s" -1   
```

**How to modify?**
- Modify the laser-scan's topic and odom's topic name in source codes;
- Modify the output file path in launch file;
- Assert laser-scan's type is `PointType` defined in source codes;
- Modify the map downsampling size if needed;


