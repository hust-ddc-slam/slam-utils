
# rosbag_play

This package control the output of a rosbag.


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
Good news! Now you only need to modify the `save_trajectory.launch` file for recording. Check the parameters in launch file:  
- `save_trajectory_en`/`save_map_en`: save or not save trajectory and map.
- `output_folder`: the folder to save the output files. Remeber to add the / at the end. 
- `method`: could be fastlio/liosam, whatever. This method will be the name of trajectory and map output, e.g., "fastlio.txt", "fastlio.pcd"
- `map_type`: output map's type, "pcd" or "ply".
- `map_downsample_size`: downsampling size of the output map. 0.1 for small/indoor scenes, and 0.2 or larger for large outdoor scene is recommanded.
- `odom_topic`/`map_topic`: change this name to register the odometry/(registered)scan topic from your SLAM method.


### Loading and view a PCD file.

**View (multiple) PCD map in ROS**
```bash
roslaunch trajectory_saver view_pcd_ros.launch
```

**How to modify?**
check `view_pcd_ros.launch` file, and change configure:
- `map_folder`: where to load all PCD files.
- `pointcloud_number`: how many PCD files are loaded. 
- `pcX` (X=0~4): PCD to view. X>`pointcloud_number` will not be loaded and shown.




