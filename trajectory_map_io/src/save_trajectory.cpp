
#include <iostream>
#include <string>
#include <vector>
#include <mutex>

// for ros
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

// for pcl
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

typedef pcl::PointXYZ PointType;                    // modify this pcl point type, based on slam's resigtered output.
typedef pcl::PointCloud<PointType> PointCloudType;

// save trajectory and map
int gb_save_trajectory(true), gb_save_map(true);
std::mutex g_odom_mutex, g_map_mutex;

// record lidar-scan and odoms
std::vector<PointCloudType> gv_registered_clouds;
std::vector<nav_msgs::Odometry> gv_odoms;

// trajectory and map output file.
std::string g_trajectory_file("/home/larrydong/trajectory.txt");
std::string g_map_file("/home/larrydong/map.ply");

// map save downsampling size
float g_map_ds_size(0.1);           // map downsampling size

// save each lidar's scan
void registeredCloudHandler(const sensor_msgs::PointCloud2ConstPtr &msg){
    PointCloudType pc;
	pcl::fromROSMsg(*msg, pc);      // TODO: assert msg's pointcloud type is same as PointCloudType.
    g_map_mutex.lock();
    gv_registered_clouds.push_back(pc);
    g_map_mutex.unlock();
}

// save each odom
void odomHandler(const nav_msgs::Odometry::ConstPtr &odom){
    g_odom_mutex.lock();
    gv_odoms.push_back(*odom);
    g_odom_mutex.unlock();
}

// capture the "save" command
void commandHandler(const std_msgs::String s){
    if(s.data == "s"){
        ROS_INFO("====== Save trajectory and map. ======");
        ROS_INFO_STREAM("       save trajectory?: " << gb_save_trajectory <<", save map?: " << gb_save_map);

        if(gb_save_trajectory){ 
            g_odom_mutex.lock();
            ofstream out(g_trajectory_file, ios::out);
            ROS_INFO("--> Start to save trajectory (TUM format)");
            out << "# timestampe x y z qx qy qz qw" << endl;
            for(auto odom : gv_odoms){
                geometry_msgs::Pose p = odom.pose.pose;
                out << std::setprecision(15) << odom.header.stamp.toSec() << " "
                    << std::setprecision(8)
                    << p.position.x << " "
                    << p.position.y << " "
                    << p.position.z << " "
                    << p.orientation.x << " "
                    << p.orientation.y << " "
                    << p.orientation.z << " "
                    << p.orientation.w << endl;
            }
            g_odom_mutex.unlock();
            ROS_INFO_STREAM("<-- Saved " << gv_odoms.size() <<" positions into file: " << g_trajectory_file);
        }

        if(gb_save_map){
            g_map_mutex.lock();
            ROS_INFO("--> Start to save map");
            // merge all pointcloud and downsampling
            PointCloudType::Ptr full_map(new PointCloudType());
            int lidar_scan_number = gv_registered_clouds.size();
            if(lidar_scan_number == 0){
                ROS_ERROR("Empty lidar-scan and no map saved");
                return ;
            }
            
            for(auto pc:gv_registered_clouds)      // merge all points (from each scan)
                *full_map += pc;
            
            pcl::VoxelGrid<PointType> dsFilter;
            dsFilter.setLeafSize(g_map_ds_size, g_map_ds_size, g_map_ds_size);
            dsFilter.setInputCloud(full_map);
            dsFilter.filter(*full_map);
            g_map_mutex.unlock();

            // save to ply/pcd file. Automatically check the file format and save.
            string map_file_type = g_map_file.substr(g_map_file.size() - 3, 3); // get output file type.
            if(map_file_type == "ply")
                pcl::io::savePLYFile(g_map_file, *full_map);
            else if(map_file_type == "pcd")
                pcl::io::savePCDFile(g_map_file, *full_map);
            else
                ROS_ERROR_STREAM("Unexpected map save format.");
            ROS_INFO_STREAM("--> Got " << lidar_scan_number<<" lidar-scan and saved " << full_map->size() << " points into file: " << g_map_file);
        }
    }
    else{
        ROS_WARN_STREAM("Unexpected command input: " << s.data);
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory_saver");
    ros::NodeHandle nh("~");
    ROS_INFO("--> Saving trajectory and map");
    ROS_INFO("  Type the following line to save trajectory and map:");
    ROS_INFO("  rostopic pub /cmd std_msgs/String \"s\" -1");

    nh.getParam("save_trajectory_en", gb_save_trajectory);
    nh.getParam("save_map_en", gb_save_map);

    nh.getParam("trajectory_save_file", g_trajectory_file);
    nh.getParam("map_save_file", g_map_file);
    nh.getParam("map_downsample_size", g_map_ds_size);

    // start to save map/trajectory
    // rostopic pub /cmd std_msgs/String "s" -1
    ros::Subscriber subString = nh.subscribe<std_msgs::String>("/cmd", 1, commandHandler);
    // TODO: modify the topic name according to your slam algorithm
    ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>("/Odometry", 100, odomHandler);
    ros::Subscriber subMap = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_registered", 100, registeredCloudHandler);
    ros::Rate r(100);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
