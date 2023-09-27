
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

using namespace std;

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloudType;

int gb_save_trajectory(true), gb_save_map(true);
std::mutex g_odom_mutex, g_map_mutex;

std::vector<PointCloudType> gv_registered_clouds;
std::vector<nav_msgs::Odometry> gv_odoms;

std::string g_trajectory_file("/home/larrydong/trajectory.txt");
std::string g_map_file("/home/larrydong/map.pcd");


void registeredCloudHandler(const sensor_msgs::PointCloud2ConstPtr &msg){
    // TODO: 
    // PointCloudType pc;
	// pcl::fromROSMsg(*msg, pc);
    // g_map_mutex.lock();
    // gv_registered_clouds.push_back(pc);
    // g_map_mutex.unlock();
}

void odomHandler(const nav_msgs::Odometry::ConstPtr &odom){
    g_odom_mutex.lock();
    gv_odoms.push_back(*odom);
    g_odom_mutex.unlock();
}

void commandHandler(const std_msgs::String s){
    if(s.data == "s"){
        ROS_INFO("====== Save trajectory and map. ======");
        ROS_INFO_STREAM("       save trajectory: " << gb_save_trajectory <<", save map: " << gb_save_map);
        if(gb_save_trajectory){ 
            g_odom_mutex.lock();
            ofstream out(g_trajectory_file, ios::out);
            ROS_INFO("--> Start to save trajectory");
            out << "# ts, qw, qx, qy, qz, tx, ty, tz " << endl;
            for(auto odom : gv_odoms){
                geometry_msgs::Pose p = odom.pose.pose;
                out << odom.header.stamp.toSec() << ", "
                    << p.orientation.w << ", "
                    << p.orientation.x << ", "
                    << p.orientation.y << ", "
                    << p.orientation.z << ", "
                    << p.position.x << ", "
                    << p.position.y << ", "
                    << p.position.z << endl;
            }
            g_odom_mutex.unlock();
            ROS_INFO_STREAM("<-- Saved " << gv_odoms.size() <<" position into file: " << g_trajectory_file);
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

    // start to save map/trajectory
    // rostopic pub /cmd std_msgs/String "s" -1
    ros::Subscriber subString = nh.subscribe<std_msgs::String>("/cmd", 1, commandHandler);
    ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>("/Odometry", 100, odomHandler);
    // ros::Subscriber subMap = nh.subscribe<PointCloudType>("/map", 100, registeredCloudHandler);
    ros::Rate r(100);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
