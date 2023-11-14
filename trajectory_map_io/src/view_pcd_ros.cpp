
#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloudType;


void pubRosMsg(const PointCloudType& pc, const ros::Publisher& puber){
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(pc, msg);
    msg.header.stamp = ros::Time().now();
    msg.header.frame_id = "map";
    puber.publish(msg);
}


// TODO: publish (multiple) PointCloud and view in ROS.
int main(int argc, char **argv) {
    ros::init(argc, argv, "pcl_viewer");
    ros::NodeHandle nh("~");
    
    ros::Publisher pub_pc1 = nh.advertise<sensor_msgs::PointCloud2>("/cloud1", 1);
    ros::Publisher pub_pc2 = nh.advertise<sensor_msgs::PointCloud2>("/cloud2", 1);

    string f1 = "/home/larrydong/floor3-data/yjy-2/fastlio.pcd";
    string f2 = "/home/larrydong/floor3-data/yjy-2/lio-livox.pcd";

    // nh.getParam("pcd_filename", pcd_filename);
    
    ROS_INFO_STREAM("--> Loading PCD 1: " << f1);
    PointCloudType::Ptr cloud (new PointCloudType);
    if (pcl::io::loadPCDFile<PointType> (f1, *cloud) == -1){
        ROS_ERROR("Couldn't load pcd file.");
        return (-1);
    }
    ROS_INFO_STREAM("<-- Loaded " << cloud->width * cloud->height << " data points ");
    
    ROS_INFO_STREAM("--> Loading PCD 2: " << f2);
    PointCloudType::Ptr cloud2 (new PointCloudType);
    if (pcl::io::loadPCDFile<PointType> (f2, *cloud2) == -1){
        ROS_ERROR("Couldn't load pcd file.");
        return (-1);
    }
    ROS_INFO_STREAM("<-- Loaded " << cloud2->width * cloud2->height << " data points ");
    

    ros::Rate rate(10);
    while (ros::ok()){
        pubRosMsg(*cloud, pub_pc1);
        pubRosMsg(*cloud2, pub_pc2);
        ros::spinOnce();
    }
    return (0);
}
