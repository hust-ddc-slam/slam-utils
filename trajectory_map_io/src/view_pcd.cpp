
#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "pcd_viewer");
    ros::NodeHandle nh("~");
    
    string pcd_filename = "test_pcd.pcd";
    nh.getParam("pcd_filename", pcd_filename);
    ROS_INFO_STREAM("--> Loading pcd from file: " << pcd_filename);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_filename, *cloud) == -1){
        ROS_ERROR("Couldn't load pcd file.");
        return (-1);
    }

    ROS_INFO_STREAM("<-- Loaded " << cloud->width * cloud->height << " data points ");
    
    // view 
    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    viewer.showCloud (cloud);
    while (!viewer.wasStopped()){}
    return (0);
}
