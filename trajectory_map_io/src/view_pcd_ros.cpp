
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



int main(int argc, char **argv) {
    ros::init(argc, argv, "ros_viewer");
    ros::NodeHandle nh("~");

    string map_folder("/default-folder");
    const int max_pc_number = 5;
    int number_of_pc(0);
    
    
    nh.getParam("map_folder", map_folder);
    nh.getParam("pointcloud_number", number_of_pc);
    ros::Publisher pub_pc[max_pc_number];
    string pc_filename[max_pc_number], pc_fullname[max_pc_number];
    for(int i=0; i<number_of_pc; ++i){
        string name("default-name");
        nh.getParam("pc" + std::to_string(i), name);
        pc_filename[i] = name;
        pub_pc[i] = nh.advertise<sensor_msgs::PointCloud2>("/" + name, 1);
    }

    ROS_WARN("Settings: ");
    ROS_INFO_STREAM("Map folder: " << map_folder);
    ROS_INFO_STREAM("Num of PC : " << number_of_pc);
    for (int i = 0; i < number_of_pc; ++i){
        pc_fullname[i] = map_folder + pc_filename[i] + ".pcd";
        ROS_INFO_STREAM(" Files " << i << ": " << pc_fullname[i]);
    }

    std::vector<PointCloudType::Ptr> clouds_ptr;
    for (int i = 0; i < number_of_pc; ++i){
        PointCloudType::Ptr cloud(new PointCloudType);
        ROS_INFO_STREAM("--> Loading PCD " << i <<" from: " << pc_fullname[i]);
        if (pcl::io::loadPCDFile<PointType> (pc_fullname[i], *cloud) == -1){
            ROS_ERROR("Couldn't load pcd file.");
            return (-1);
        }
        ROS_INFO_STREAM("<-- Loaded " << cloud->width * cloud->height << " data points ");
        clouds_ptr.push_back(cloud);
    }

    ROS_WARN_STREAM("<== Loaded done. Start to publish all PC");
    ros::Rate rate(10);
    while (ros::ok()){
        for(int i=0; i<number_of_pc; ++i){
            pubRosMsg(*clouds_ptr[i], pub_pc[i]);
        }
        ros::spinOnce();
    }
    return (0);
}
