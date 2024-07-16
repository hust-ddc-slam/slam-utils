#include <iostream>
#include <string>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

using namespace std;

int main(int argc, char** argv) {

    ros::init(argc, argv, "modify_frame_id");
    ros::NodeHandle nh("~");
    
    const string folder = "/media/larry/M2-SSD/PublicDataset/M2DGR-Plus/";
    string bag_name = "bridge1";
    
    string input_bag_name(folder + bag_name + ".bag");
    string output_bag_name(folder + bag_name + ".out.bag");

    string lidar_topic("/rslidar_points");
    string imu_topic("/imu");

    ROS_INFO_STREAM("Change the frameid");
    ROS_INFO_STREAM("Input bag: " << input_bag_name);
    ROS_INFO_STREAM("Outpu bag: " << output_bag_name);
    

    // Open the input bag file
    rosbag::Bag input_bag;
    input_bag.open(input_bag_name, rosbag::bagmode::Read);

    // Create the output bag file
    rosbag::Bag output_bag;
    output_bag.open(output_bag_name, rosbag::bagmode::Write);

    // Create a view for the specified topic
    std::vector<std::string> topics;
    topics.push_back(lidar_topic);
    topics.push_back(imu_topic);
    rosbag::View view(input_bag, rosbag::TopicQuery(topics));

        // Iterate through the messages on the specified topics
    for (const rosbag::MessageInstance& msg_instance : view) {
        if (msg_instance.getTopic() == lidar_topic) {
            // Get the PointCloud2 message
            sensor_msgs::PointCloud2::ConstPtr pointcloud_msg = msg_instance.instantiate<sensor_msgs::PointCloud2>();
            if (pointcloud_msg != nullptr) {
                // Modify the frame_id
                sensor_msgs::PointCloud2 modified_msg = *pointcloud_msg;
                if (modified_msg.header.frame_id == "/rslidar") {
                    modified_msg.header.frame_id = "rslidar";
                }
                // Write the modified message to the output bag
                output_bag.write(lidar_topic, msg_instance.getTime(), modified_msg);
            }
        } else if (msg_instance.getTopic() == imu_topic) {
            // Get the IMU message
            sensor_msgs::Imu::ConstPtr imu_msg = msg_instance.instantiate<sensor_msgs::Imu>();
            if (imu_msg != nullptr) {
                // Write the IMU message to the output bag
                output_bag.write(imu_topic, msg_instance.getTime(), *imu_msg);
            }
        }
    }

    // Close the bag files
    input_bag.close();
    output_bag.close();

    return 0;
}

