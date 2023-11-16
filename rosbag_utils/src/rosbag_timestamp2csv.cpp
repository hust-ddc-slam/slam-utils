#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/foreach.hpp>

#include <iostream>
#include <fstream>
#include <string>

using namespace std;


int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "bag_to_csv");
    ros::NodeHandle nh("~");

    string csv_filename("default-csv-output"), input_bag("default-input-rosbag");
    string imu_topic("/imu/data"), lidar_topic("/velodyne_points");
    int imu_en(0), lidar_en(0);

    nh.getParam("input_bag", input_bag);
    nh.getParam("output_csv", csv_filename);
    nh.getParam("imu_topic", imu_topic);
    nh.getParam("lidar_topic", lidar_topic);
    nh.getParam("imu_en", imu_en);
    nh.getParam("lidar_en", lidar_en);

    // Open the bag file
    rosbag::Bag bag;
    try {
        bag.open(input_bag, rosbag::bagmode::Read);
    } catch (const std::exception& e) {
        std::cerr << "Error opening bag file: " << e.what() << std::endl;
        return 1;
    }

    // Set up the topics to look for
    std::vector<std::string> topics;
    topics.push_back(imu_topic);
    topics.push_back(lidar_topic);

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // Output CSV file
    std::ofstream output_csv(csv_filename);

    // Check if file is opened correctly
    if (!output_csv.is_open()) {
        std::cerr << "Could not open output CSV file." << std::endl;
        return 1;
    }

    // Write the header to the CSV
    output_csv << "# timestamp,message_type(0 for IMU, 1 for lidar)" << std::endl;

    // Process messages
    ros::Time start_time = ros::TIME_MAX;
    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
        // Determine the start time
        if (start_time == ros::TIME_MAX) {
            start_time = m.getTime();
        }
        if (m.isType<sensor_msgs::Imu>() && imu_en == 1) {
            // Calculate the timestamp relative to the first message
            double timestamp = (m.getTime() - start_time).toSec();
            // Write to the CSV file
            output_csv << timestamp << ",0" << std::endl;
        } else if (m.isType<sensor_msgs::PointCloud2>() && lidar_en == 1) {
            // Calculate the timestamp relative to the first message
            double timestamp = (m.getTime() - start_time).toSec();
            // Write to the CSV file
            output_csv << timestamp << ",1" << std::endl;
        }
    }

    // Clean up
    output_csv.close();
    bag.close();

    ROS_WARN("CSV file created successfully.");
    return 0;
}

