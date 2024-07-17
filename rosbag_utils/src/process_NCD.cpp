
// this code merge and clean NCD rosbag.

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/filesystem.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

namespace fs = boost::filesystem;

int main(int argc, char** argv) {
    ros::init(argc, argv, "bag_merger");
    ros::NodeHandle nh;

    std::string bag_folder = "/media/larry/M2-SSD/PublicDataset/NCD/01_short_experiment/";
    
    
    rosbag::Bag output_bag;
    std::string output_bag_file = "/home/larry/data/01_short_experiment.full.bag";
    output_bag.open(output_bag_file, rosbag::bagmode::Write);

    for (fs::directory_iterator itr(bag_folder); itr != fs::directory_iterator(); ++itr) {
        if (fs::is_regular_file(itr->path()) && itr->path().extension() == ".bag") {
            std::string bag_filename = itr->path().string();
            ROS_INFO("Processing bag file: %s", bag_filename.c_str());
            rosbag::Bag input_bag;
            input_bag.open(bag_filename, rosbag::bagmode::Read);
            ROS_INFO("Bag is open.");

            std::vector<std::string> topics;
            topics.push_back("/os1_cloud_node/points");
            topics.push_back("/os1_cloud_node/imu");

            rosbag::View view(input_bag, rosbag::TopicQuery(topics));

            for (const rosbag::MessageInstance& msg : view) {
                if (msg.getTopic() == "/os1_cloud_node/points") {
                    sensor_msgs::PointCloud2::ConstPtr pointcloud_msg = msg.instantiate<sensor_msgs::PointCloud2>();
                    if (pointcloud_msg != nullptr) {
                        output_bag.write("/os1_cloud_node/points", msg.getTime(), pointcloud_msg);
                    }
                } else if (msg.getTopic() == "/os1_cloud_node/imu") {
                    sensor_msgs::Imu::ConstPtr imu_msg = msg.instantiate<sensor_msgs::Imu>();
                    if (imu_msg != nullptr) {
                        output_bag.write("/os1_cloud_node/imu", msg.getTime(), imu_msg);
                    }
                }
            }

            input_bag.close();
            ROS_INFO("This bag done.");
        }
    }

    output_bag.close();
    ROS_INFO("Merging completed. Output bag saved to: %s", output_bag_file.c_str());

    return 0;
}
