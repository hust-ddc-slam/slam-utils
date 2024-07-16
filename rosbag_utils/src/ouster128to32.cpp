#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <vector>
#include <iostream>
#include <cmath>

int g_first = 0;

namespace ouster_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      uint32_t t;
      uint16_t reflectivity;
      uint16_t  ring;      
      uint16_t ambient;
      uint32_t range;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint16_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)


bool isValidAngle(float angle) {
    static const std::vector<float> valid_angles = {-14, -12, -10, 0, 2, 4, 6, 16};
    for (float valid_angle : valid_angles) {
        if (std::fabs(angle - valid_angle) < 0.5) { // Using a threshold to account for minor variations
            return true;
        }
    }
    return false;
}

// Function to calculate the elevation angle and filter based on the given criteria
void ouster128to32(const pcl::PointCloud<pcl::PointXYZI>& input_cloud, pcl::PointCloud<ouster_ros::Point>& output_cloud) {
    output_cloud.header = input_cloud.header; // Preserve the header information
    if(g_first == 0)
        g_first = 1;
    else
        std::abort();
    for (const auto& point : input_cloud.points) {
        float angle = atan2(point.z, sqrt(point.x * point.x + point.y * point.y)) * (180.0 / M_PI);
        // if (isValidAngle(angle)) {
        //     ouster_ros::Point new_point;
        //     new_point.x = point.x;
        //     new_point.y = point.y;
        //     new_point.z = point.z;
        //     new_point.ring = static_cast<uint16_t>(angle);
        //     output_cloud.points.push_back(new_point);
        // }
        ROS_INFO_STREAM("Angle: " << angle);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rosbag_filter");
    ros::NodeHandle nh;

    std::string folder_name = "/home/larry/data/";
    std::string bag_name = "ntu_day_02_os1_128";

    rosbag::Bag input_bag;
    input_bag.open(folder_name + bag_name + ".bag", rosbag::bagmode::Read);

    rosbag::Bag output_bag;
    output_bag.open(folder_name + bag_name + ".ouster32.bag", rosbag::bagmode::Write);


    std::vector<std::string> topics;
    topics.push_back("/os_cloud_node/imu");
    topics.push_back("/os_cloud_node/points");
    
    rosbag::View view(input_bag, rosbag::TopicQuery(topics));
    
    ROS_INFO("--> Begin to extract.");
    for (const rosbag::MessageInstance& m : view) {
        sensor_msgs::Imu::ConstPtr i = m.instantiate<sensor_msgs::Imu>();
        if (i != nullptr) {
            output_bag.write("/os_cloud_node/imu", m.getTime(), *i);
        }

        sensor_msgs::PointCloud2::ConstPtr p = m.instantiate<sensor_msgs::PointCloud2>();
        if (p != nullptr) {
            pcl::PointCloud<pcl::PointXYZI> lidar;
            pcl::fromROSMsg(*p, lidar);
            pcl::PointCloud<ouster_ros::Point> lidar_converted;
            ouster128to32(lidar, lidar_converted);
            sensor_msgs::PointCloud2 output_cloud;
            pcl::toROSMsg(lidar_converted, output_cloud);
            output_cloud.header.frame_id = p->header.frame_id;
            output_cloud.header.stamp = p->header.stamp;
            output_bag.write("/os_cloud_node/points", m.getTime(), output_cloud);
        }
    }

    input_bag.close();
    output_bag.close();
    ROS_WARN("Saved.");
    return 0;
}



    
