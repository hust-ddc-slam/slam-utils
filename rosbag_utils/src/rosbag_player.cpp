#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


#include <csignal>
#include <iostream>
#include <vector>

using namespace std;

#define LiDAR_Frequency (10)
#define IMU_Frequency (100)

typedef sensor_msgs::PointCloud2 LidarType;
typedef sensor_msgs::Imu ImuType;

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloudType;
typedef sensor_msgs::PointCloud2 RosPointCloudType;


bool flg_exit = false;

void SigHandle(int sig){
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
}


int main(int argc, char **argv){

    ros::init(argc, argv, "rosbag_player");
    ros::NodeHandle nh("~");

    string bag_file("/default");
    string lidar_topic("/ouster/points"), imu_topic("/ouster/imu");        // original rosbag data
    string pointcloud_topic("/pointcloud");     // convert to ros PointCloud2 for rviz view
    int flag_show_pointcloud = 0;

    nh.getParam("bag_file", bag_file);
    nh.getParam("lidar_topic", lidar_topic);
    nh.getParam("imu_topic", imu_topic);
    nh.getParam("pointcloud_topic", pointcloud_topic);
    nh.getParam("show_pointcloud", flag_show_pointcloud);

    ros::Publisher pub_lidar = nh.advertise<LidarType>(lidar_topic, 10);
    ros::Publisher pub_imu = nh.advertise<ImuType>(imu_topic, 200);
    ros::Publisher pub_pointcloud2 = nh.advertise<RosPointCloudType>(pointcloud_topic, 10);

    ROS_WARN("Settings: ");
    ROS_INFO_STREAM("Rosbag from: " << bag_file);
    ROS_INFO_STREAM("Lidar topic: " << lidar_topic);
    ROS_INFO_STREAM("IMU topic  : " << imu_topic);
    ROS_INFO_STREAM("Show ROS PC: " << flag_show_pointcloud);
    ROS_INFO_STREAM("Output PC  : " << pointcloud_topic);

    rosbag::Bag bag;
    bag.open(bag_file, rosbag::bagmode::Read);
    if(!bag.isOpen()){
        ROS_ERROR("Cannot find rosbag: ");
        return -1;
    }

    // 设置过滤器读取lidar和IMU数据
    std::vector<std::string> topics;
    topics.push_back(lidar_topic);
    topics.push_back(imu_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    ROS_INFO_STREAM("--> Loading from bag: " << bag_file);

    // get bag times
    ros::Time bag_begin_time = view.getBeginTime();
    ros::Time bag_end_time = view.getEndTime();
    double duration = (bag_end_time-bag_begin_time).toSec();
    
    // create vector to save lidar/imus
    vector<LidarType> lidars;
    vector<ImuType> imus;
    lidars.reserve((int)(duration+1)*LiDAR_Frequency);
    imus.reserve((int)(duration+1)*IMU_Frequency);

    // load data
    for (rosbag::View::iterator it = view.begin(); it != view.end(); ++it) {
        const rosbag::MessageInstance& msg = *it;
        if (msg.isType<ImuType>()) {
            ImuType::ConstPtr imu = msg.instantiate<ImuType>();
            imus.push_back(*imu);
        }
        if(msg.isType<LidarType>()){
            LidarType::ConstPtr lidar = msg.instantiate<LidarType>();
            lidars.push_back(*lidar);
        }
    }
    ROS_INFO_STREAM("<-- Loaded. Lidar: " << lidars.size() <<", IMU: " << imus.size());


    // Play back the datas.
    int lidar_idx = 0, imu_idx = 0;
    bool is_first_pub = true;
    double first_scan_time = 0.0;
    while (lidar_idx < lidars.size() && imu_idx < imus.size()){
        // publish lidar(livox CustomMsg) for other methods
        const LidarType& scan = lidars[lidar_idx];
        double scan_time = scan.header.stamp.toSec();
        pub_lidar.publish(scan);
        if(is_first_pub){
            is_first_pub = false;
            first_scan_time = scan.header.stamp.toSec();
            std::cout << "----------------------------------" << endl;
            ROS_INFO_STREAM("lidar frame_id is: " << scan.header.frame_id);
            std::cout << std::fixed << "first scan time: " << first_scan_time << std::endl;
        }


        int imu_cnt = 0;
        while(imus[imu_idx].header.stamp.toSec()<scan_time && imu_idx<imus.size()){
            pub_imu.publish(imus[imu_idx]);
            imu_idx++;
            imu_cnt++;
        }
        cout << "--> Publish scan: " << lidar_idx << ", at time:  " << (scan_time-first_scan_time) << "s, and imu number: " << imu_cnt << endl;
        lidar_idx++;
        // int key = -1;
        // while(key==-1){
        //     key = std::cin.get();
        //     if(key == EOF){
        //         std::cin.clear();
        //         std::cin.ignore();
        //         key = -1;
        //     }        ROS_INFO_STREAM("Key is: " << key);
        // }
        int key = std::cin.get();
        if(key == 'q' || key=='s')          // stop with input: 's' or 'q'
            break;
    }

    bag.close();
    return 0;
}
