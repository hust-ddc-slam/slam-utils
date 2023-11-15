#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <boost/foreach.hpp>
#include <livox_ros_driver/CustomMsg.h>

#include <csignal>
#include <vector>

using namespace std;

#define LiDAR_Frequency (10)
#define IMU_Frequency (200)

typedef livox_ros_driver::CustomMsg LidarType;
typedef sensor_msgs::Imu ImuType;


bool flg_exit = false;

void SigHandle(int sig){
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
}


int main(int argc, char **argv){

    ros::init(argc, argv, "rosbag_player");
    ros::NodeHandle nh("~");

    string bag_file("/default");
    string lidar_topic("/livox/lidar"), imu_topic("/livox/imu");

    nh.getParam("bag_file", bag_file);
    nh.getParam("lidar_topic", lidar_topic);
    nh.getParam("imu_topic", imu_topic);

    // 创建lidar和IMU的发布者
    ros::Publisher pub_lidar = nh.advertise<LidarType>(lidar_topic, 10);
    ros::Publisher pub_imu = nh.advertise<ImuType>(imu_topic, 200);

    ROS_WARN_STREAM("Rosbag from: " << bag_file);
    ROS_WARN_STREAM("Lidar topic: " << lidar_topic);
    ROS_WARN_STREAM("IMU topic  : " << imu_topic);

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
    while (lidar_idx < lidars.size() && imu_idx < imus.size()){
        auto lidar = lidars[lidar_idx];
        double lidar_time = lidar.header.stamp.toSec();
        pub_lidar.publish(lidars[lidar_idx]);
        int imu_cnt = 0;
        while(imus[imu_idx].header.stamp.toSec()<lidar_time && imu_idx<imus.size()){
            pub_imu.publish(imus[imu_idx]);
            imu_idx++;
            imu_cnt++;
        }
        lidar_idx++;
        cout << "-- Publish lidar at time:  " << lidar_time << ", and imu number: " << imu_cnt << endl;
        int key = std::cin.get();
        if(key == 'q' || key=='s')          // stop with input: 's' or 'q'
            break;
    }

    bag.close();
    return 0;
}
