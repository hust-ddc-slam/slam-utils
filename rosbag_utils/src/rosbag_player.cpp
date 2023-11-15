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

    // 初始化ROS节点
    ros::init(argc, argv, "rosbag_reader");
    ros::NodeHandle nh;

    // 创建lidar和IMU的发布者
    ros::Publisher pub_lidar = nh.advertise<LidarType>("/livox/lidar", 10);
    ros::Publisher pub_imu = nh.advertise<ImuType>("/livox/imu", 200);

    // 打开rosbag文件
    string bag_name = "/home/larrydong/fast-lio_ws/data/balcony_5th_floor_avia.bag";
    rosbag::Bag bag;
    bag.open(bag_name, rosbag::bagmode::Read);
    if(!bag.isOpen()){
        ROS_ERROR("Cannot find rosbag: ");
        return -1;
    }

    // 设置过滤器读取lidar和IMU数据
    std::vector<std::string> topics;
    topics.push_back("/livox/lidar");
    topics.push_back("/livox/imu");
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    ROS_INFO_STREAM("--> Loading from bag: " << bag_name);

    // get bag times
    ros::Time bag_begin_time = view.getBeginTime();
    ros::Time bag_end_time = view.getEndTime();
    double duration = (bag_end_time-bag_begin_time).toSec();
    
    int cnt_lidar = 0, cnt_imu = 0;

    vector<LidarType> lidars;
    vector<ImuType> imus;
    lidars.reserve((int)(duration+1)*LiDAR_Frequency);
    imus.reserve((int)(duration+1)*IMU_Frequency);

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
        if(key == 'q' || key=='s')
            break;
    }

    bag.close();
    return 0;
}
