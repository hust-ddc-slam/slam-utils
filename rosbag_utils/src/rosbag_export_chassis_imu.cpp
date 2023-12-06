#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <fstream>
#include <vector>
#include "chassis_package/ChassisMsg.h" // 自定义消息的头文件


#include <csignal>
bool flg_exit = false;

// 定义用于存储数据的结构
std::vector<std::vector<float>> imu_data;
std::vector<std::vector<float>> chassis_data;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    ROS_INFO("IMU callback");
    std::vector<float> imu_row;
    imu_row.push_back(msg->linear_acceleration.x);
    imu_row.push_back(msg->linear_acceleration.y);
    imu_row.push_back(msg->linear_acceleration.z);
    imu_row.push_back(msg->angular_velocity.x);
    imu_row.push_back(msg->angular_velocity.y);
    imu_row.push_back(msg->angular_velocity.z);
    imu_data.push_back(imu_row);
}

void chassisCallback(const chassis_package::ChassisMsg::ConstPtr& msg) {
    ROS_INFO("Chassis callback");
    std::vector<float> chassis_row;
    chassis_row.push_back(msg->leftRealRPS);
    chassis_row.push_back(msg->rightRealRPS);
    chassis_data.push_back(chassis_row);
}

void saveToCSV(const std::string& filename, const std::vector<std::vector<float>>& data) {
    std::ofstream file(filename);
    for (const auto& row : data) {
        for (size_t i = 0; i < row.size(); ++i) {
            file << row[i];
            if (i < row.size() - 1) file << " ";
        }
        file << "\n";
    }
    file.close();
}



void SigHandle(int sig){
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "data_collector");
    ros::NodeHandle nh("~");

    signal(SIGINT, SigHandle);
    ros::Subscriber imu_sub = nh.subscribe("/livox/imu", 1000, imuCallback);
    ros::Subscriber chassis_sub = nh.subscribe("/chassis/data", 1000, chassisCallback);

    ros::Rate r(1000);
    while(ros::ok()){
        if (flg_exit) 
            break;
        ros::spin(); // 等待回调函数
        r.sleep();
    }
    // 保存数据到CSV文件
    saveToCSV("/home/larrydong/imu.csv", imu_data);
    saveToCSV("/home/larrydong/chassis.csv", chassis_data);
    ROS_WARN("Data saved.");

    return 0;
}

