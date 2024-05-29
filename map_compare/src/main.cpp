#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>



using PointT = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointT>;


// Function to calculate point to plane distance
float pointToPlaneDistance(const PointT& point, const Eigen::Vector4f& plane) {
    return std::abs(plane[0] * point.x + plane[1] * point.y + plane[2] * point.z + plane[3]) /
           std::sqrt(plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2]);
}

// Function to fit a plane using 5 nearest neighbors
Eigen::Vector4f fitPlane(const PointCloud::Ptr& cloud, const std::vector<int>& indices) {
    Eigen::MatrixXf A(indices.size(), 3);
    Eigen::VectorXf b(indices.size());
    for (size_t i = 0; i < indices.size(); ++i) {
        A(i, 0) = cloud->points[indices[i]].x;
        A(i, 1) = cloud->points[indices[i]].y;
        A(i, 2) = cloud->points[indices[i]].z;
        b(i) = -1.0;
    }
    Eigen::Vector3f plane_params = A.colPivHouseholderQr().solve(b);
    Eigen::Vector4f plane;
    plane << plane_params, 1.0;
    return plane;
}

// Function to compute average point-to-plane distance
float computeAverageDistance(const PointCloud::Ptr& source, const PointCloud::Ptr& target) {
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(target);
    float total_distance = 0.0;
    size_t cnt=0;
    for (const auto& point : source->points) {
        std::vector<int> nearest_indices(5);
        std::vector<float> nearest_distances(5);
        kdtree.nearestKSearch(point, 5, nearest_indices, nearest_distances);
        Eigen::Vector4f plane = fitPlane(target, nearest_indices);
        double dist = pointToPlaneDistance(point, plane);
        if(dist > 1)   // maybe - ground points.
          continue;
        total_distance += dist;
        cnt++;
    }
    return total_distance / cnt;
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_processor");
    ros::NodeHandle nh;

    std::string file_gt, file_slam;
    nh.getParam("file_gt", file_gt);
    nh.getParam("file_slam", file_slam);

    PointCloud::Ptr cloud_gt(new PointCloud);
    PointCloud::Ptr cloud_slam(new PointCloud);

    if (pcl::io::loadPLYFile<PointT>(file_gt, *cloud_gt) == -1) {
        ROS_ERROR("Couldn't read GT map: %s", file_gt.c_str());
        return -1;
    }
    ROS_INFO_STREAM("--> GT loaded. Size: " << cloud_gt->points.size());

    if (pcl::io::loadPLYFile<PointT>(file_slam, *cloud_slam) == -1) {
        ROS_ERROR("Couldn't read SLAM map: %s", file_slam.c_str());
        return -1;
    }
    ROS_INFO_STREAM("--> SLAM map loaded. Size: " << cloud_slam->points.size());


    // downsample the point cloud 
    double leaf_size = 0;
    nh.getParam("leaf_size", leaf_size);
    // ROS_INFO_STREAM("DS the point cloud. Leaf size: " << leaf_size);
    // pcl::VoxelGrid<PointT> voxel_filter;
    // voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    
    // voxel_filter.setInputCloud(cloud_slam);
    // PointCloud::Ptr pc_slam(new PointCloud);
    // voxel_filter.filter(*pc_slam);

    // voxel_filter.setInputCloud(cloud_gt);
    // PointCloud::Ptr pc_gt(new PointCloud);
    // voxel_filter.filter(*pc_gt);
    

    double sample_size = 0;
    nh.getParam("sample_size", sample_size);
    pcl::RandomSample<PointT> random_sample_filter;

    random_sample_filter.setSample(sample_size);

    random_sample_filter.setInputCloud(cloud_slam);
    PointCloud::Ptr pc_slam(new PointCloud);
    random_sample_filter.filter(*pc_slam);

    random_sample_filter.setInputCloud(cloud_gt);
    PointCloud::Ptr pc_gt(new PointCloud);
    random_sample_filter.filter(*pc_gt);


    // cloud_gt = pc_gt;
    // cloud_slam = pc_slam;

    ROS_INFO_STREAM("<-- DS done. Size1: " << pc_gt->points.size() <<", size2: " << pc_slam->points.size());
    
    
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(pc_slam);
    icp.setInputTarget(pc_gt);
    icp.setMaximumIterations(100); // Increase maximum iterations for higher precision
    icp.setTransformationEpsilon(1e-9); // Increase transformation epsilon for higher precision
    icp.setEuclideanFitnessEpsilon(1e-9); // Increase fitness epsilon for higher precision

    PointCloud::Ptr cloud_aligned(new PointCloud);
    ROS_INFO("--> Begin ICP...");
    icp.align(*cloud_aligned);


    if (icp.hasConverged()) {
        ROS_INFO("ICP converged.");
        ROS_INFO("Fitness score: %f", icp.getFitnessScore());

        Eigen::Matrix4f transformation = icp.getFinalTransformation();
        ROS_INFO_STREAM("Transformation matrix:\n" << transformation);

        // Calculate the average distance between the aligned cloud and the target cloud
        double ave_distance = computeAverageDistance(cloud_aligned, pc_gt);

        double d2 = computeAverageDistance(pc_gt, cloud_gt);
        ROS_INFO_STREAM("New dis: " << d2);


    } else {
        ROS_WARN("ICP did not converge.");
    }

    ros::Publisher pub_gt = nh.advertise<sensor_msgs::PointCloud2>("/map/map_gt", 1);
    ros::Publisher pub_slam = nh.advertise<sensor_msgs::PointCloud2>("/map/map_slam", 1);
    ros::Publisher pub_slam_reg = nh.advertise<sensor_msgs::PointCloud2>("/map/map_slam_reg", 1);

    ros::Rate rate(1); // 1 Hz
    while (ros::ok()) {
        sensor_msgs::PointCloud2 output_aaa;
        pcl::toROSMsg(*pc_gt, output_aaa);
        output_aaa.header.frame_id = "map";

        sensor_msgs::PointCloud2 output_bbb;
        pcl::toROSMsg(*pc_slam, output_bbb);
        output_bbb.header.frame_id = "map";

        sensor_msgs::PointCloud2 output_ccc;
        pcl::toROSMsg(*cloud_aligned, output_ccc);
        output_ccc.header.frame_id = "map";

        pub_gt.publish(output_aaa);
        pub_slam.publish(output_bbb);
        pub_slam_reg.publish(output_ccc);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

