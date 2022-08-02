#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
#include <sstream>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

int cloud_number = 0;
pcl::PointCloud<pcl::PointXYZI> saved_cloud;
std::string save_cloud_path, save_odom_path;

void saveCloudAll(const sensor_msgs::PointCloud2& cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZI> now_cloud;
    pcl::fromROSMsg(cloud_msg, now_cloud);
    saved_cloud += now_cloud;
    std::cout << "point_cloud: " << saved_cloud.size() << std::endl;
    if(saved_cloud.size() > 1000000)
    {
        std::stringstream ss;
        ss << save_cloud_path << cloud_number++ << ".pcd";
        pcl::io::savePCDFileASCII (ss.str(), saved_cloud);
        saved_cloud.clear();
        std::cout << "save cloud: " << save_cloud_path << std::endl;
    }
}

void saveCloud(const sensor_msgs::PointCloud2& cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZI> now_cloud;
    pcl::fromROSMsg(cloud_msg, now_cloud);
    std::cout << "point_cloud: " << now_cloud.size() << std::endl;

    std::stringstream ss;
    ss << save_cloud_path << cloud_number++ << ".pcd";
    pcl::io::savePCDFileASCII (ss.str(), now_cloud);
    std::cout << "save cloud: " << save_cloud_path << std::endl;
}

void saveOdometry(const nav_msgs::Path& path)
{
    auto pose = *(path.poses.end() - 1);
    auto position = pose.pose.position;

    std::stringstream ss;
    ss << save_odom_path << path.poses.size()-1 << ".txt";
    std::ofstream lidar_data_file(ss.str(), std::ofstream::out);
    lidar_data_file << position.x << " " << position.y << " " << position.z;
    lidar_data_file.close();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "save_cloud");
    ros::NodeHandle n("~");

    n.getParam("cloud_file_path", save_cloud_path);
    n.getParam("odom_file_path", save_odom_path);

    std::string save_cloud_topic;
    n.getParam("save_cloud_topic", save_cloud_topic);
    ros::Subscriber sub_cloud = n.subscribe(save_cloud_topic, 2, saveCloud);

    std::string save_odom_topic;
    n.getParam("save_odom_topic", save_odom_topic);
    ros::Subscriber sub_odom = n.subscribe(save_odom_topic, 2, saveOdometry);

    ros::spin();

    return 0;
}