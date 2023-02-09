#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <sensor_msgs/PointCloud2.h>

std::vector<float> read_lidar_data(const std::string lidar_data_path)
{
    std::ifstream lidar_data_file(lidar_data_path, std::ifstream::in | std::ifstream::binary);
    lidar_data_file.seekg(0, std::ios::end);                                //指针移动到文件末尾
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);    //读取文件末尾的指针位置，相当于读取数据块的大小
    lidar_data_file.seekg(0, std::ios::beg);                                //指针回到文件开头，准备读取数据

    std::vector<float> lidar_data_buffer(num_elements);
    lidar_data_file.read(reinterpret_cast<char*>(&lidar_data_buffer[0]), num_elements*sizeof(float));   //这个cast好啊，对该对象从位模式上进行重新解释
    return lidar_data_buffer;
}

// calculate intensity (copy aloam)
template<class PointCloud>
void calculate_intensity(PointCloud & laser_cloud) {
    
    const int cloudSize = laser_cloud.size();
    constexpr double scanPeriod = 0.1;
    constexpr int N_SCANS = 64;
    bool halfPassed = false;
    int count = cloudSize;
    float startOri = -atan2(laser_cloud.points[0].y, laser_cloud.points[0].x);
    float endOri = -atan2(laser_cloud.points[cloudSize - 1].y,
                        laser_cloud.points[cloudSize - 1].x) +
                2 * M_PI;

    if (endOri - startOri > 3 * M_PI)
    {
        endOri -= 2 * M_PI;
    }
    else if (endOri - startOri < M_PI)
    {
        endOri += 2 * M_PI;
    }

    for (int i = 0; i < cloudSize; i++)
    {
        pcl::PointXYZI& point = laser_cloud.points[i];
        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int scanID = 0;

        if (N_SCANS == 16)
        {
            scanID = int((angle + 15) / 2 + 0.5);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                // count--;
                // continue;
                scanID = 0;
            }
        }
        else if (N_SCANS == 32)
        {
            scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                // count--;
                // continue;
                scanID = 0;
            }
        }
        else if (N_SCANS == 64)
        {   
            if (angle >= -8.83)
                scanID = int((2 - angle) * 3.0 + 0.5);
            else
                scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

            // use [0 50]  > 50 remove outlies 
            if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
            {
                // count--;
                // continue;
                scanID = 0;
            }
        }
        else
        {
            printf("wrong scan number\n");
            ROS_BREAK();
        }
        //printf("angle %f scanID %d \n", angle, scanID);

        float ori = -atan2(point.y, point.x);
        if (!halfPassed)
        { 
            if (ori < startOri - M_PI / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > startOri + M_PI * 3 / 2)
            {
                ori -= 2 * M_PI;
            }

            if (ori - startOri > M_PI)
            {
                halfPassed = true;
            }
        }
        else
        {
            ori += 2 * M_PI;
            if (ori < endOri - M_PI * 3 / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > endOri + M_PI / 2)
            {
                ori -= 2 * M_PI;
            }
        }

        float relTime = (ori - startOri) / (endOri - startOri);
        point.intensity = scanID + scanPeriod * relTime;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kitti_helper_single_test");
    ros::NodeHandle n("~");
    std::string dataset_folder, sequence_number, output_bag_file;
    n.getParam("dataset_folder", dataset_folder);
    n.getParam("sequence_number", sequence_number);
    std::cout << "Reading sequence " << sequence_number << " from " << dataset_folder << '\n';

    // cloud output
    std::string timestamp_path = "fixed/sequences/" + sequence_number + "/times.txt";
    std::ifstream timestamp_file(dataset_folder + timestamp_path, std::ifstream::in);
    std::cout << "Load " << timestamp_path << " finish..." << std::endl;

    bool to_bag;
    n.getParam("to_bag", to_bag);
    if (to_bag)
        n.getParam("output_bag_file", output_bag_file);
    int publish_delay;
    n.getParam("publish_delay", publish_delay);
    publish_delay = publish_delay <= 0 ? 1 : publish_delay;
    
    int bin_name_width;
    n.getParam("bin_name_width", bin_name_width);

    ros::Publisher pub_laser_cloud = n.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 2);

    // bag init 
    rosbag::Bag bag_out;
    if (to_bag)
        bag_out.open(output_bag_file, rosbag::bagmode::Write);
    std::cout << "Load " << output_bag_file << " finish..." << std::endl;

    std::string line;
    std::size_t line_num = 0;

    // identity odom
    nav_msgs::Odometry odomGT;
    odomGT.header.frame_id = "/camera_init";
    Eigen::Quaterniond q(Eigen::Matrix3d::Identity());
    Eigen::Vector3d t(0, 0, 0);
    odomGT.pose.pose.orientation.x = q.x();
    odomGT.pose.pose.orientation.y = q.y();
    odomGT.pose.pose.orientation.z = q.z();
    odomGT.pose.pose.orientation.w = q.w();
    odomGT.pose.pose.position.x = t(0);
    odomGT.pose.pose.position.y = t(1);
    odomGT.pose.pose.position.z = t(2);

    // read and save
    ros::Rate r(10.0 / publish_delay);
    while (std::getline(timestamp_file, line) && ros::ok())
    {
        float timestamp = stof(line);
        std::cout << "time: " << timestamp << " | ";
      
        // read lidar point cloud
        std::stringstream lidar_data_path;
        lidar_data_path << dataset_folder << "fixed/sequences/" + sequence_number + "/velodyne/" 
                        << std::setfill('0') << std::setw(bin_name_width) << line_num << ".bin";
        std::vector<float> lidar_data = read_lidar_data(lidar_data_path.str());
        std::cout << "totally " << lidar_data.size() / 4.0 << " points in this lidar frame \n";

        pcl::PointCloud<pcl::PointXYZI> laser_cloud;
        for (std::size_t i = 0; i < lidar_data.size(); i += 4)
        {
            pcl::PointXYZI point;
            point.x = lidar_data[i];
            point.y = lidar_data[i + 1];
            point.z = lidar_data[i + 2];
            point.intensity = lidar_data[i + 3];
            laser_cloud.push_back(point);
        }

        calculate_intensity(laser_cloud);       

        sensor_msgs::PointCloud2 laser_cloud_msg;
        pcl::toROSMsg(laser_cloud, laser_cloud_msg);
        // laser_cloud_msg.header.stamp = ros::Time().fromSec(timestamp);
        laser_cloud_msg.header.stamp = ros::Time::now();
        laser_cloud_msg.header.frame_id = "/camera_init";
        pub_laser_cloud.publish(laser_cloud_msg);

        // fix odom header
        odomGT.header.stamp = ros::Time::now();

        if (to_bag)
        {
            bag_out.write("/velodyne_cloud_registered", ros::Time::now(), laser_cloud_msg);
            bag_out.write("/aft_mapped_to_init", ros::Time::now(), odomGT);
        }

        // std::stringstream ply_save_path;
        // ply_save_path << dataset_folder << "fixed/sequences/" + sequence_number + "/velodyne/" 
        //                 << std::setfill('0') << std::setw(6) << line_num << ".ply";
        // pcl::io::savePLYFile(ply_save_path.str(), laser_cloud);

        line_num ++;
        r.sleep();
    }
    bag_out.close();
    std::cout << "Done \n";


    return 0;
}