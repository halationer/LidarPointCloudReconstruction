#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/file_io.h>
#include <sensor_msgs/PointCloud2.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_helper");
    ros::NodeHandle n("~");
    std::string dataset_folder, output_bag_file;
    n.getParam("dataset_folder", dataset_folder);
    std::cout << "Reading sequence from " << dataset_folder << '\n';

    std::string poses_path = "poses.txt";
    std::ifstream poses_file(dataset_folder + poses_path, std::ifstream::in);

    bool to_bag;
    n.getParam("to_bag", to_bag);
    if (to_bag)
        n.getParam("output_bag_file", output_bag_file);
    int publish_delay;
    n.getParam("publish_delay", publish_delay);
    publish_delay = publish_delay <= 0 ? 1 : publish_delay;

    ros::Publisher pub_laser_cloud = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 2);

    rosbag::Bag bag_out;
    if (to_bag)
        bag_out.open(output_bag_file, rosbag::bagmode::Write);

    std::string line;
    std::size_t line_num = 0;

    ros::Rate r(10.0 / publish_delay);
    while (std::getline(poses_file, line) && ros::ok())
    {
        std::cout << "seq: " << line_num << " | ";
      
        // read lidar point cloud
        std::stringstream lidar_data_path;
        lidar_data_path << dataset_folder << "pcd/"
                        << std::setfill('0') << std::setw(5) << line_num << ".pcd";
        pcl::PointCloud<pcl::PointXYZ> pc;
        sensor_msgs::PointCloud2 smpc;
        pcl::io::loadPCDFile(lidar_data_path.str(), pc);
        pcl::toROSMsg(pc, smpc);
        std::cout << "totally " << pc.size() << " points in this lidar frame \n";


        sensor_msgs::PointCloud2& laser_cloud_msg = smpc;
        laser_cloud_msg.header.stamp = ros::Time::now();
        laser_cloud_msg.header.frame_id = "/camera_init";
        pub_laser_cloud.publish(laser_cloud_msg);

        if (to_bag)
        {
            bag_out.write("/velodyne_points", ros::Time::now(), laser_cloud_msg);
        }

        line_num ++;
        r.sleep();
    }
    bag_out.close();
    std::cout << "Done \n";


    return 0;
}