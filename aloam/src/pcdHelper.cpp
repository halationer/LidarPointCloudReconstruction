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

class FileNameGetter {

private:
    enum GetFileNameMethod {
        by_number   = 1,
        by_filename = 2,
    };

    std::string dataset_folder;
    std::string ref_file_path;
    std::ifstream ref_file;
    enum GetFileNameMethod method;

public:
    std::size_t line_num = 0;

    FileNameGetter(const std::string& dataset_folder, const std::string& ref_file_path, int method) : 
        dataset_folder(dataset_folder), ref_file_path(ref_file_path), method(GetFileNameMethod(method)) {

        std::cout << "open ref file: " << (dataset_folder + ref_file_path) << std::endl;
        ref_file.open(dataset_folder + ref_file_path, std::ifstream::in);
    }

    ~FileNameGetter() {
        ref_file.close();
    }

    bool NextFileName(std::string& file_name) {

        std::string line;

        if(!std::getline(ref_file, line)) return false;
        
        std::stringstream lidar_data_path;
        switch(method) {

            case by_number:
                lidar_data_path << dataset_folder << "pcd/"
                                << std::setfill('0') << std::setw(5) << line_num << ".pcd";
                break;

            case by_filename:
                lidar_data_path << dataset_folder << line;
                break;
        }
        file_name = lidar_data_path.str();

        ++line_num;

        return true;
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_helper");
    ros::NodeHandle n("~");
    std::string dataset_folder, output_bag_file;
    n.getParam("dataset_folder", dataset_folder);
    if(dataset_folder.back() != '/') dataset_folder += "/";
    std::cout << "Reading sequence from " << dataset_folder << '\n';

    int file_name_method;
    n.getParam("file_name_method", file_name_method);

    std::string ref_path;
    n.getParam("ref_file", ref_path);

    FileNameGetter name_getter(dataset_folder, ref_path, file_name_method);

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

    std::string file_name;

    ros::Rate r(10.0 / publish_delay);
    while (name_getter.NextFileName(file_name) && ros::ok())
    {
        std::cout << "seq: " << name_getter.line_num << " | " << file_name << " | ";
      
        // read lidar point cloud
        pcl::PointCloud<pcl::PointXYZ> pc;
        sensor_msgs::PointCloud2 smpc;
        pcl::io::loadPCDFile(file_name, pc);
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

        r.sleep();
    }
    bag_out.close();
    std::cout << "Done \n";


    return 0;
}