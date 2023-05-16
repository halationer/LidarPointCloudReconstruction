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
#include <pcl/io/ply_io.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/pcl_macros.h>

namespace pcl {
    struct PointDoubleXYZ
    {
        union EIGEN_ALIGN64 { \
            double data[4]; \
            struct { \
            double x; \
            double y; \
            double z; \
            }; \
        };
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
    } EIGEN_ALIGN64;                    // enforce SSE padding for correct memory alignment


};

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointDoubleXYZ,           // here we assume a XYZ + "test" (as fields)
                                (double, x, x)
                                (double, y, y)
                                (double, z, z)
)

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_helper");
    ros::NodeHandle n("~");
    std::string pose_file, pc_folder, output_bag_file;
    n.getParam("pose_file", pose_file);
    n.getParam("pc_folder", pc_folder);
    std::cout << "Reading sequence from " << pc_folder << '\n';

    std::ifstream poses_file(pose_file, std::ifstream::in);
    
    bool reverse = false;
    n.getParam("reverse", reverse);
    std::ofstream poses_file_re(pose_file+".re", std::ofstream::out);
    std::vector<std::string> lines;

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
    int line_num = 0;
    n.getParam("start_num", line_num);


    ros::Rate r(10.0 / publish_delay);
    while (std::getline(poses_file, line) && ros::ok())
    {
        lines.push_back(line);
        std::cout << "seq: " << line_num << " | ";
      
        // read lidar point cloud
        std::stringstream lidar_data_path;
        lidar_data_path << pc_folder << std::setfill('0') << std::setw(6) << line_num << ".ply";
        pcl::PointCloud<pcl::PointDoubleXYZ> pc_double;
        pcl::PointCloud<pcl::PointXYZ> pc;
        sensor_msgs::PointCloud2 smpc;
        pcl::io::loadPLYFile(lidar_data_path.str(), pc_double);
        for(auto&& point : pc_double.points) pc.points.push_back(pcl::PointXYZ(point.x, point.y, point.z));
        pcl::toROSMsg(pc, smpc);
        std::cout << "totally " << pc.size() << " points in this lidar frame \n";


        sensor_msgs::PointCloud2& laser_cloud_msg = smpc;
        laser_cloud_msg.header.stamp = ros::Time::now();
        laser_cloud_msg.header.frame_id = "/haisai";
        pub_laser_cloud.publish(laser_cloud_msg);

        if (to_bag)
        {
            bag_out.write("/velodyne_points", ros::Time::now(), laser_cloud_msg);
        }

        if(reverse) line_num--;
        else line_num++;
        r.sleep();
    }
    bag_out.close();
    std::cout << "Done \n";

    for(auto iter = lines.rbegin(); iter != lines.rend(); ++iter)
        poses_file_re << *iter;

    poses_file.close();
    poses_file_re.close();

    return 0;
}