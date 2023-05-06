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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kitti_helper_with_gt");
    ros::NodeHandle n("~");
    std::string dataset_folder, sequence_number, output_bag_file;
    n.getParam("dataset_folder", dataset_folder);
    n.getParam("sequence_number", sequence_number);
    std::cout << "Reading sequence " << sequence_number << " from " << dataset_folder << '\n';

    // cloud output
    std::string timestamp_path = "fixed/sequences/" + sequence_number + "/times.txt";
    std::ifstream timestamp_file(dataset_folder + timestamp_path, std::ifstream::in);

    bool to_bag;
    n.getParam("to_bag", to_bag);
    if (to_bag)
        n.getParam("output_bag_file", output_bag_file);
    int publish_delay;
    n.getParam("publish_delay", publish_delay);
    publish_delay = publish_delay <= 0 ? 1 : publish_delay;

    ros::Publisher pub_laser_cloud = n.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 2);

    // pose output
    ros::Publisher pubOdomGT = n.advertise<nav_msgs::Odometry> ("/aft_mapped_to_init", 5);
    nav_msgs::Odometry odomGT;
    odomGT.header.frame_id = "/camera_init";
    
    ros::Publisher pubPathGT = n.advertise<nav_msgs::Path> ("/path_gt", 5);
    nav_msgs::Path pathGT;
    pathGT.header.frame_id = "/camera_init";

    std::string ground_truth_path = "fixed/sequences/" + sequence_number + "/poses.txt";
    std::ifstream ground_truth_file(dataset_folder + ground_truth_path, std::ifstream::in);   

    std::string calib_path = "fixed/sequences/" + sequence_number + "/calib.txt";
    std::ifstream calib_file(dataset_folder + calib_path, std::ifstream::in);   

    Eigen::Matrix3d R_transform;
    R_transform << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    Eigen::Quaterniond q_transform(R_transform);

    // bag init 
    rosbag::Bag bag_out;
    if (to_bag)
        bag_out.open(output_bag_file, rosbag::bagmode::Write);

    std::string line;
    std::size_t line_num = 0;

    // calib
    Eigen::Matrix4d calib_matrix;
    std::string velo_to_cam;
    while(std::getline(calib_file, line)) velo_to_cam = line;
    std::stringstream vtc_stream(velo_to_cam);
    std::getline(vtc_stream, line, ' ');
    for(std::size_t i = 0; i < 3; ++i) {
        for(std::size_t j = 0; j < 4; ++j) {
            std::getline(vtc_stream, line, ' ');
            calib_matrix(i, j) = stod(line);
        }
    }
    calib_matrix.row(3) = Eigen::RowVector4d(0, 0, 0, 1);
    Eigen::Matrix4d calib_matrix_inverse = calib_matrix.inverse();
    std::cout << std::setprecision(15) << calib_matrix << std::endl << calib_matrix_inverse << std::endl;

    ros::Rate r(10.0 / publish_delay);
    while (std::getline(timestamp_file, line) && ros::ok())
    {
        float timestamp = stof(line);
        std::cout << "time: " << timestamp << " | ";

        // gt poses
        std::getline(ground_truth_file, line);
        std::stringstream pose_stream(line);
        std::string s;
        Eigen::Matrix4d gt_pose;
        for (std::size_t i = 0; i < 3; ++i)
        {
            for (std::size_t j = 0; j < 4; ++j)
            {
                std::getline(pose_stream, s, ' ');
                gt_pose(i, j) = stof(s);
            }
        }
        gt_pose.row(3) = Eigen::RowVector4d(0, 0, 0, 1);

        Eigen::Quaterniond q_w_i(gt_pose.topLeftCorner<3, 3>());
        Eigen::Quaterniond q = q_transform * q_w_i;
        q.normalize();
        Eigen::Vector3d t = q_transform * gt_pose.topRightCorner<3, 1>();

        odomGT.header.stamp = ros::Time::now();
        odomGT.pose.pose.orientation.x = q.x();
        odomGT.pose.pose.orientation.y = q.y();
        odomGT.pose.pose.orientation.z = q.z();
        odomGT.pose.pose.orientation.w = q.w();
        odomGT.pose.pose.position.x = t(0);
        odomGT.pose.pose.position.y = t(1);
        odomGT.pose.pose.position.z = t(2);
        pubOdomGT.publish(odomGT);

        geometry_msgs::PoseStamped poseGT;
        poseGT.header = odomGT.header;
        poseGT.pose = odomGT.pose.pose;
        pathGT.header.stamp = odomGT.header.stamp;
        pathGT.poses.push_back(poseGT);
        pubPathGT.publish(pathGT);
      
        // read lidar point cloud
        std::stringstream lidar_data_path;
        lidar_data_path << dataset_folder << "fixed/sequences/" + sequence_number + "/velodyne/" 
                        << std::setfill('0') << std::setw(6) << line_num << ".bin";
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

        // calculate intensity (copy aloam)
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
                    count--;
                    continue;
                }
            }
            else if (N_SCANS == 32)
            {
                scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
                if (scanID > (N_SCANS - 1) || scanID < 0)
                {
                    count--;
                    continue;
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
                    count--;
                    continue;
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

        Eigen::Matrix4d transform = gt_pose;
        // transform.block<3,3>(0, 0) = q_transform * gt_pose.topLeftCorner<3,3>();
        // transform.block<3,1>(0, 3) = q_transform * gt_pose.topRightCorner<3,1>();
        // transform.block<1,4>(3, 0) = gt_pose.row(3);
        // std::cout << transform << std::endl;
        transform = calib_matrix.inverse() * transform.eval() * calib_matrix;
        pcl::transformPointCloud(laser_cloud, laser_cloud, transform);


        sensor_msgs::PointCloud2 laser_cloud_msg;
        pcl::toROSMsg(laser_cloud, laser_cloud_msg);
        // laser_cloud_msg.header.stamp = ros::Time().fromSec(timestamp);
        laser_cloud_msg.header.stamp = ros::Time::now();
        laser_cloud_msg.header.frame_id = "/camera_init";
        pub_laser_cloud.publish(laser_cloud_msg);

        if (to_bag)
        {
            bag_out.write("/velodyne_cloud_registered", ros::Time::now(), laser_cloud_msg);
            bag_out.write("/aft_mapped_to_init", ros::Time::now(), odomGT);
            bag_out.write("/path_gt", ros::Time::now(), pathGT);
        }

        std::stringstream ply_save_path;
        ply_save_path << dataset_folder << "fixed/sequences/" + sequence_number + "/velodyne/" 
                        << std::setfill('0') << std::setw(6) << line_num << ".ply";
        pcl::io::savePLYFile(ply_save_path.str(), laser_cloud);

        line_num ++;
        r.sleep();
    }
    bag_out.close();
    std::cout << "Done \n";


    return 0;
}