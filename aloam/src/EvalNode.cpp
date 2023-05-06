#include <cmath>
#include <vector>
#include <string>
#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <fstream>
#include <thread>
#include <iomanip>
#include <nav_msgs/Path.h>

const double scanPeriod = 0.1;

int N_SCANS = 0;
std::string evalPoseFile;
std::string bagFile;

ros::Publisher pubLaserCloud, pubOdomAftMapped, pubLaserAfterMappedPath;
nav_msgs::Path laserAfterMappedPath;
std::vector<Eigen::Matrix4d> poses;

double MINIMUM_RANGE = 0.1; 

int frame_count = 0;

template <typename PointT>
void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                              pcl::PointCloud<PointT> &cloud_out, float thres)
{
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;

    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
    std::vector<int> indices;

    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
    removeClosedPointCloud(laserCloudIn, laserCloudIn, MINIMUM_RANGE);


    int cloudSize = laserCloudIn.points.size();
    float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
    float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,
                          laserCloudIn.points[cloudSize - 1].x) +
                   2 * M_PI;

    if (endOri - startOri > 3 * M_PI)
    {
        endOri -= 2 * M_PI;
    }
    else if (endOri - startOri < M_PI)
    {
        endOri += 2 * M_PI;
    }
    //printf("end Ori %f\n", endOri);

    bool halfPassed = false;
    int count = cloudSize;
    PointType point;
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);
    for (int i = 0; i < cloudSize; i++)
    {
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;

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
        laserCloudScans[scanID].push_back(point); 
    }
    
    cloudSize = count;
    printf("points size %d \n", cloudSize);

    pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
    for (int i = 0; i < N_SCANS; i++)
    { 
        *laserCloud += laserCloudScans[i];
    }

    pcl::PointCloud<PointType>& pc = *laserCloud;
    Eigen::Matrix4d& pose = poses[frame_count];
    std::cout << std::setprecision(18) << std::scientific << pose << std::endl;
    for(auto& p : pc) {
        Eigen::Vector4d point(p.x, p.y, p.z, 1);
        point = pose * point;
        p.x = point.x();
        p.y = point.y();
        p.z = point.z();
    }

    sensor_msgs::PointCloud2 laserCloudOutMsg;
    pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
    laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
    laserCloudOutMsg.header.frame_id = "/camera_init";
    pubLaserCloud.publish(laserCloudOutMsg);
}

void readPoses(const std::string& fileName) {

    std::ifstream poseFile(fileName, std::ifstream::in);

    std::string line;
    while(std::getline(poseFile, line)) {

        std::stringstream pose_stream(line);
        std::string s;
        Eigen::Matrix4d gt_pose;
        for (std::size_t i = 0; i < 3; ++i)
        {
            for (std::size_t j = 0; j < 4; ++j)
            {
                std::getline(pose_stream, s, ' ');
                gt_pose(i, j) = stod(s);
            }
        }
        gt_pose.row(3) = Eigen::RowVector4d(0, 0, 0, 1);

        poses.push_back(gt_pose);
    }

    poseFile.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "EvalNode");
    ros::NodeHandle nh;
    ros::NodeHandle localnh("~");

    localnh.param<int>("scan_line", N_SCANS, 16);
    printf("scan line number %d \n", N_SCANS);
    if(N_SCANS != 16 && N_SCANS != 32 && N_SCANS != 64)
    {
        printf("only support velodyne with 16, 32 or 64 scan line!");
        return 0;
    }
    localnh.param<double>("minimum_range", MINIMUM_RANGE, 0.1);

    // read poses
    localnh.param<std::string>("eval_pose_file", evalPoseFile, "./underground_pos.txt");
    std::cout << "read_pos_file: " << evalPoseFile << std::endl;
    readPoses(evalPoseFile);
    printf("pose number %lu \n", poses.size());

    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 100);
	pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 100);
	pubLaserAfterMappedPath = nh.advertise<nav_msgs::Path>("/aft_mapped_path", 100);

    // read point clouds
    localnh.param<std::string>("bag_file", bagFile, "./underground.bag");
    std::cout << "read_bag_file" << bagFile << std::endl;
    rosbag::Bag in_bag;
    in_bag.open(bagFile, rosbag::bagmode::Read);
    rosbag::View view(in_bag, rosbag::TopicQuery("/velodyne_points"));
    for(auto pMsg = view.begin(); pMsg != view.end() && ros::ok(); ++pMsg) {


        // publish pc
        auto& msg = *pMsg;
        std::string topic = msg.getTopic();
        ros::Time time = msg.getTime();
        sensor_msgs::PointCloud2Ptr pCloud = msg.instantiate<sensor_msgs::PointCloud2>();
        std::cout << topic << " | " << time.toNSec() << " | size: " << pCloud->width << ", " << pCloud->height << std::endl;
        laserCloudHandler(pCloud);


        // fix odom
        auto& pose = poses[frame_count];
        Eigen::Quaterniond q_w_curr(pose.topLeftCorner<3, 3>());
        Eigen::Vector3d t_w_curr = pose.topRightCorner<3, 1>();


        // publish odom
        nav_msgs::Odometry odomAftMapped;
        odomAftMapped.header.frame_id = "/camera_init";
        odomAftMapped.child_frame_id = "/aft_mapped";
        odomAftMapped.header.stamp = time;
        odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
        odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
        odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
        odomAftMapped.pose.pose.orientation.w = q_w_curr.w();
        odomAftMapped.pose.pose.position.x = t_w_curr.x();
        odomAftMapped.pose.pose.position.y = t_w_curr.y();
        odomAftMapped.pose.pose.position.z = t_w_curr.z();
        pubOdomAftMapped.publish(odomAftMapped);

        geometry_msgs::PoseStamped laserAfterMappedPose;
        laserAfterMappedPose.header = odomAftMapped.header;
        laserAfterMappedPose.pose = odomAftMapped.pose.pose;
        laserAfterMappedPath.header.stamp = odomAftMapped.header.stamp;
        laserAfterMappedPath.header.frame_id = "/camera_init";
        laserAfterMappedPath.poses.push_back(laserAfterMappedPose);
        pubLaserAfterMappedPath.publish(laserAfterMappedPath);

        static tf::TransformBroadcaster br;
        tf::Transform transform;
        tf::Quaternion q;
        transform.setOrigin(tf::Vector3(t_w_curr(0),
                                        t_w_curr(1),
                                        t_w_curr(2)));
        q.setW(q_w_curr.w());
        q.setX(q_w_curr.x());
        q.setY(q_w_curr.y());
        q.setZ(q_w_curr.z());
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "/camera_init", "/aft_mapped"));
        br.sendTransform(tf::StampedTransform(tf::Transform::getIdentity(), odomAftMapped.header.stamp, "/map", "/camera_init"));

        std::this_thread::sleep_for(std::chrono::milliseconds(95));

        ++frame_count;
    }



    in_bag.close();
    return 0;
}
