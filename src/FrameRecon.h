#ifndef FRAMERECON_H
#define FRAMERECON_H

#include <string>
#include <ctime>

//ros related
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>

//pcl related
#include "pcl_ros/transforms.h"  
#include <pcl_conversions/pcl_conversions.h>

//polygon related
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>

//project related
#include "GHPR.h"
#include "SectorPartition.h"
#include "ExplicitRec.h"
#include "CircularVector.h"

// Trajectory state data. 
struct RosTimePoint{

	// The time of the measurement leading to this state (in seconds). //
	ros::Time oTimeStamp;

	//********angles***********
	//float roll;    ///< The current roll angle. 
	//float pitch;    ///< The current pitch angle.
	//float yaw;   ///< The current yaw angle. 

	//********coordinate value****************
	// The global trajectory position in 3D space. 
	pcl::PointXYZ oLocation;

};

//******************************************************************
// this class below is to compute topological guidance map based on SLAM
//
// this class need input of data below:
// 1. odometry position
// 2. ground point clouds
// 3. obstacle point clouds
// 4. boundary point clouds 
//
// the output of this class is below:
// 1. the next best view position
// 
// created and edited by Huang Pengdi, 2022.05.03
// Email: alualu628628@gmail.com
//******************************************************************

class FrameRecon{

 public:

  //*************Initialization function*************
  //Constructor
  FrameRecon(ros::NodeHandle & node,
              ros::NodeHandle & nodeHandle);

  //Destructor
  virtual ~FrameRecon();

  //Reads and verifies the ROS parameters.
  bool ReadLaunchParams(ros::NodeHandle & nodeHandle);  

  void SamplePoints(const pcl::PointCloud<pcl::PointXYZ> & vCloud, pcl::PointCloud<pcl::PointXYZ> & vNewCloud, int iSampleNum, bool bIntervalSamp = true);

  //*************handler function*************
  //handle the trajectory information
  //ConfidenceValue calculation is triggered as soon as receving odom data
  void HandleTrajectory(const nav_msgs::Odometry & oTrajectory);

  //handle the ground point clouds topic
  void HandlePointClouds(const sensor_msgs::PointCloud2 & vCloudRosData);

  //*************Output function*************
  //publish point clouds
  void PublishPointCloud(pcl::PointCloud<pcl::PointXYZ> & vCloud);

  //*******odom related*******
  //Trajectory line interpolation
  void InterpolateTraj(const RosTimePoint & oCurrent, const RosTimePoint & oPast, const float& fRatio,
  pcl::PointXYZ & oInter);

  //Query the nearest trajectory point
  pcl::PointXYZ ComputeQueryTraj(const ros::Time & oQueryTime);


 private:

  //***for input odom topic***
  //the m_oOdomSuber subscirber is to hearinput  odometry topic
  ros::Subscriber m_oOdomSuber;

  //the name of input odometry topic (robot trajectory)
  std::string m_sOdomTopic;

  unsigned int m_iTrajFrameNum;

  //***for input point cloud topic***
  //the m_oCloudSuber subscirber is to hear input point cloud topic
  ros::Subscriber m_oCloudSuber;

  //the name of input point cloud topic 
  std::string m_sCloudTopic; 


  //How many frames of point cloud have been calculated cumulatively
  unsigned int m_iPCFrameCount;

  //frame sampling
  int m_iFrameSmpNum;

  //sampling number of input point clouds
  int m_iSampleInPNum;

  //sampling number of sector
  int m_iSectorNum;
  
  //***for output point cloud topic***
  //point cloud publisher for test
  ros::Publisher m_oCloudPublisher;

  //**frenquency related**

  float m_fViewZOffset;//z offset of odom to lidar sensor
  
  //the frame count of trajectory point


  //explicit reconstruction
  ExplicitRec m_oExplicitBuilder;

  //circle vector of odom
  CircularVector<RosTimePoint> m_vOdomHistory;



  
};




#endif











