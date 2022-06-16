#ifndef FramesFusion_H
#define FramesFusion_H

#include <string>
#include <ctime>
#include <iostream>
#include <cmath>

//ros related
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>

//pcl related
#include "pcl_ros/transforms.h"  

#include <pcl/io/pcd_io.h>         
#include <pcl/io/ply_io.h>         
#include <pcl/point_types.h> 
#include <pcl_conversions/pcl_conversions.h>

//polygon related
#include <visualization_msgs/Marker.h>

//project related
#include "CircularVector.h"
#include "SignedDistance.h"
#include "CIsoSurface.h"
#include "MeshOperation.h"



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

class FramesFusion{

public:

//*************Initialization function*************
      //Constructor
      FramesFusion(ros::NodeHandle & node,
              ros::NodeHandle & nodeHandle);

      //Destructor
      virtual ~FramesFusion();

      //Reads and verifies the ROS parameters.
      bool ReadLaunchParams(ros::NodeHandle & nodeHandle);  

      void SamplePoints(const pcl::PointCloud<pcl::PointXYZ> & vCloud, pcl::PointCloud<pcl::PointXYZ> & vNewCloud, int iSampleNum, bool bIntervalSamp = true);

      //*************main function*************
      //handle the ground point clouds topic
      void HandlePointClouds(const sensor_msgs::PointCloud2 & vCloudRosData);

      //handle the trajectory information
      void HandleTrajectory(const nav_msgs::Odometry & oTrajectory);

      //get the nearby point for reconstruction
      void GetNearClouds(float fNearLengths);

      //compute the Euclidean distance between two points
      float EuclideanDistance(const pcl::PointXYZ & oBasedP, const pcl::PointNormal & oTargetP);

      //get the nearby point clouds
      void NearbyClouds(const pcl::PointCloud<pcl::PointNormal> & pRawCloud, const pcl::PointXYZ & oBasedP, pcl::PointCloud<pcl::PointNormal> & pNearCloud, float fLength);

      //build surrounding models
      void SurroundModeling(const pcl::PointXYZ & oBasedP, pcl::PolygonMesh & oCBModel);

      //*************Output function*************
      //publish point clouds
      void PublishPointCloud(const pcl::PointCloud<pcl::PointXYZ> & vCloud);
      //reload, publish point clouds with labels
      void PublishPointCloud(const pcl::PointCloud<pcl::PointXYZ> & vCloud, const std::vector<float> & vFeatures);

      //publish meshes
      void PublishMeshs(const pcl::PolygonMesh & oMeshModel);

      //output point cloud for test
      void OutputPCFile(const pcl::PointCloud<pcl::PointXYZ> & vCloud, bool bAllRecord = false);

      //reload, output point cloud with given feature for test
      void OutputPCFile(const pcl::PointCloud<pcl::PointXYZ> & vCloud, const std::vector<float> & vFeatures, bool bAllRecord = false);

private:

      //***file related***
      std::string m_sFileHead;

      //full name of output txt that records the point clouds
      std::stringstream m_sOutPCFileName; 

      //whether the file is generated or not
      bool m_bOutPCFileFlag;
  
      //ouput file
      std::ofstream m_oOutPCFile;

      //output point cloud with normal
      std::stringstream m_sOutPCNormalFileName; 

      //***for input point cloud topic***
      //the m_oCloudSuber subscirber is to hear input point cloud topic
      ros::Subscriber m_oCloudSuber;

      //the name of input point cloud topic 
      std::string m_sInCloudTopic; 

      //***for output cloud topic***
      //output point cloud topic
      std::string m_sOutCloudTopic;

      //output point cloud frame
      std::string m_sOutCloudTFId;

      //point cloud publisher for test
      ros::Publisher m_oCloudPublisher;

      //***for output mesh***
      //output point cloud topic
      std::string m_sOutMeshTopic;

      //output point cloud frame
      std::string m_sOutMeshTFId;

      //polygon publisher for test
      ros::Publisher m_oMeshPublisher;

      //nearby length
      float m_fNearLengths;

      //voxel resolution
      pcl::PointXYZ m_oVoxelRes;

      //frame sampling
      int m_iFrameSmpNum;

      //sampling number of input point clouds
      int m_iSampleInPNum;

      //nearby mesh update period
      float m_fNearMeshPeriod;

      //How many frames of point cloud have been calculated cumulatively
      unsigned int m_iPCFrameCount;

      //circle vector of odom
      ros::Time m_oLastModelingTime;

      //frame count
      unsigned int m_iOdomCount;

      //map point clouds with normals
      //accumulated processed point cloud
      pcl::PointCloud<pcl::PointNormal> m_vMapPCN;

      //features of map point clouds
      //Features can be specified
      std::vector<float> m_vMapPCFeas;
 
};




#endif











