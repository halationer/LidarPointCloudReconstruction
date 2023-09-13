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

#include <pcl/io/pcd_io.h>         
#include <pcl/io/ply_io.h>         
#include <pcl/point_types.h> 
#include <pcl_conversions/pcl_conversions.h>

//polygon related
#include <visualization_msgs/Marker.h>
#include <shape_msgs/Mesh.h>

//project related
#include "GHPR.h"
#include "SectorPartition.h"
#include "ExplicitRec.h"
#include "CircularVector.h"
#include "MeshSample.h"
#include "tools/DebugManager.h"

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
	pcl::PointXYZI oLocation;

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

  //For lazy loading
  virtual void LazyLoading();

  //Reads and verifies the ROS parameters.
  virtual bool ReadLaunchParams(ros::NodeHandle & nodeHandle);  

  void SamplePoints(const pcl::PointCloud<pcl::PointXYZI> & vCloud, pcl::PointCloud<pcl::PointXYZI> & vNewCloud, int iSampleNum, bool bIntervalSamp = true);

  //*************handler function*************
  //handle the trajectory information
  virtual void HandleTrajectory(const nav_msgs::Odometry & oTrajectory);

  //handle the ground point clouds topic
  virtual void HandlePointClouds(const sensor_msgs::PointCloud2 & vCloudRosData);

  //*************Output function*************
  //publish point clouds
  void PublishPointCloud(const pcl::PointCloud<pcl::PointXYZI> & vCloud);
  //reload, publish point clouds with labels
  void PublishPointCloud(const pcl::PointCloud<pcl::PointXYZI> & vCloud, const std::vector<float> & vFeatures);
   //reload, publish point clouds with labels
  void PublishPointCloud(const pcl::PointCloud<pcl::PointNormal> & vCloudNormal);

  template<class T>
  void PublishPointCloud(pcl::PointCloud<T>& pointcloud, ros::Publisher& publisher);

  //publish meshes
  void PublishMeshs();
  void PublishMeshForAlgorithm();

  //*******odom related*******
  //Trajectory line interpolation
  void InterpolateTraj(const RosTimePoint & oCurrent, const RosTimePoint & oPast, const float& fRatio,
  pcl::PointXYZI & oInter);

  //Query the nearest trajectory point
  pcl::PointXYZI ComputeQueryTraj(const ros::Time & oQueryTime);

  //output point cloud for test
  void OutputPCFile(const pcl::PointCloud<pcl::PointXYZI> & vCloud, bool bAllRecord = false);

  //reload, output point cloud with given feature for test
  void OutputPCFile(const pcl::PointCloud<pcl::PointXYZI> & vCloud, const std::vector<float> & vFeatures, bool bAllRecord = false);


 protected:

  //***file related***
  std::string m_sFileHead;

  //full name of output txt that records the point clouds
  std::stringstream m_sOutPCFileName; 

  //whether the file is generated or not
  bool m_bOutPCFileFlag;
  
  //ouput file
  std::ofstream m_oOutPCFile;

  //***for input odom topic***
  //the m_oOdomSuber subscirber is to hearinput  odometry topic
  ros::Subscriber m_oOdomSuber;
  //the name of input odometry topic (robot trajectory), 在 reconstruction.launch 中赋值 /slam_odom
  std::string m_sInOdomTopic;

  unsigned int m_iTrajFrameNum;

  //***for input point cloud topic***
  //the m_oCloudSuber subscirber is to hear input point cloud topic
  ros::Subscriber m_oCloudSuber;
  //the name of input point cloud topic, 在 reconstruction.launch 中赋值 /slam_points
  std::string m_sInCloudTopic; 


  //***for output cloud topic***
  //output point cloud topic, 在 reconstruction.launch 中赋值 /processed_cloud 目前该节点不存在
  std::string m_sOutCloudTopic;
  //output point cloud frame
  std::string m_sOutCloudTFId;
  //point cloud publisher for test
  ros::Publisher m_oCloudPublisher;

  //***for output mesh***
  //output point cloud topic, 在 reconstruction.launch 中赋值 /frame_meshs，单帧网格的输出
  std::string m_sOutMeshTopic;
  //output point cloud frame
  std::string m_sOutMeshTFId;
  //polygon publisher for test
  ros::Publisher m_oMeshPublisher;

  std::string m_sOutMeshAlgoTopic;
  ros::Publisher m_oMeshAlgoPublisher;

  //displayed point topic
  std::string m_sAdditionalPointTopic = "/additional_points";
  ros::Publisher m_oAdditionalPointPublisher;

  //frame sampling
  int m_iFrameSmpNum;

  //sampling number of input point clouds
  int m_iSampleInPNum;

  //sampling number of sector
  int m_iSectorNum;

  // Lidar ids config, stored in the intensity of point-cloud
  int m_iLidarLineMin;
  int m_iLidarLineMax;

  //**frenquency related**

  float m_fViewZOffset;//z offset of odom to lidar sensor
  
  //explicit reconstruction
  ExplicitRec m_oExplicitBuilder;

  //circle vector of odom
  CircularVector<RosTimePoint> m_vOdomHistory;

  //How many frames of point cloud have been calculated cumulatively
  unsigned int m_iPCFrameCount;

  //frame count
  unsigned int m_iTrajCount;

  //map point clouds with normals
  //accumulated processed point cloud
  pcl::PointCloud<pcl::PointXYZI> m_vMapPCN;

  //features of map point clouds
  //Features can be specified
  std::vector<float> m_vMapPCFeas;

  double m_dAverageReconstructTime;
  double m_dMaxReconstructTime;

  int m_iReconstructFrameNum;
  int m_iTotalFrameNum;

  // For lazy node loading mode
  ros::NodeHandle& node;
  ros::NodeHandle& nodeHandle;
  
  bool m_bOutputFiles;

  TimeDebugger timer;
};

std::ostream& operator<<(std::ostream& out, const sensor_msgs::PointCloud2::_header_type& header);

#endif











