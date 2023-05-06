#ifndef FramesFusion_H
#define FramesFusion_H

#include <string>
#include <ctime>
#include <iostream>
#include <cmath>
#include <unordered_map>
#include <algorithm>
#include <mutex>

#include <thread>

//ros related
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//pcl related
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>         
#include <pcl/io/ply_io.h>         
#include <pcl/point_types.h> 
#include <pcl_conversions/pcl_conversions.h>

//project related
#include "CircularVector.h"
#include "SignedDistance.h"
#include "CIsoSurface.h"
#include "MeshOperation.h"
#include "HashVoxeler.h"
#include "tools/CloudVector.h"
#include "tools/OutputUtils.h"

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

pcl::PointCloud<pcl::PointNormal>::Ptr AllCloud(pcl::PointCloud<pcl::PointNormal>& cloud_vector);

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
    FramesFusion(ros::NodeHandle & node, ros::NodeHandle & nodeHandle);
    virtual void LazyLoading();

    //Destructor
    virtual ~FramesFusion();

    //Reads and verifies the ROS parameters.
    bool ReadLaunchParams(ros::NodeHandle & nodeHandle);  

    //*************main function*************
    //handle the ground point clouds topic
    void HandlePointClouds(const sensor_msgs::PointCloud2 & vCloudRosData);

    //handle the trajectory information
    void HandleTrajectory(const nav_msgs::Odometry & oTrajectory);
    //multiple thread version
    void HandleTrajectoryThread(const nav_msgs::Odometry & oTrajectory);

    // Build surface models based on new points that received from ros
    virtual void SlideModeling(pcl::PolygonMesh & oResultMesh, const int iFrameId);
    

////////////
    void SamplePoints(const pcl::PointCloud<pcl::PointXYZ> & vCloud, pcl::PointCloud<pcl::PointXYZ> & vNewCloud, int iSampleNum, bool bIntervalSamp = true);

    //get the nearby point for reconstruction
    void GetNearClouds(float fNearLengths);

    //compute the Euclidean distance between two points
    float EuclideanDistance(const pcl::PointXYZ & oBasedP, const pcl::PointNormal & oTargetP);

    //get the nearby point clouds
    void NearbyClouds(const pcl::PointCloud<pcl::PointNormal> & pRawCloud, const pcl::PointXYZ & oBasedP, pcl::PointCloud<pcl::PointNormal> & pNearCloud, float fLength);
    void NearbyClouds(CloudVector & pRawCloud, const pcl::PointXYZ & oBasedP, pcl::PointCloud<pcl::PointNormal> & pNearCloud, float fLength);
    
    //get the nearby point clouds and delete the geted point in rawCloud
    void ExtractNearbyClouds(pcl::PointCloud<pcl::PointNormal> & pRawCloud, const pcl::PointXYZ & oBasedP, pcl::PointCloud<pcl::PointNormal> & pNearCloud, float fLength);
    void ExtractNearbyClouds(CloudVector & pRawCloud, const pcl::PointXYZ & oBasedP, pcl::PointCloud<pcl::PointNormal> & pNearCloud, float fLength);

    //build surrounding models
    void SurroundModeling(const pcl::PointXYZ & oBasedP, pcl::PolygonMesh & oCBModel, const int iFrameId);
    // void SurroundModelingWithPointProcessing(const pcl::PointXYZ & oBasedP, pcl::PolygonMesh & oCBModel, const int iFrameId);

    void FusionNormalBackToPoint(const pcl::PointCloud<pcl::PointNormal>& pNearCloud, pcl::PointCloud<pcl::PointNormal> & pRawCloud, int offset, int point_num);
    void FusionNormalBackToPoint(const pcl::PointCloud<pcl::PointNormal>& pNearCloud, CloudVector & pRawCloud, int offset, int point_num);

    //*************Output function*************
    //publish point clouds
    template<class T>
    void PublishPointCloud(const pcl::PointCloud<T> & vCloud);
    
    //reload, publish point clouds with labels
    void PublishPointCloud(const pcl::PointCloud<pcl::PointXYZ> & vCloud, const std::vector<float> & vFeatures);

    //publish debug clouds for self defined topic
    template<class T>
    void PublishPointCloud(const pcl::PointCloud<T> & vCloud, const std::vector<float> & vFeatures, const std::string sTopicName);

    //publish meshes
    void PublishMeshs(const pcl::PolygonMesh & oMeshModel);

    //output point cloud for test
    void OutputPCFile(const pcl::PointCloud<pcl::PointXYZ> & vCloud, bool bAllRecord = false);

    //reload, output point cloud with given feature for test
    void OutputPCFile(const pcl::PointCloud<pcl::PointXYZ> & vCloud, const std::vector<float> & vFeatures, bool bAllRecord = false);

protected:

    //***file related***
    std::string m_sFileHead;

    //wether output evaluate files
    bool m_bOutputFiles;

    //full name of output txt that records the point clouds
    std::stringstream m_sOutPCFileName; 

    //whether the file is generated or not
    bool m_bOutPCFileFlag;

    //ouput file
    std::ofstream m_oOutPCFile;

    //***for input point cloud topic***
    //the m_oCloudSuber subscirber is to hear input point cloud topic
    ros::Subscriber m_oCloudSuber;
    //the name of input point cloud topic 
    std::string m_sInCloudTopic; 

    //***for input odom topic***
    //the m_oOdomSuber subscirber is to hearinput  odometry topic
    ros::Subscriber m_oOdomSuber;
    //the name of input odometry topic (robot trajectory)
    std::string m_sInOdomTopic;

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
    // std::mutex m_mPCNMutex;
    pcl::PointCloud<pcl::PointNormal> m_vMapPCN;
    pcl::PointCloud<pcl::PointNormal> m_vMapPCNAdded;
    pcl::PointCloud<pcl::PointNormal> m_vMapPCNTrueAdded;

    bool m_bUseAdditionalPoints;

    //features of map point clouds
    //Features can be specified
    std::vector<float> m_vMapPCFeas;

    // Reconstruct time statics
    double m_dAverageReconstructTime;
    double m_dMaxReconstructTime;
    int m_iReconstructFrameNum;

    // data association switch
    bool m_bSurfelFusion;
    // viewpoint and current frame for surfel fusion
    // void SurfelFusion(pcl::PointNormal oLidarPos, pcl::PointCloud<pcl::PointNormal>& vDepthMeasurementCloud);
    // more strict when filtering the points
    // void AddedSurfelFusion(pcl::PointNormal oLidarPos, pcl::PointCloud<pcl::PointNormal>& vDepthMeasurementCloud);
    // viewpoint and current frame for surfel fusion - multi-thread
    void SurfelFusionCore(pcl::PointNormal oLidarPos, pcl::PointCloud<pcl::PointNormal>& vDepthMeasurementCloud, pcl::PointCloud<pcl::PointNormal>& vPointCloudBuffer);
    virtual void SurfelFusionQuick(pcl::PointNormal oLidarPos, pcl::PointCloud<pcl::PointNormal>& vDepthMeasurementCloud);
    virtual void UpdateOneFrame(const pcl::PointNormal& oViewPoint, pcl::PointCloud<pcl::PointNormal>& vFilteredMeasurementCloud);

    bool m_bAsyncReconstruction;

    // data associate time statics
    double m_dAverageFusionTime;
    double m_dMaxFusionTime;
    int m_iFusionFrameNum;

    // simple to publish in a new topic
    ros::NodeHandle& m_oGlobalNode;
    ros::NodeHandle& m_oNodeHandle;
    std::unordered_map<std::string, ros::Publisher> m_vDebugPublishers;

    ros::Rate m_OdomLoopRate;

    // add for hash fusion
    // std::mutex m_mNewPointMutex;
    // pcl::PointCloud<pcl::PointNormal> m_vNewPoints; //
    HashVoxeler m_oVoxeler;

    // meshing params
	int m_iKeepTime;
	int m_iConvDim;
	int m_iConvAddPointNumRef;
	float m_fConvFusionDistanceRef1;

    // use union set to judge connection
    bool m_bUseUnionSetConnection;
    bool m_bOnlyMaxUnionSet;

    // // sdf
    // SignedDistance* m_pSdf;
    // std::mutex m_mSdfMutex;
};

#endif











