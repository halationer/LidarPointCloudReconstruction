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
#include "OutputUtils.h"

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

// TODO: New cloud data structure
// 目前初步测试，多帧异步没问题，多帧异步+点云融合有时候会崩溃，其他情况还未测试
// 接下来要添加的功能 
//     1.点云帧数控制，超出的帧将被剔除，并且输出到结果点云中去
//     2.不同帧的点使用不同颜色显示
//     3.如何解决回环问题？ 将删除的帧绑定一个Box，每次检测与Box的碰撞，如果有重叠区域，则将此帧重新加载（进阶选做）- 如果实现重加载则需要修改key的逻辑，不能为seq

class CloudVector {

public:

    CloudVector():dirty_flag(true),max_window_size(100),auto_release_frames(true),auto_save_frames(true) {}
    CloudVector(const CloudVector &vCloud) = delete;
    CloudVector& operator=(const CloudVector &vCloud) = delete;

    void SetMaxWindow(int window_size) { max_window_size = window_size; }
    void SetAutoRelease(bool auto_release) { auto_release_frames = auto_release; }
    void SetAutoSave(bool auto_save) { auto_save_frames = auto_save; }

    unsigned int size() {
        
        ComputeSize();

        return size_pre_sum.size() ? size_pre_sum.back() : 0;
    }
  
    friend pcl::PointCloud<pcl::PointNormal>& operator+=(pcl::PointCloud<pcl::PointNormal>& vCloudA, const CloudVector& vCloudB);    
    friend CloudVector& operator+=(CloudVector &vCloudA, const CloudVector &vCloudB);
    friend CloudVector& operator+=(CloudVector &vCloudA, const pcl::PointCloud<pcl::PointNormal> vCloudB);
    friend pcl::PointCloud<pcl::PointNormal>::Ptr AllCloud(CloudVector& cloud_vector);
    
    pcl::PointNormal& at(const int index);
    void erase(int index);

    pcl::PointNormal& operator[](const int index) {
        
        return this->at(index);
    }

    void push_back(const pcl::PointNormal& point, int seq = 0) {
        
        dirty_flag = true;
        data[seq]->push_back(point);
    }

    void ReleaseFrames();
    void SaveFrame(int seq);

private:

    void ComputeSize();

    bool dirty_flag;
    
    bool auto_release_frames;
    bool auto_save_frames;
    unsigned int max_window_size;

    std::vector<unsigned int> size_pre_sum;
    std::vector<int> seq_record;

    // key := pointcloud.header.seq | value := &pointcloud
    std::map<int, pcl::PointCloud<pcl::PointNormal>::Ptr> data;
};

pcl::PointCloud<pcl::PointNormal>::Ptr AllCloud(CloudVector& cloud_vector);
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
    //multiple thread version
    void HandleTrajectoryThread(const nav_msgs::Odometry & oTrajectory);

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
    void SurroundModelingWithPointProcessing(const pcl::PointXYZ & oBasedP, pcl::PolygonMesh & oCBModel, const int iFrameId);
    void SurroundModelingOnlyCheckOcclusion(const pcl::PointXYZ & oBasedP, pcl::PolygonMesh & oCBModel, const int iFrameId);

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

    void CheckAddedPointWithOcclusion(const pcl::PointCloud<pcl::PointNormal> & vAddedCloud, Voxelization & oVoxeler,
        const pcl::PointXYZ & oViewPoint, std::vector<int> & vTruePointIndices);

private:

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
    std::mutex m_mPCNMutex;
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
    void SurfelFusion(pcl::PointNormal oLidarPos, pcl::PointCloud<pcl::PointNormal>& vDepthMeasurementCloud);
    // more strict when filtering the points
    void AddedSurfelFusion(pcl::PointNormal oLidarPos, pcl::PointCloud<pcl::PointNormal>& vDepthMeasurementCloud);
    // viewpoint and current frame for surfel fusion - multi-thread
    void SurfelFusionQuick(pcl::PointNormal oLidarPos, pcl::PointCloud<pcl::PointNormal>& vDepthMeasurementCloud);

    bool m_bAsyncReconstruction;
    bool m_bQuickSurfelFusion;

    // data associate time statics
    double m_dAverageFusionTime;
    double m_dMaxFusionTime;
    int m_iFusionFrameNum;

    // simple to publish in a new topic
    ros::NodeHandle& m_oGlobalNode;
    ros::NodeHandle& m_oNodeHandle;
    std::unordered_map<std::string, ros::Publisher> m_vDebugPublishers;

    ros::Rate m_OdomLoopRate;
};

#endif











