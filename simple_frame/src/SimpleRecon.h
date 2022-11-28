#include"FrameRecon.h"
#include"MeshSample.h"
#include"Confidence.h"

#include<pcl/io/png_io.h>
#include<pcl/surface/gp3.h>
#include<pcl/kdtree/kdtree_flann.h>

#ifndef __SIMPLE_RECONSTRUCTION__
#define __SIMPLE_RECONSTRUCTION__

//
namespace DELTA_TYPE{
    enum DELTA_TYPE {
        AVERAGE,    //平均数
        MODE,       //众数
    };
} // namespace DELTA_TYPE

namespace LIDAR_TYPE
{
    enum LIDAR_TYPE {
        __16_LINES__ = 16,
        __32_LINES__ = 32,
        __64_LINES__ = 64,
    };
} // namespace LIDAR_TYPE


class SimpleRecon : public FrameRecon {

public:
    typedef FrameRecon super;

    //************* ros init **************
    SimpleRecon(ros::NodeHandle & node, ros::NodeHandle & nodeHandle);
    virtual ~SimpleRecon();
    virtual void LazyLoading();
    virtual bool ReadLaunchParams(ros::NodeHandle & nodeHandle);  
    

    //************* handler function, subcribe callback *************
    //handle the trajectory information
    virtual void HandleTrajectory(const nav_msgs::Odometry & oTrajectory);

    //handle the ground point clouds topic
    virtual void HandlePointClouds(const sensor_msgs::PointCloud2 & vCloudRosData);

    //************* ROS Visualization ************
    template<class T>
    void PublishMesh(pcl::PointCloud<T>& pointcloud, std::vector<pcl::Vertices>& triangles);

    //******* Save Mesh to File ******
    template<class T>
    void SaveMesh(const pcl::PointCloud<T>& pointcloud, const std::vector<pcl::Vertices>& triangles);

    //************* Reconstruction ***************
    static void ReOrganizePoints(const pcl::PointCloud<pcl::PointXYZI>& in_cloud, std::vector<pcl::PointCloud<pcl::PointXYZI>>& out_cloud);
    static float GetCloudPointDelta(const std::vector<pcl::PointCloud<pcl::PointXYZI>>& in_cloud, DELTA_TYPE::DELTA_TYPE delta_type);
    static void FromCloudListToRGB(const std::vector<pcl::PointCloud<pcl::PointXYZI>>& in_cloud, std::vector<unsigned char>& out_rgb, int lidar_type);
    static void FromCloudListToDepth(const std::vector<pcl::PointCloud<pcl::PointXYZI>>& in_cloud, std::vector<unsigned char>& out_depth, int lidar_type, pcl::PointXYZI view_point);
    static void FromCloudListToPointVector(const std::vector<pcl::PointCloud<pcl::PointXYZI>>& in_cloud, std::vector<std::vector<pcl::PointXYZI>>& out_vector, int lidar_type);
    static void FromPointVectorToMesh(const std::vector<std::vector<pcl::PointXYZI>>& in_cloud, pcl::PointCloud<pcl::PointXYZI>& out_cloud, std::vector<pcl::Vertices>& out_polygons);

    //TODO：这三项或被新函数取代
    static void RecordPseudoFaces(const pcl::PointXYZI & oViewPoint, const pcl::PointCloud<pcl::PointXYZI> & vCenterPoints, const std::vector<pcl::Vertices> & vFaces, const Eigen::MatrixXf & oMatNormal, 
        std::vector<bool> & vTrueFaceStatus, std::vector<float> & vFaceWeightint, float fPseudoFaceThr = 0.05f);
    static void RemoveMeshFaces(const pcl::PointXYZI& oViewPoint, const pcl::PointCloud<pcl::PointXYZI>& in_cloud, std::vector<pcl::Vertices>& in_polygons,
        pcl::PointCloud<pcl::PointNormal>& out_cloud, std::vector<pcl::Vertices>& out_polygons, float fPseudoFaceThr = 0.05f);
    static void RemoveMeshFaces(const pcl::PointXYZI& oViewPoint, const pcl::PointCloud<pcl::PointXYZI>& in_cloud, std::vector<pcl::Vertices>& in_polygons,
        pcl::PointCloud<pcl::PointNormal>& out_cloud, std::vector<pcl::Vertices>& out_polygons, 
        pcl::PointCloud<pcl::PointXYZI>& vCenterPoints, Eigen::MatrixXf& oMatNormal, std::vector<float>& vFaceWeight, float fPseudoFaceThr = 0.05f);

    static void RecordPseudoFaces(const pcl::PointXYZI & oViewPoint, const pcl::PointCloud<pcl::PointXYZI> & vCenterPoints, const std::vector<pcl::Vertices> & vFaces, const Eigen::MatrixXf & oMatNormal, 
        std::vector<bool> & vTrueFaceStatus, std::vector<Confidence> & vFaceWeight, float fPseudoFaceThr = 0.05f);
    static void RemoveMeshFaces(const pcl::PointXYZI& oViewPoint, const pcl::PointCloud<pcl::PointXYZI>& in_cloud, std::vector<pcl::Vertices>& in_polygons,
        pcl::PointCloud<pcl::PointNormal>& out_cloud, std::vector<pcl::Vertices>& out_polygons, 
        pcl::PointCloud<pcl::PointXYZI>& vCenterPoints, Eigen::MatrixXf& oMatNormal, std::vector<Confidence>& vFaceWeight, float fPseudoFaceThr = 0.05f);

private:

    int m_iLidarType;

    bool m_bShowConfidence;

    bool m_bCombineTwoClouds;
};

#endif