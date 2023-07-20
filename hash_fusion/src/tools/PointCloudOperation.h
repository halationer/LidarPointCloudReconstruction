#ifndef POINT_CLOUD_OPERATION
#define POINT_CLOUD_OPERATION

#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include<Eigen/Core>

class PointCloudOperation {

private:
    static PointCloudOperation instance;
    PointCloudOperation(){}
    PointCloudOperation(PointCloudOperation&)=delete;
public:
    static PointCloudOperation& GetInstance() {
        return instance;
    }


public:
    //compute the Euclidean distance between two points
    inline float EuclideanDistance(const pcl::PointXYZ & oBasedP, const pcl::PointNormal & oTargetP);

    //get the nearby point clouds
    void NearbyClouds(const pcl::PointCloud<pcl::PointNormal> & pRawCloud, const pcl::PointXYZ & oBasedP, pcl::PointCloud<pcl::PointNormal> & pNearCloud, float fLength);
    // void NearbyClouds(CloudVector & pRawCloud, const pcl::PointXYZ & oBasedP, pcl::PointCloud<pcl::PointNormal> & pNearCloud, float fLength);
    
    //get the nearby point clouds and delete the geted point in rawCloud
    void ExtractNearbyClouds(pcl::PointCloud<pcl::PointNormal> & pRawCloud, const pcl::PointXYZ & oBasedP, pcl::PointCloud<pcl::PointNormal> & pNearCloud, float fLength);
    // void ExtractNearbyClouds(CloudVector & pRawCloud, const pcl::PointXYZ & oBasedP, pcl::PointCloud<pcl::PointNormal> & pNearCloud, float fLength);
    
    void TranslatePointCloudToLocal(
        const pcl::PointNormal& oLidarPos, 
        const pcl::PointCloud<pcl::PointNormal>& vGlobalCloud,
        pcl::PointCloud<pcl::PointNormal>& vLocalCloud);
};

#endif