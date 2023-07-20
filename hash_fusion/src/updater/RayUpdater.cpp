#include"RayUpdater.h"

RayUpdater RayUpdater::instance;

/**
 * @brief ray_cast method to update volume
 * @param oLidarPos lidar center pos as ray start pos
 * @param vDepthMeasurementCloud one frame point cloud, and its center is oLidarPos
 * @param oVolume the volume to be updated
 * @param bKeepVoxel wether to keep the voxel that considered to be dynamic
*/
void RayUpdater::RayFusion(
    pcl::PointNormal oLidarPos, 
    pcl::PointCloud<pcl::PointNormal>& vDepthMeasurementCloud,
    VolumeBase& oVolume,
    bool bKeepVoxel)
{
    pcl::PointCloud<pcl::PointNormal> vLocalCloud;
    m_oPcOperation.TranslatePointCloudToLocal(oLidarPos, vDepthMeasurementCloud, vLocalCloud);

    
}