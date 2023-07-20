#ifndef RAY_UPDATER
#define RAY_UPDATER

#include "volume/VolumeBase.h"
#include "tools/PointCloudOperation.h"
#include "tools/RosPublishManager.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class RayUpdater {

//singleton
private:
    static RayUpdater instance;
    RayUpdater():
        m_oPcOperation(PointCloudOperation::GetInstance()),
        m_oRpManager(RosPublishManager::GetInstance()) {}
    RayUpdater(RayUpdater&)=delete;
public:
    static RayUpdater& GetInstance() {
        return instance;
    }


public:
    void RayFusion(
        pcl::PointNormal oLidarPos, 
        pcl::PointCloud<pcl::PointNormal>& vDepthMeasurementCloud,
        VolumeBase& oVolume,
        bool bKeepVoxel);


protected:
    PointCloudOperation& m_oPcOperation;
    RosPublishManager& m_oRpManager;
};

#endif