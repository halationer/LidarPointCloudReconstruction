#ifndef PROJECT_UPDATER
#define PROJECT_UPDATER

#include "volume/VolumeBase.h"
#include "tools/PointCloudOperation.h"
#include "tools/RosPublishManager.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class ProjectUpdater {

//singleton
private:
    static ProjectUpdater instance;
    ProjectUpdater():
        m_oPcOperation(PointCloudOperation::GetInstance()),
        m_oRpManager(RosPublishManager::GetInstance()) {}
    ProjectUpdater(ProjectUpdater&)=delete;
public:
    static ProjectUpdater& GetInstance() {
        return instance;
    }


public:
    void SurfelFusionQuick(
        pcl::PointNormal oLidarPos, 
        pcl::PointCloud<pcl::PointNormal>& vDepthMeasurementCloud,
        VolumeBase& oVolume,
        bool bKeepVoxel);


private:
    void SurfelFusionCore(
        pcl::PointNormal oLidarPos, 
        pcl::PointCloud<pcl::PointNormal>& vDepthMeasurementCloud, 
        pcl::PointCloud<pcl::PointNormal>& vPointCloudBuffer,
        VolumeBase& oVolume);


protected:
    PointCloudOperation& m_oPcOperation;
    RosPublishManager& m_oRpManager;
};

#endif