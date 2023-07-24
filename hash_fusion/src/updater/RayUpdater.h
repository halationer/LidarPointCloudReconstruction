#ifndef RAY_UPDATER
#define RAY_UPDATER

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>
#include <vector>

#include "volume/VolumeBase.h"
#include "tools/PointCloudOperation.h"
#include "tools/RosPublishManager.h"
#include "tools/HashPos.h"

class RayCaster{

private:
    Eigen::Vector3f m_vStartPoint;
    Eigen::Vector3f m_vEndPoint;
    Eigen::Vector3f m_vBlockSize;

    // Caster params
    // 当前行进到的体素位置
    Eigen::Vector3i m_vCurrentPos;
    // 当前到最近的角点的向量记录
    Eigen::Vector3f m_vCurrentToNearstCorner;
    // 每步走的距离记录（三维不同的距离）
    Eigen::Vector3f m_vStepDistance;
    Eigen::Vector3i m_vStepDirection;
    // 当前处于第几步
    int m_iCurrentStep;
    // 共几步
    int m_iStepNum;

public:
    RayCaster(const Eigen::Vector3f& vStartPoint, const Eigen::Vector3f& vEndPoint, const Eigen::Vector3f& vBlockSize);
    bool GetNextHashPos(HashPos& oBlockPos);
    void InitCasterParams();
    void MakeRayDebugMarker(const std::vector<HashPos>& vBlockList, visualization_msgs::MarkerArray& oOutputVolume, int iOffset = 0); 
};

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