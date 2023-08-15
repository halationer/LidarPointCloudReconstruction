#ifndef RAY_UPDATER
#define RAY_UPDATER

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>
#include <unordered_set>

#include "volume/VolumeBase.h"
#include "volume/HashBlock.h"
#include "tools/PointCloudOperation.h"
#include "tools/RosPublishManager.h"
#include "tools/HashPos.h"
#include "tools/DebugManager.h"

class RayCaster{

private:

    Eigen::Vector3f m_vStartPoint;
    Eigen::Vector3f m_vEndPoint;
    Eigen::Vector3f m_vBlockSize;
    static Eigen::Vector3f m_vLastStartPoint;
    static Eigen::Vector3f m_vLastEndPoint;
    static Eigen::Vector3f m_vLastBlockSize;

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

    typedef std::unordered_set<HashPos, HashFunc> HashPosSet;
    static void MakeRayDebugMarker(const HashPosSet& vBlockList, visualization_msgs::MarkerArray& oOutputVolume, int iIdOffset = 0); 
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
        const pcl::PointNormal& oLidarPos, 
        pcl::PointCloud<pcl::PointNormal>& vDepthMeasurementCloud,
        VolumeBase& oVolume,
        bool bKeepVoxel);

private:
    void BlockFusion(
        const pcl::PointNormal& oLidarPos, 
        pcl::PointCloud<pcl::PointNormal>& vLocalCloud,
	    HashBlock* pHashBlockVolume,
        Block& oBlock);

    void GetDepthAndIndexMap(pcl::PointCloud<pcl::PointNormal>& vLocalCloud);

protected:
    PointCloudOperation& m_oPcOperation;
    RosPublishManager& m_oRpManager;
    std::vector<std::vector<double>> m_vDepthImage;
    std::vector<std::vector<size_t>> m_vDepthIndex;
    std::unordered_set<HashPos, HashFunc> m_vPosRecord;

    TimeDebugger m_oFuseTimer;
};

#endif