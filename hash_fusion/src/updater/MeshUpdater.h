#ifndef MESH_UPDATER
#define MESH_UPDATER

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <Eigen/Core>
#include <unordered_set>

#include "volume/VolumeBase.h"
#include "volume/HashBlock.h"
#include "tools/PointCloudOperation.h"
#include "tools/RosPublishManager.h"
#include "tools/HashPos.h"
#include "tools/DebugManager.h"

class MeshUpdater {

//singleton
private:
    static MeshUpdater instance;
    MeshUpdater():
        m_oRpManager(RosPublishManager::GetInstance()) {}
    MeshUpdater(MeshUpdater&)=delete;
public:
    static MeshUpdater& GetInstance() {
        return instance;
    }


public:
    void MeshFusion(
        const pcl::PointNormal& oLidarPos, 
        pcl::PolygonMesh& oSingleMesh,
        VolumeBase& oVolume,
        bool bKeepVoxel);

private:

protected:
    RosPublishManager& m_oRpManager;
    std::unordered_set<HashPos, HashFunc> m_vPosRecord;

    TimeDebugger m_oFuseTimer;
};

#endif