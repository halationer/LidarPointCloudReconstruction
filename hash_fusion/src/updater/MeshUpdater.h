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

private:
    class Triangle {

        public:
            Eigen::Vector3f a, b, c;
            Eigen::Vector3f ab, bc, ca;
            pcl::PointXYZI formula;

            pcl::Vector3fMap n() { return formula.getVector3fMap(); }
            float& D() {return formula.intensity; }

            Triangle(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c):a(a),b(b),c(c) {
                ab = b - a;
                bc = c - b;
                ca = a - c;
                n() = ab.cross(bc).normalized();
                D() = n().dot(-a);
            }
    };

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
    // use ghpr-space to get in-or-out info
    void MeshFusionV1(
        const pcl::PointNormal& oLidarPos, 
        pcl::PolygonMesh& oSingleMesh,
        VolumeBase& oVolume,
        bool bKeepVoxel);
    
    void MeshFusion(
        const pcl::PointNormal& oLidarPos, 
        std::vector<pcl::PolygonMesh>& vSingleMeshList,
        VolumeBase& oVolume,
        bool bKeepVoxel);

    void GhprConvertCloud(
        const pcl::PointXYZI & oCenter, 
        const pcl::PointCloud<pcl::PointXYZI> & vCloud,
        pcl::PointCloud<pcl::PointXYZI> & vResCloud);

private:

protected:
    RosPublishManager& m_oRpManager;
    std::unordered_set<HashPos, HashFunc> m_vPosRecord;

    TimeDebugger m_oFuseTimer;
    
};

#endif