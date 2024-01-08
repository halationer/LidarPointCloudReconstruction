#ifndef MESH_UPDATER
#define MESH_UPDATER

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <Eigen/Core>
#include <unordered_set>
#include <list>
#include <cmath>

#include "volume/VolumeBase.h"
#include "volume/HashBlock.h"
#include "volume/DistanceIoVolume.h"

#include "tools/PointCloudOperation.h"
#include "tools/RosPublishManager.h"
#include "tools/HashPos.h"
#include "tools/DebugManager.h"
#include "tools/ThreadPool.h"

namespace Updater{

inline double GetYaw(const Eigen::Vector3f& vDirection) {
    return std::atan2((double)vDirection.y(), (double)vDirection.x());
}

class Triangle {

public:
    Eigen::Vector3f a, b, c;
    Eigen::Vector3f ab, bc, ca;
    pcl::PointXYZI formula;
    double minYaw, maxYaw;
    float confidence = 1.0f;

    pcl::Vector3fMap n() { return formula.getVector3fMap(); }
    float& D() {return formula.intensity; }

    Triangle(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c):a(a),b(b),c(c) {
        ab = b - a;
        bc = c - b;
        ca = a - c;
        n() = ab.cross(bc).normalized();
        D() = n().dot(-a);
    }

    void CalculateYaw(const Eigen::Vector3f& vCenter){
        double yawA = GetYaw(a - vCenter);
        double yawB = GetYaw(b - vCenter);
        double yawC = GetYaw(c - vCenter);
        if(yawA > yawB) std::swap(yawA, yawB);
        if(yawA > yawC) std::swap(yawA, yawC);
        if(yawC < yawB) std::swap(yawB, yawC);
        minYaw = yawA;
        maxYaw = yawC;
    }
};

class TriangleMesh {

public:
    typedef std::shared_ptr<TriangleMesh> Ptr;

    std::vector<Triangle> mesh;
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::PointXYZ min, max;
    std::vector<int> vTriangleMinYawIndex, vTriangleMaxYawIndex;

    void GetAABB();
    void GetYawSortedIndex();
    void FindInnerOuter(pcl::PointCloud<pcl::PointXYZI>& vQueryCloud);
    void FindInnerOuterSimple(pcl::PointCloud<pcl::PointXYZI>& vQueryCloud);
    void FindInnerOuterSimple(pcl::DistanceIoVoxel& oQueryPoint);
    void FindInnerOuterSimple(pcl::PointCloud<pcl::DistanceIoVoxel>& oQueryCloud);
    void SetTriangleConfidence(const std::vector<float>& vConfidenceList);

    static Ptr GetFromPolygonMesh(pcl::PolygonMesh& oMesh);

private:
    void RayUpdateVoxel(const Eigen::Vector3f& vRayStart, pcl::DistanceIoVoxel& oQueryPoint);

private:
    static std::mutex mSetPointIntensity;
};

typedef std::vector<TriangleMesh::Ptr> SectorMesh;
typedef std::shared_ptr<SectorMesh> SectorMeshPtr;

class SectorMeshFrames {
    
private:
    int m_iFrameStart = -1, m_iFrameEnd = 0;
    std::vector<SectorMeshPtr> m_vFrames;
    int m_iWindowSize;

public:
    SectorMeshFrames(int size):m_vFrames(size),m_iWindowSize(size){}

public:
    void FrameEnqueue(SectorMeshPtr mesh) {
        m_iFrameEnd %= m_iWindowSize;
        if(m_iFrameEnd == m_iFrameStart)
            m_iFrameStart = (m_iFrameStart + 1) % m_iWindowSize;
        m_iFrameStart = std::max(m_iFrameStart, 0);
        m_vFrames[m_iFrameEnd++] = mesh;
    }
    SectorMeshPtr GetCurrentFrame() {
        return m_vFrames[(m_iFrameEnd - 1 + m_iWindowSize) % m_iWindowSize];
    }
    SectorMeshPtr GetStartFrame() {
        if(m_iFrameStart < 0) return nullptr;
        return m_vFrames[m_iFrameStart];
    }
    
    static pcl::PointCloud<pcl::PointXYZI> GetSectorMeshCloud(SectorMeshPtr pMesh);
};

// main class
class MeshUpdater {

private:
    Tools::ThreadPool m_oThreadPool;
    TimeDebugger m_oFuseTimer;
    RosPublishManager& m_oRpManager;
    SectorMeshFrames m_vFrameMeshes;
    std::vector<pcl::PointCloud<pcl::PointXYZI>> m_vDebugOutClouds;

public:
    static constexpr int m_iMaxFrameWindow = 5;

//singleton
private:
    static MeshUpdater instance;
    MeshUpdater():
        m_oRpManager(RosPublishManager::GetInstance()),
        m_vFrameMeshes(m_iMaxFrameWindow),
        m_oThreadPool(4),
        m_vDebugOutClouds(16, pcl::PointCloud<pcl::PointXYZI>()) {}
    MeshUpdater(MeshUpdater&)=delete;

public:
    static MeshUpdater& GetInstance() {
        return instance;
    }
    // ~MeshUpdater(){ SaveDebugOutClouds(); }

private:
    void SaveDebugOutClouds();
    void UpdateVolume(
        const SectorMesh& oSectorMesh, 
        DistanceIoVolume* const pDistanceIoVolume, 
        std::vector<pcl::PointCloud<pcl::DistanceIoVoxel>::Ptr>& vCorners,
        size_t iLevel);
    void UpdateVisibleVolume(
        const SectorMesh& oSectorMesh, 
        DistanceIoVolume* const pDistanceIoVolume,
        std::vector<pcl::PointCloud<pcl::DistanceIoVoxel>::Ptr>& vCorners,
        size_t iLevel = 0);

    void MakeDynamicObject(const DistanceIoVolume* pDistanceIoVolume, const pcl::PointCloud<pcl::PointXYZI>& vDynamicPoints);

public:
    // use ghpr-space to get in-or-out info
    void MeshFusionV1(
        const pcl::PointNormal& oLidarPos, 
        pcl::PolygonMesh& oSingleMesh,
        VolumeBase& oVolume,
        bool bKeepVoxel);
    
    // use ray-mesh to update corner occupancy
    void MeshFusionV2(
        const pcl::PointNormal& oLidarPos, 
        std::vector<pcl::PolygonMesh>& vSingleMeshList,
        VolumeBase& oVolume,
        bool bKeepVoxel);

    void MeshFusionV3(
        const pcl::PointNormal& oLidarPos, 
        std::vector<pcl::PolygonMesh>& vSingleMeshList,
        VolumeBase& oVolume,
        bool bKeepVoxel);

    void MeshFusionV4(
        const pcl::PointNormal& oLidarPos, 
        std::vector<pcl::PolygonMesh>& vSingleMeshList,
        VolumeBase& oVolume,
        bool bKeepVoxel);

    void MeshFusion(
        const pcl::PointNormal& oLidarPos, 
        std::vector<pcl::PolygonMesh>& vSingleMeshList,
        std::vector<std::vector<float>>& vMeshConfidence,
        VolumeBase& oVolume,
        bool bKeepVoxel);

    void GhprConvertCloud(
        const pcl::PointXYZI & oCenter, 
        const pcl::PointCloud<pcl::PointXYZI> & vCloud,
        pcl::PointCloud<pcl::PointXYZI> & vResCloud);
};

}

#endif