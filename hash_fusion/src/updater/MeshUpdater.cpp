#include "MeshUpdater.h"

MeshUpdater MeshUpdater::instance;

void MeshUpdater::MeshFusion(
    const pcl::PointNormal& oLidarPos, 
    pcl::PolygonMesh& oSingleMesh,
    VolumeBase& oVolume,
    bool bKeepVoxel)
{
    m_oFuseTimer.NewLine();

    HashBlock* pHashBlockVolume = dynamic_cast<HashBlock*>(&oVolume);
    if(pHashBlockVolume == nullptr) {
        ROS_ERROR("[updater/MeshUpdater] The volume type is not HashBlock!");
        return;
    }

    // Transfer cloud - 0.2ms
    pcl::PointCloud<pcl::PointXYZ> oSingleCloud;
    pcl::fromPCLPointCloud2(oSingleMesh.cloud, oSingleCloud);
    Eigen::MatrixXf vSingleCloud = oSingleCloud.getMatrixXfMap();
    ROS_INFO_PURPLE("cloud matrix col-row: %d, %d", vSingleCloud.cols(), vSingleCloud.rows());

    // Get AABB box - 0.2ms
    float max_x = vSingleCloud.row(0).maxCoeff();
    float min_x = vSingleCloud.row(0).minCoeff();
    float max_y = vSingleCloud.row(1).maxCoeff();
    float min_y = vSingleCloud.row(1).minCoeff();
    float max_z = vSingleCloud.row(2).maxCoeff();
    float min_z = vSingleCloud.row(2).minCoeff();
    pcl::PointXYZ vMin(min_x, min_y, min_z);
    pcl::PointXYZ vMax(max_x, max_y, max_z);
    ROS_INFO_PURPLE("AABB: (%f, %f, %f) - (%f, %f, %f)", min_x, min_y, min_z, max_x, max_y, max_z);

    // Get Updated Blocks
    RosPublishManager::HashPosSet vUpdatedPos;
    HashPos oMinPos, oMaxPos;
    pHashBlockVolume->PointBelongBlockPos(vMin, oMinPos);
    pHashBlockVolume->PointBelongBlockPos(vMax, oMaxPos);
    for(int i = oMinPos.x; i <= oMaxPos.x; ++i) {
        for(int j = oMinPos.y; j <= oMaxPos.y; ++j) {
            for(int k = oMinPos.z; k <= oMaxPos.z; ++k) {
                vUpdatedPos.emplace(i, j, k);
            }
        }
    }
    m_oRpManager.PublishBlockSet(vUpdatedPos, pHashBlockVolume->m_vBlockSize, "/mesh_block_debug");
    m_oRpManager.PublishPointCloud(oSingleCloud, std::vector<float>(oSingleCloud.size(), 0), "/mesh_block_points");
    m_oFuseTimer.DebugTime("get_blocks");

    // Find outter blocks
    pcl::PointCloud<pcl::PointXYZI> oTriangles;
    pcl::PointCloud<pcl::PointNormal> oMeshCenters;
    for(auto polygon : oSingleMesh.polygons) {
        const Eigen::Vector3f& a = oSingleCloud[polygon.vertices[0]].getVector3fMap();
        const Eigen::Vector3f& b = oSingleCloud[polygon.vertices[1]].getVector3fMap();
        const Eigen::Vector3f& c = oSingleCloud[polygon.vertices[2]].getVector3fMap();

        Eigen::Vector3f ab = b - a;
        Eigen::Vector3f bc = c - b;
        Eigen::Vector3f n = ab.cross(bc).normalized();
        Eigen::Vector3f center = (a + b + c) / 3.0f;
        pcl::PointNormal oPoint;
        oPoint.getVector3fMap() = center;
        oPoint.getNormalVector3fMap() = n;
        oMeshCenters.push_back(oPoint);
    }
    m_oRpManager.PublishNormalPoints(oMeshCenters, "/mesh_normal_debug");
    m_oFuseTimer.DebugTime("get_triangles");

    m_oFuseTimer.CoutCurrentLine();
}