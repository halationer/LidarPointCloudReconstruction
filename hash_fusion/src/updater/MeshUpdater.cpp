#include "MeshUpdater.h"
#include <pcl/io/ply_io.h>
#include <algorithm>
#include <numeric>
#include <mutex>

namespace Updater {

MeshUpdater MeshUpdater::instance;

pcl::PointCloud<pcl::PointXYZI> GetSectorMeshCloud(SectorMeshPtr pMesh) {
    
    pcl::PointCloud<pcl::PointXYZI> vCloud;
    if(pMesh != nullptr) {
        int iMeshSize = 0;
        for(TriangleMeshPtr mesh : *pMesh) {
            iMeshSize += mesh->mesh.size();
        }
        for(auto mesh : *pMesh) {
            vCloud += mesh->cloud;
        }
        ROS_INFO_PURPLE("get cloud: sectors count %d, point %d, mesh %d", pMesh->size(), vCloud.size(), iMeshSize);
    }
    return vCloud;
}

TriangleMeshPtr ConvertToTriangleMesh(pcl::PolygonMesh& oSingleMesh) {

    TriangleMeshPtr pTriangles(new TriangleMesh);
    pcl::MsgFieldMap field_map;
    pcl::createMapping<pcl::PointXYZ>(oSingleMesh.cloud.fields, field_map);
    pcl::fromPCLPointCloud2(oSingleMesh.cloud, pTriangles->cloud, field_map);

    pTriangles->mesh.reserve(oSingleMesh.polygons.size());
    const Eigen::Vector3f& vCenter = pTriangles->cloud.back().getVector3fMap();
    for(auto& polygon : oSingleMesh.polygons) {
        const Eigen::Vector3f& a = pTriangles->cloud[polygon.vertices[0]].getVector3fMap();
        const Eigen::Vector3f& b = pTriangles->cloud[polygon.vertices[1]].getVector3fMap();
        const Eigen::Vector3f& c = pTriangles->cloud[polygon.vertices[2]].getVector3fMap();
        Triangle oTriangle(a, b, c);
        oTriangle.CalculateYaw(vCenter);
        pTriangles->mesh.push_back(oTriangle);
    }

    return pTriangles;
}

void TriangleMesh::GetAABB() {

    Eigen::MatrixXf vSingleCloud = this->cloud.getMatrixXfMap();
    float max_x = vSingleCloud.row(0).maxCoeff();
    float min_x = vSingleCloud.row(0).minCoeff();
    float max_y = vSingleCloud.row(1).maxCoeff();
    float min_y = vSingleCloud.row(1).minCoeff();
    float max_z = vSingleCloud.row(2).maxCoeff();
    float min_z = vSingleCloud.row(2).minCoeff();
    this->min = pcl::PointXYZ(min_x, min_y, min_z);
    this->max = pcl::PointXYZ(max_x, max_y, max_z);
    // ROS_INFO_PURPLE("cloud matrix col-row: %d, %d", vSingleCloud.cols(), vSingleCloud.rows());
    // ROS_INFO_PURPLE("AABB: (%f, %f, %f) - (%f, %f, %f)", min_x, min_y, min_z, max_x, max_y, max_z);
}

void TriangleMesh::GetYawSortedIndex() {

    vTriangleMinYawIndex.resize(mesh.size());
    vTriangleMaxYawIndex.resize(mesh.size());
    iota(vTriangleMinYawIndex.begin(), vTriangleMinYawIndex.end(), 0);
    iota(vTriangleMaxYawIndex.begin(), vTriangleMaxYawIndex.end(), 0);
    sort(vTriangleMinYawIndex.begin(), vTriangleMinYawIndex.end(), [&](int a, int b){
        return mesh[a].minYaw < mesh[b].minYaw;
    });
    sort(vTriangleMaxYawIndex.begin(), vTriangleMaxYawIndex.end(), [&](int a, int b){
        return mesh[a].maxYaw < mesh[b].maxYaw;
    });
}

std::mutex mPointMutex;

void FindInnerOuter(
    pcl::PointCloud<pcl::PointXYZI>& vStartCloud,
    TriangleMeshPtr pTriangles) {
    
    TriangleMesh& oTriangles = *pTriangles;

    if(oTriangles.cloud.size() == 0) return;
    const Eigen::Vector3f& vCenter = oTriangles.cloud.back().getVector3fMap();

    std::vector<int> vPredictIndex;
    vPredictIndex.reserve(oTriangles.mesh.size());
    std::vector<Triangle>& vTriangles = oTriangles.mesh;
    
    for(auto& oPoint : vStartCloud) {

        if(oPoint.x < oTriangles.min.x || oPoint.x > oTriangles.max.x) continue;
        if(oPoint.y < oTriangles.min.y || oPoint.y > oTriangles.max.y) continue;
        if(oPoint.z < oTriangles.min.z || oPoint.z > oTriangles.max.z) continue;
        Eigen::Vector3f vCenterToPoint = oPoint.getVector3fMap() - vCenter;

        // search
        std::vector<char> searchSymbol(vTriangles.size(), false);
        double yaw = GetYaw(vCenterToPoint);
        int left = 0, right = oTriangles.vTriangleMinYawIndex.size();
        while(left < right) {
            int mid = (left + right) >> 1;
            if(vTriangles[oTriangles.vTriangleMinYawIndex[mid]].minYaw <= yaw) left = mid + 1;
            else right = mid;
        }
        for(int i = 0; i < left; ++i) searchSymbol[oTriangles.vTriangleMinYawIndex[i]] = true;

        left = 0;
        right = oTriangles.vTriangleMaxYawIndex.size() - 1;
        while(left < right) {
            int mid = (left + right) >> 1;
            if(vTriangles[oTriangles.vTriangleMaxYawIndex[mid]].maxYaw < yaw) left = mid + 1;
            else right = mid;
        }
        vPredictIndex.clear();
        for(int i = left; i < oTriangles.vTriangleMaxYawIndex.size(); ++i)
            if(searchSymbol[oTriangles.vTriangleMaxYawIndex[i]])
                vPredictIndex.push_back(oTriangles.vTriangleMaxYawIndex[i]);

        //这个位置似乎能同时判断遮挡和穿透
        // O -- 2 -- 1.05 -- 1 -- 0.98 -- 0.9 ---
        constexpr float factors[] = {1.05f, 0.98f, 0.9f};
        constexpr float dfactors[] = {0.5, -0.5f};
        // constexpr float factors[] = {1.03f, 0.98f, 0.95f};
        //为了消除地面的误判断，或许应该采用点到平面的真实距离，而不是射线的深度差
        // for(auto&& oTriangle : vTriangles) { 
        for(auto&& oTriangleIndex : vPredictIndex) {
            Triangle& oTriangle = vTriangles[oTriangleIndex];
            float fIntersectDepth = -(vCenter.dot(oTriangle.n()) + oTriangle.D()) / vCenterToPoint.dot(oTriangle.n());
            if(fIntersectDepth < factors[2]) continue;

            Eigen::Vector3f oIntersectPoint = vCenter + fIntersectDepth * vCenterToPoint;
            if(oTriangle.ab.cross(oIntersectPoint - oTriangle.a).dot(oTriangle.n()) < 0) continue;
            if(oTriangle.bc.cross(oIntersectPoint - oTriangle.b).dot(oTriangle.n()) < 0) continue;
            if(oTriangle.ca.cross(oIntersectPoint - oTriangle.c).dot(oTriangle.n()) < 0) continue;

            float fDistance = - oPoint.getVector3fMap().dot(oTriangle.n()) - oTriangle.D();

            // fIntersectDepth > 1 - Point在Mesh之内，即现在的Mesh穿透了之前的Point
            // fIntersectDepth < 1 - Point在Mesh之外，即现在的Mesh遮挡了之前的Point
            std::unique_lock<std::mutex> lock(mPointMutex);
            int& intensity = reinterpret_cast<int&>(oPoint.intensity);
            // if(fIntersectDepth > factors[0]) intensity = (intensity << 2) | 3;     // conflict
            // else if(fIntersectDepth > factors[1]) intensity = (intensity << 2) | 1; // support
            // else intensity = (intensity << 2) | 2; // occlude
            if(fDistance > dfactors[0]) intensity = (intensity << 2) | 3;     // conflict
            else if(fDistance > dfactors[1]) intensity = (intensity << 2) | 1; // support
            else intensity = (intensity << 2) | 2; // occlude
            break;
        }
    }
}

void MeshUpdater::MeshFusion(
    const pcl::PointNormal& oLidarPos,
    std::vector<pcl::PolygonMesh>& vSingleMeshList,
    VolumeBase& oVolume,
    bool bKeepVoxel)
{
    m_oFuseTimer.NewLine();

    HashBlock* pHashBlockVolume = dynamic_cast<HashBlock*>(&oVolume);
    if(pHashBlockVolume == nullptr) {
        ROS_ERROR("[updater/MeshUpdater] The volume type is not HashBlock!");
        return;
    }

    std::unordered_map<HashPos, pcl::PointXYZI, HashFunc> vHashCorner;
    pcl::PointCloud<pcl::PointNormal> oMeshCenters;
    RosPublishManager::HashPosSet vUpdatedPos;
    SectorMeshPtr pSectorMesh(new SectorMesh);
    m_vFrameMeshes.FrameEnqueue(pSectorMesh);

    SectorMeshPtr pStartMesh = m_vFrameMeshes.GetStartFrame();
    // pcl::PointCloud<pcl::PointXYZI> vStartCloud = GetSectorMeshCloud(pStartMesh);

	std::vector<Tools::TaskPtr> tasks;

    for(auto && oSingleMesh : vSingleMeshList) {

        // triangle mesh init
        TriangleMeshPtr pTriangles = ConvertToTriangleMesh(oSingleMesh);
        pTriangles->GetAABB();
        pTriangles->GetYawSortedIndex();
        pSectorMesh->push_back(pTriangles);
        m_oFuseTimer.DebugTime("convert_mesh");

        // jump if not enough frames
        if(pStartMesh == nullptr) continue;

        for(auto& pStartSector : *pStartMesh) {
            
            // 调试心得 - 局部变量不要引用，在出作用域时会出现未定义行为 - pTriangles
            tasks.emplace_back(m_oThreadPool.AddTask([&,pTriangles](){

                // find points is inner or outof the mesh
                FindInnerOuter(pStartSector->cloud, pTriangles);
            }));
        }

    }

    if(pStartMesh != nullptr) {

        for(TriangleMeshPtr& pTriangles : *pStartMesh) {

            for(auto& pCurrentSector : *pSectorMesh) {
                
                tasks.emplace_back(m_oThreadPool.AddTask([&,pTriangles](){

                    // find points is inner or outof the mesh
                    FindInnerOuter(pCurrentSector->cloud, pTriangles);
                }));
            }

        }
    }
    
    // wait for task ending
	for(auto& task : tasks) {
		task->Join();
	}
    m_oFuseTimer.DebugTime("judge points");


    m_oRpManager.PublishBlockSet(vUpdatedPos, pHashBlockVolume->m_vBlockSize, "/mesh_block_debug");
    m_oRpManager.PublishNormalPoints(oMeshCenters, "/mesh_normal_debug", [pSectorMesh](pcl::PointCloud<pcl::PointNormal>& oMeshCenters) {
        for(auto& sector : *pSectorMesh) {
            for(auto& triangle : sector->mesh) {
                Eigen::Vector3f center = (triangle.a + triangle.b + triangle.c) / 3.0f;
                pcl::PointNormal oPoint;
                oPoint.getVector3fMap() = center;
                oPoint.getNormalVector3fMap() = triangle.n();
                oMeshCenters.push_back(oPoint);
            }
        }
        ROS_INFO_PURPLE("Full Mesh Size: %d", oMeshCenters.size());
    });

    pcl::PointCloud<pcl::PointXYZI> vAllCorners;
    std::vector<float> associate;
    for(auto& pStartSector : *pStartMesh) {
        for(auto&& oPoint : pStartSector->cloud) {
            vAllCorners.push_back(oPoint);
            const int& intensity = reinterpret_cast<int&>(oPoint.intensity);
            associate.push_back(intensity == 11 || intensity == 14 ? 0.4 : -1);
            // associate.push_back(intensity == 15 ? 1 : -1);
            if(intensity >= 0 && intensity < 16)
                m_vDebugOutClouds[intensity].push_back(oPoint);
        }
    }

    // 把所有情况列出，添加到不同的点云，然后分别保存

    m_oRpManager.PublishPointCloud(vAllCorners, associate, "/corner_is_free_debug");

    m_oFuseTimer.CoutCurrentLine();
}

void MeshUpdater::MeshFusionV2(
    const pcl::PointNormal& oLidarPos,
    std::vector<pcl::PolygonMesh>& vSingleMeshList,
    VolumeBase& oVolume,
    bool bKeepVoxel)
{
    m_oFuseTimer.NewLine();

    HashBlock* pHashBlockVolume = dynamic_cast<HashBlock*>(&oVolume);
    if(pHashBlockVolume == nullptr) {
        ROS_ERROR("[updater/MeshUpdater] The volume type is not HashBlock!");
        return;
    }

    std::unordered_map<HashPos, pcl::PointXYZI, HashFunc> vHashCorner;
    pcl::PointCloud<pcl::PointNormal> oMeshCenters;
    RosPublishManager::HashPosSet vUpdatedPos;

    for(auto && oSingleMesh : vSingleMeshList) {

        // Transfer cloud
        pcl::PointCloud<pcl::PointXYZ> oSingleCloud_;
        pcl::PointCloud<pcl::PointXYZI> oSingleCloud;
        pcl::fromPCLPointCloud2(oSingleMesh.cloud, oSingleCloud_);
        pcl::copyPointCloud(oSingleCloud_, oSingleCloud);
        Eigen::MatrixXf vSingleCloud = oSingleCloud.getMatrixXfMap();
        ROS_INFO_PURPLE("cloud matrix col-row: %d, %d", vSingleCloud.cols(), vSingleCloud.rows());

        // pcl::toPCLPointCloud2(oSingleCloud, oSingleMesh.cloud);
        // pcl::io::savePLYFileBinary("/home/yudong/test.ply", oSingleMesh);

        // Get AABB box
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
        // m_oRpManager.PublishPointCloud(oSingleCloud, std::vector<float>(oSingleCloud.size(), 0), "/mesh_block_points");
        m_oFuseTimer.DebugTime("get_blocks");

        // Get triangles
        std::vector<Triangle> oTriangles;
        for(auto polygon : oSingleMesh.polygons) {
            const Eigen::Vector3f& a = oSingleCloud[polygon.vertices[0]].getVector3fMap();
            const Eigen::Vector3f& b = oSingleCloud[polygon.vertices[1]].getVector3fMap();
            const Eigen::Vector3f& c = oSingleCloud[polygon.vertices[2]].getVector3fMap();

            Triangle oTriangle(a, b, c);
            oTriangles.push_back(oTriangle);
            Eigen::Vector3f center = (a + b + c) / 3.0f;
            pcl::PointNormal oPoint;
            oPoint.getVector3fMap() = center;
            oPoint.getNormalVector3fMap() = oTriangle.n();
            oMeshCenters.push_back(oPoint);
        }
        m_oFuseTimer.DebugTime("get_triangles");

        // expand
        constexpr int expand = 0;
        oMinPos.x -= expand;
        oMinPos.y -= expand;
        oMinPos.z -= expand;
        oMaxPos.x += expand;
        oMaxPos.y += expand;
        oMaxPos.z += expand;
        // std::unordered_map<HashPos, bool, HashFunc> vBlocksFree;


        // 射线与面片相交
        // (o + td)n+D = 0
        // t = -(on+D)/dn
        // p = o+td
        // （abxap, bcxbp, caxcp）dot n >= (0, 0, 0)

        pcl::PointCloud<pcl::PointXYZI> vCorners;
        const Eigen::Vector3f& vCenter = oSingleCloud.back().getVector3fMap();
        for(int x = oMinPos.x; x <= oMaxPos.x + 1; ++x) {
        for(int y = oMinPos.y; y <= oMaxPos.y + 1; ++y) {
        for(int z = oMinPos.z; z <= oMaxPos.z + 1; ++z) {

            HashPos oPos(x, y, z);
            pcl::PointXYZI oPoint;
            oPoint.getVector3fMap() = Eigen::Vector3f(x, y, z);
            oPoint.getVector3fMap() = oPoint.getVector3fMap().cwiseProduct(pHashBlockVolume->m_vBlockSize);
            oPoint.intensity = 0;

            for(auto&& oTriangle : oTriangles) {
                Eigen::Vector3f vCenterToCorner = oPoint.getVector3fMap() - vCenter;
                float fIntersectDepth = -(vCenter.dot(oTriangle.n()) + oTriangle.D()) / vCenterToCorner.dot(oTriangle.n());
                if(fIntersectDepth < 1.0f) continue;
                Eigen::Vector3f oIntersectPoint = vCenter + fIntersectDepth * vCenterToCorner;
                if(oTriangle.ab.cross(oIntersectPoint - oTriangle.a).dot(oTriangle.n()) < 0) continue;
                if(oTriangle.bc.cross(oIntersectPoint - oTriangle.b).dot(oTriangle.n()) < 0) continue;
                if(oTriangle.ca.cross(oIntersectPoint - oTriangle.c).dot(oTriangle.n()) < 0) continue;
                oPoint.intensity = 1;
                break;
            }

            vCorners.push_back(oPoint);
            if(vHashCorner.count(oPos))
                vHashCorner[oPos].intensity = std::max(vHashCorner[oPos].intensity, oPoint.intensity);
            else vHashCorner[oPos] = oPoint;

            // if(x > oMinPos.x && y > oMinPos.y && z > oMinPos.z) {
            //     HashPos pos(x - 1, y - 1, z - 1);
            //     vBlocksFree[pos] = 
            // }
        }}}
        m_oFuseTimer.DebugTime("judge corners");
    }

    pcl::PointCloud<pcl::PointXYZI> vAllCorners;
    std::vector<float> associate;
    for(auto && [_,oPoint] : vHashCorner) {
        // if(oPoint.intensity) {
            vAllCorners.push_back(oPoint);
            associate.push_back(oPoint.intensity * 0.4f);
        // }
    }
    m_oRpManager.PublishPointCloud(vAllCorners, associate, "/corner_is_free_debug");
    m_oRpManager.PublishNormalPoints(oMeshCenters, "/mesh_normal_debug");
    m_oRpManager.PublishBlockSet(vUpdatedPos, pHashBlockVolume->m_vBlockSize, "/mesh_block_debug");

    m_oFuseTimer.CoutCurrentLine();
}

void MeshUpdater::MeshFusionV1(
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
    pcl::PointCloud<pcl::PointXYZI> oSingleCloud_, oSingleCloud;
    pcl::fromPCLPointCloud2(oSingleMesh.cloud, oSingleCloud_);
    Eigen::MatrixXf vSingleCloud = oSingleCloud_.getMatrixXfMap();
    GhprConvertCloud(oSingleCloud_.back(), oSingleCloud_, oSingleCloud);
    ROS_INFO_PURPLE("cloud matrix col-row: %d, %d", vSingleCloud.cols(), vSingleCloud.rows());

    // pcl::toPCLPointCloud2(oSingleCloud, oSingleMesh.cloud);
    // pcl::io::savePLYFileBinary("/home/yudong/test.ply", oSingleMesh);

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
    m_oRpManager.PublishPointCloud(oSingleCloud_, std::vector<float>(oSingleCloud.size(), 0), "/mesh_block_points");
    m_oFuseTimer.DebugTime("get_blocks");

    // Get triangles
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

        pcl::PointXYZI oTriangle;
        oTriangle.getVector3fMap() = n;
        oTriangle.intensity = n.dot(-a);
        oTriangles.push_back(oTriangle);
    }
    m_oRpManager.PublishNormalPoints(oMeshCenters, "/mesh_normal_debug");
    m_oFuseTimer.DebugTime("get_triangles");

    // Block in the mesh
    constexpr int expand = 8;
    oMinPos.x -= expand;
    oMinPos.y -= expand;
    oMinPos.z -= expand;
    oMaxPos.x += expand;
    oMaxPos.y += expand;
    oMaxPos.z += expand;
    // std::unordered_map<HashPos, bool, HashFunc> vBlocksFree;
    pcl::PointCloud<pcl::PointXYZI> vCorners, vConvertCorners;
    for(int x = oMinPos.x; x <= oMaxPos.x + 1; ++x) {
        for(int y = oMinPos.y; y <= oMaxPos.y + 1; ++y) {
            for(int z = oMinPos.z; z <= oMaxPos.z + 1; ++z) {

                pcl::PointXYZI oPoint;
                oPoint.getVector3fMap() = Eigen::Vector3f(x, y, z);
                oPoint.getVector3fMap() = oPoint.getVector3fMap().cwiseProduct(pHashBlockVolume->m_vBlockSize);
                oPoint.intensity = 0;
                vCorners.push_back(oPoint);
            }
        }
    }

    GhprConvertCloud(oSingleCloud_.back(), vCorners, vConvertCorners);
    // pcl::io::savePLYFileBinary("/home/yudong/testcorner.ply", vConvertCorners);

    std::vector<float> associate;
    for(auto oPoint : vConvertCorners) {
        
        for(auto&& oTriangle : oTriangles) {
            if(oPoint.getVector4fMap().dot(oTriangle.getVector4fMap()) > 0){
                // oPoint.intensity += 1.0f / oTriangles.size();
                oPoint.intensity = 1;
                break;
            }
        }
        associate.push_back(oPoint.intensity * 0.4f);

        // if(x > oMinPos.x && y > oMinPos.y && z > oMinPos.z) {
        //     HashPos pos(x - 1, y - 1, z - 1);
        //     vBlocksFree[pos] = 
        // }
    }
    m_oRpManager.PublishPointCloud(vCorners, associate, "/corner_is_free_debug");
    m_oFuseTimer.DebugTime("judge corners");

    m_oFuseTimer.CoutCurrentLine();
}

void MeshUpdater::GhprConvertCloud(const pcl::PointXYZI & oCenter, const pcl::PointCloud<pcl::PointXYZI> & vCloud, pcl::PointCloud<pcl::PointXYZI> & vResCloud){

	vResCloud.clear();

	//the modulus of each point
	std::vector<float> vNormEachPoint;

	//get local coordinates based on the viewpoint 
	for (int i = 0; i != vCloud.size(); ++i){

		// std::cout << "point_index: " << i << ";\tpoint_intensity: " << pCloud->points[i].intensity << std::endl;

		pcl::PointXYZI oOnePoint;
        oOnePoint.getVector3fMap() = vCloud[i].getVector3fMap() - oCenter.getVector3fMap();
        oOnePoint.intensity = oOnePoint.getVector3fMap().norm();
		vResCloud.push_back(oOnePoint);
	}

	//get the radius of local coordiante system
    float param = 3.6;
	float fRadius = pow(10.0, param) * vResCloud.getMatrixXfMap().row(3).maxCoeff();

	//
	for (size_t i = 0; i != vResCloud.size(); i++){
		float numerator = 2 * (fRadius - vResCloud[i].intensity);
        vResCloud[i].getVector3fMap() += numerator * vResCloud[i].getVector3fMap() / vResCloud[i].intensity;
        // vResCloud[i].getVector3fMap() += oCenter.getVector3fMap();
        vResCloud[i].intensity = vCloud[i].intensity;
	}
}

void MeshUpdater::SaveDebugOutClouds(){
    for(int i = 0; i < m_vDebugOutClouds.size(); ++i) {
        pcl::io::savePLYFileBinary("/home/yudong/Desktop/newtest/"+std::to_string(i)+".ply", m_vDebugOutClouds[i]);
    }
}

}