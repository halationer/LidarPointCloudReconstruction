#include "MeshUpdater.h"
#include <pcl/io/ply_io.h>
#include <algorithm>
#include <numeric>

namespace Updater {

MeshUpdater MeshUpdater::instance;

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

    SectorMeshPtr pStartFrame = m_vFrameMeshes.GetStartFrame();
    if(pStartFrame != nullptr) {
        ROS_INFO_PURPLE("startFrame: sectors count %d", pStartFrame->size());
        for(TriangleMeshPtr mesh : *pStartFrame) {
            ROS_INFO_PURPLE("   startFrame: sector mesh size %d", mesh->mesh.size());
        }
    }        
    pcl::PointCloud<pcl::PointXYZI> vStartCloud;
    for(auto mesh : *pStartFrame) {
        vStartCloud += mesh->cloud;
    }


    for(auto && oSingleMesh : vSingleMeshList) {

        // triangle mesh init
        TriangleMeshPtr pTriangles(new TriangleMesh);
        pSectorMesh->push_back(pTriangles);
        auto& vTriangles = pTriangles->mesh;

        // Transfer cloud
        pcl::PointCloud<pcl::PointXYZ> oSingleCloud_;
        pcl::PointCloud<pcl::PointXYZI>& oSingleCloud = pTriangles->cloud;
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
        const Eigen::Vector3f& vCenter = oSingleCloud.back().getVector3fMap();
        for(auto polygon : oSingleMesh.polygons) {
            const Eigen::Vector3f& a = oSingleCloud[polygon.vertices[0]].getVector3fMap();
            const Eigen::Vector3f& b = oSingleCloud[polygon.vertices[1]].getVector3fMap();
            const Eigen::Vector3f& c = oSingleCloud[polygon.vertices[2]].getVector3fMap();

            Triangle oTriangle(a, b, c);
            oTriangle.CalculateYaw(vCenter);
            vTriangles.push_back(oTriangle);

            Eigen::Vector3f center = (a + b + c) / 3.0f;
            pcl::PointNormal oPoint;
            oPoint.getVector3fMap() = center;
            oPoint.getNormalVector3fMap() = oTriangle.n();
            oMeshCenters.push_back(oPoint);
        }
        std::vector<int> vTriangleMinYawIndex, vTriangleMaxYawIndex;
        vTriangles.reserve(oSingleMesh.polygons.size());
        vTriangleMinYawIndex.resize(oSingleMesh.polygons.size());
        vTriangleMaxYawIndex.resize(oSingleMesh.polygons.size());
        iota(vTriangleMinYawIndex.begin(), vTriangleMinYawIndex.end(), 0);
        iota(vTriangleMaxYawIndex.begin(), vTriangleMaxYawIndex.end(), 0);
        sort(vTriangleMinYawIndex.begin(), vTriangleMinYawIndex.end(), [&](int a, int b){
            return vTriangles[a].minYaw < vTriangles[b].minYaw;
        });
        sort(vTriangleMaxYawIndex.begin(), vTriangleMaxYawIndex.end(), [&](int a, int b){
            return vTriangles[a].maxYaw < vTriangles[b].maxYaw;
        });
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

        if(pStartFrame == nullptr) break;

        std::vector<int> vPredictIndex;
        vPredictIndex.reserve(vTriangles.size());
        pcl::PointCloud<pcl::PointXYZI> vCorners;
        // for(int x = oMinPos.x; x <= oMaxPos.x + 1; ++x) {
        // for(int y = oMinPos.y; y <= oMaxPos.y + 1; ++y) {
        // for(int z = oMinPos.z; z <= oMaxPos.z + 1; ++z) {
        for(auto& oPoint : vStartCloud) {

            if(oPoint.x < min_x || oPoint.x > max_x) continue;
            if(oPoint.y < min_y || oPoint.y > max_y) continue;
            if(oPoint.z < min_z || oPoint.z > max_z) continue;
            
            // pcl::PointXYZI oPoint;
            // oPoint.getVector3fMap() = Eigen::Vector3f(x, y, z);
            // oPoint.getVector3fMap() = oPoint.getVector3fMap().cwiseProduct(pHashBlockVolume->m_vBlockSize);
            // oPoint.intensity = 0;
            Eigen::Vector3f vCenterToCorner = oPoint.getVector3fMap() - vCenter;

            // search
            double yaw = Triangle::GetYaw(vCenterToCorner);
            int left = 0, right = vTriangleMinYawIndex.size();
            while(left < right) {
                int mid = (left + right) >> 1;
                if(vTriangles[vTriangleMinYawIndex[mid]].minYaw <= yaw) left = mid + 1;
                else right = mid;
            }
            for(int i = 0; i < left; ++i) vTriangles[vTriangleMinYawIndex[i]].searchSymbol = true;

            left = 0;
            right = vTriangleMaxYawIndex.size() - 1;
            while(left < right) {
                int mid = (left + right) >> 1;
                if(vTriangles[vTriangleMaxYawIndex[mid]].maxYaw < yaw) left = mid + 1;
                else right = mid;
            }
            vPredictIndex.clear();
            for(int i = left; i < vTriangleMaxYawIndex.size(); ++i)
                if(vTriangles[vTriangleMaxYawIndex[i]].searchSymbol)
                    vPredictIndex.push_back(vTriangleMaxYawIndex[i]);
            for(int i = 0; i < vTriangles.size(); ++i) vTriangles[i].searchSymbol = false;

            // 这个位置似乎能同时判断遮挡和穿透
            // for(auto&& oTriangle : vTriangles) { 
            for(auto&& oTriangleIndex : vPredictIndex) {
                Triangle& oTriangle = vTriangles[oTriangleIndex];
                float fIntersectDepth = -(vCenter.dot(oTriangle.n()) + oTriangle.D()) / vCenterToCorner.dot(oTriangle.n());
                if(fIntersectDepth < 1.05f) continue;
                Eigen::Vector3f oIntersectPoint = vCenter + fIntersectDepth * vCenterToCorner;
                if(oTriangle.ab.cross(oIntersectPoint - oTriangle.a).dot(oTriangle.n()) < 0) continue;
                if(oTriangle.bc.cross(oIntersectPoint - oTriangle.b).dot(oTriangle.n()) < 0) continue;
                if(oTriangle.ca.cross(oIntersectPoint - oTriangle.c).dot(oTriangle.n()) < 0) continue;
                oPoint.intensity = 1;
                break;
            }

            // HashPos oPos(x, y, z);
            // vCorners.push_back(oPoint);
            // if(vHashCorner.count(oPos))
            //     vHashCorner[oPos].intensity = std::max(vHashCorner[oPos].intensity, oPoint.intensity);
            // else vHashCorner[oPos] = oPoint;
        }
        // }}}
        m_oFuseTimer.DebugTime("judge corners");
    }

    pcl::PointCloud<pcl::PointXYZI> vAllCorners;
    std::vector<float> associate;
    for(auto&& oPoint : vStartCloud) {
    // for(auto && [_,oPoint] : vHashCorner) {
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

}