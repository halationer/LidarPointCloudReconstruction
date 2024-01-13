#include "MeshUpdater.h"
#include <pcl/io/ply_io.h>
#include <algorithm>
#include <numeric>
#include <mutex>

#include "volume/SimpleVolume.h"
#include "volume/DistanceIoVolume.h"
#include "tools/UnionSet.h"
#include "tools/BoundingBox.h"

namespace Updater {

MeshUpdater MeshUpdater::instance;
std::mutex TriangleMesh::mSetPointIntensity;

/////////////////////////////////////////////////////////
/////////////////// Object Manager //////////////////////
/////////////////////////////////////////////////////////
void ObjectManager::ReceiveClustersAndUpdate(const std::vector<tools::BoundingBox>& vClusterList, const int& iFrameStamp) {

    for(auto&& oCluster : vClusterList) {

        Eigen::Vector3f vCenter = (oCluster.GetMaxBound() + oCluster.GetMinBound()) * 0.5f;
        float fRadius = (oCluster.GetMaxBound() - oCluster.GetMinBound()).sum() / 3.0f;
        SequenceIter pSeq = FindCorrespondingSequence(vCenter, fRadius, iFrameStamp);
        UpdateObjectSequence(pSeq, vCenter, fRadius, iFrameStamp);
    }
}

ObjectManager::SequenceIter ObjectManager::FindCorrespondingSequence(const Eigen::Vector3f& vCenter, const float& fRadius, const int& iFrameStamp) {

    float fMinDistance = INFINITY;
    SequenceIter pCorrespondSeq = m_vObjectSequenceList.end();

    for(SequenceIter pSeq = m_vObjectSequenceList.begin(); pSeq != m_vObjectSequenceList.end(); pSeq++) {
        
        // if not element in seq, the object is deleted
        if(pSeq->empty()) continue;

        // calculate corespondence
        const Eigen::Vector3f& vLastPosition = pSeq->back().GetPosition();
        float distance = (vLastPosition - vCenter).norm();
        int delta_frame = iFrameStamp - (int)pSeq->back().framestamp;
        float velocity = distance / delta_frame;

        // judge if the object belong the seq
        constexpr float velocity_threshold = 0.5f;
        if(velocity <= velocity_threshold && distance < fMinDistance) {
            pCorrespondSeq = pSeq;
            fMinDistance = distance;
        }
    }

    return pCorrespondSeq;
}

void ObjectManager::UpdateObjectSequence(SequenceIter pSequence, const Eigen::Vector3f& vCenter, const float& fRadius, const int& iFrameStamp) {

    // if no correspondence
    if(pSequence == m_vObjectSequenceList.end()) {
        CreateObjectSequence(vCenter, fRadius, iFrameStamp);
        return;
    }

    // make object
    tools::Object oObject;
    oObject.GetPosition() = vCenter;
    oObject.GetVelocity() = (vCenter - pSequence->back().GetPosition()).normalized();
    oObject.inited = true;
    oObject.radius = fRadius;
    oObject.framestamp = iFrameStamp;
    oObject.timestamp = ros::Time::now().toSec();

    // push object into sequence
    pSequence->push_back(oObject);
}

void ObjectManager::CreateObjectSequence(const Eigen::Vector3f& vCenter, const float& fRadius, const int& iFrameStamp) {
    
    // make object
    tools::Object oObject;
    oObject.GetPosition() = vCenter;
    oObject.GetVelocity() = Eigen::Vector3f::UnitY();
    oObject.inited = true;
    oObject.radius = fRadius;
    oObject.framestamp = iFrameStamp;
    oObject.timestamp = ros::Time::now().toSec();

    // find a sequence
    SequenceIter pSequenceContianer = m_vObjectSequenceList.end();
    for(SequenceIter pSeq = m_vObjectSequenceList.begin(); pSeq != m_vObjectSequenceList.end(); pSeq++) {
        if(pSeq->size() == 0) {
            pSequenceContianer = pSeq;
            break;
        }
    }

    // if find failed, create a new sequence
    if(pSequenceContianer == m_vObjectSequenceList.end()) {
        m_vObjectSequenceList.emplace_back();
        pSequenceContianer = m_vObjectSequenceList.end()-1;
    }

    // put the object into sequence
    pSequenceContianer->push_back(oObject);
}

std::vector<tools::Object> ObjectManager::GetActivedObjects(const int& iFrameStamp) {

    std::vector<tools::Object> vObjectList;
    for(auto&& oObject : m_vObjectSequenceList) {
        if( oObject.size() > 3 && 
            oObject.back().framestamp == iFrameStamp && 
            (oObject.end()-2)->framestamp == iFrameStamp-1) {
            vObjectList.push_back(oObject.back());
        }
    }
    return vObjectList;
}

/////////////////////////////////////////////////////////
/////////////////// Sector Mesh Cloud ///////////////////
/////////////////////////////////////////////////////////
pcl::PointCloud<pcl::PointXYZI> SectorMeshFrames::GetSectorMeshCloud(SectorMeshPtr pMesh) {
    
    pcl::PointCloud<pcl::PointXYZI> vCloud;
    if(pMesh != nullptr) {
        int iMeshSize = 0;
        for(TriangleMesh::Ptr mesh : *pMesh) {
            iMeshSize += mesh->mesh.size();
        }
        for(auto mesh : *pMesh) {
            vCloud += mesh->cloud;
        }
        ROS_INFO_PURPLE("get cloud: sectors count %d, point %d, mesh %d", pMesh->size(), vCloud.size(), iMeshSize);
    }
    return vCloud;
}

/////////////////////////////////////////////////////////
/////////////////// Triangle Mesh ///////////////////////
/////////////////////////////////////////////////////////
TriangleMesh::Ptr TriangleMesh::GetFromPolygonMesh(pcl::PolygonMesh& oMesh) {

    TriangleMesh::Ptr pTriangles(new TriangleMesh);
    pcl::MsgFieldMap field_map;
    pcl::createMapping<pcl::PointXYZ>(oMesh.cloud.fields, field_map);
    pcl::fromPCLPointCloud2(oMesh.cloud, pTriangles->cloud, field_map);

    pTriangles->mesh.reserve(oMesh.polygons.size());
    const Eigen::Vector3f& vCenter = pTriangles->cloud.back().getVector3fMap();
    for(auto& polygon : oMesh.polygons) {
        const Eigen::Vector3f& a = pTriangles->cloud[polygon.vertices[0]].getVector3fMap();
        const Eigen::Vector3f& b = pTriangles->cloud[polygon.vertices[1]].getVector3fMap();
        const Eigen::Vector3f& c = pTriangles->cloud[polygon.vertices[2]].getVector3fMap();
        Triangle oTriangle(a, b, c);
        oTriangle.CalculateYaw(vCenter);
        pTriangles->mesh.push_back(oTriangle);
    }

    return pTriangles;
}

void TriangleMesh::SetTriangleConfidence(const std::vector<float>& vConfidenceList) {

    for(int i = 0; i < mesh.size(); ++i) {
        mesh[i].confidence = vConfidenceList[i];
    }
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

void TriangleMesh::FindInnerOuter(pcl::PointCloud<pcl::PointXYZI>& vQueryCloud) {
    
    TriangleMesh& oTriangles = *this;

    if(oTriangles.cloud.size() == 0) return;
    const Eigen::Vector3f& vCenter = oTriangles.cloud.back().getVector3fMap();

    std::vector<int> vPredictIndex;
    vPredictIndex.reserve(oTriangles.mesh.size());
    std::vector<Triangle>& vTriangles = oTriangles.mesh;
    
    for(auto& oPoint : vQueryCloud) {

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
        constexpr float dfactors[] = {0.2, -0.2f};
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
            std::unique_lock<std::mutex> lock(TriangleMesh::mSetPointIntensity);
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

void TriangleMesh::FindInnerOuterSimple(pcl::PointCloud<pcl::PointXYZI>& vQueryCloud) {
    
    TriangleMesh& oTriangles = *this;

    if(oTriangles.cloud.size() == 0) return;
    const Eigen::Vector3f& vCenter = oTriangles.cloud.back().getVector3fMap();

    std::vector<int> vPredictIndex;
    vPredictIndex.reserve(oTriangles.mesh.size());
    std::vector<Triangle>& vTriangles = oTriangles.mesh;
    
    for(auto& oPoint : vQueryCloud) {

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
        constexpr float dfactors[] = {0.2, -0.2f};
        // constexpr float factors[] = {1.03f, 0.98f, 0.95f};
        //为了消除地面的误判断，或许应该采用点到平面的真实距离，而不是射线的深度差
        // for(auto&& oTriangle : vTriangles) { 
        for(auto&& oTriangleIndex : vPredictIndex) {
            Triangle& oTriangle = vTriangles[oTriangleIndex];
            float fIntersectDepth = -(vCenter.dot(oTriangle.n()) + oTriangle.D()) / vCenterToPoint.dot(oTriangle.n());
            if(fIntersectDepth < 1) continue;

            Eigen::Vector3f oIntersectPoint = vCenter + fIntersectDepth * vCenterToPoint;
            if(oTriangle.ab.cross(oIntersectPoint - oTriangle.a).dot(oTriangle.n()) < 0) continue;
            if(oTriangle.bc.cross(oIntersectPoint - oTriangle.b).dot(oTriangle.n()) < 0) continue;
            if(oTriangle.ca.cross(oIntersectPoint - oTriangle.c).dot(oTriangle.n()) < 0) continue;

            float fDistance = - oPoint.getVector3fMap().dot(oTriangle.n()) - oTriangle.D();
            std::unique_lock<std::mutex> lock(TriangleMesh::mSetPointIntensity);
            oPoint.intensity = 1.0;
            break;
        }
    }
}

void TriangleMesh::FindInnerOuterSimple(pcl::DistanceIoVoxel& oQueryPoint) {
    
    TriangleMesh& oTriangles = *this;

    if(oTriangles.cloud.size() == 0) return;
    const Eigen::Vector3f& vCenter = oTriangles.cloud.back().getVector3fMap();

    std::vector<int> vPredictIndex;
    vPredictIndex.reserve(oTriangles.mesh.size());
    std::vector<Triangle>& vTriangles = oTriangles.mesh;
    
    if(oQueryPoint.x < oTriangles.min.x || oQueryPoint.x > oTriangles.max.x) return;
    if(oQueryPoint.y < oTriangles.min.y || oQueryPoint.y > oTriangles.max.y) return;
    if(oQueryPoint.z < oTriangles.min.z || oQueryPoint.z > oTriangles.max.z) return;
    Eigen::Vector3f vCenterToQueryPoint = oQueryPoint.getVector3fMap() - vCenter;

    // search
    std::vector<char> searchSymbol(vTriangles.size(), false);
    double yaw = GetYaw(vCenterToQueryPoint);
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
    constexpr float dfactors[] = {0.2, -0.2f};
    // constexpr float factors[] = {1.03f, 0.98f, 0.95f};
    //为了消除地面的误判断，或许应该采用点到平面的真实距离，而不是射线的深度差
    // for(auto&& oTriangle : vTriangles) { 
    for(auto&& oTriangleIndex : vPredictIndex) {
        Triangle& oTriangle = vTriangles[oTriangleIndex];
        float fIntersectDepth = -(vCenter.dot(oTriangle.n()) + oTriangle.D()) / vCenterToQueryPoint.dot(oTriangle.n());
        if(fIntersectDepth < 1) continue;

        Eigen::Vector3f oIntersectPoint = vCenter + fIntersectDepth * vCenterToQueryPoint;
        if(oTriangle.ab.cross(oIntersectPoint - oTriangle.a).dot(oTriangle.n()) < 0) continue;
        if(oTriangle.bc.cross(oIntersectPoint - oTriangle.b).dot(oTriangle.n()) < 0) continue;
        if(oTriangle.ca.cross(oIntersectPoint - oTriangle.c).dot(oTriangle.n()) < 0) continue;

        float fDistance = - oQueryPoint.getVector3fMap().dot(oTriangle.n()) - oTriangle.D();
        std::unique_lock<std::mutex> lock(TriangleMesh::mSetPointIntensity);
        oQueryPoint.io = 1.0;
        break;
    }
}

void TriangleMesh::FindInnerOuterSimple(pcl::PointCloud<pcl::DistanceIoVoxel>& vQueryCloud) {
    
    if(this->cloud.size() == 0) return;

    const Eigen::Vector3f& vCenter = this->cloud.back().getVector3fMap();

    for(pcl::DistanceIoVoxel& oPoint : vQueryCloud) {
        
        // update point
        RayUpdateVoxel(vCenter, oPoint);

        // update point
        RayUpdateVoxelOnlyDistance(vCenter+Eigen::Vector3f(0, 0, 5.0f), oPoint);
    }
}

void TriangleMesh::RayUpdateVoxel(const Eigen::Vector3f& vRayStart, pcl::DistanceIoVoxel& oQueryPoint) {

    std::vector<Triangle>& vTriangles = this->mesh;
    std::vector<int> vPredictIndex;
    vPredictIndex.reserve(vTriangles.size());
    Eigen::Vector3f vRay = oQueryPoint.getVector3fMap() - vRayStart;

    // search
    std::vector<char> searchSymbol(vTriangles.size(), false);
    double yaw = GetYaw(vRay);
    int left = 0, right = this->vTriangleMinYawIndex.size();
    while(left < right) {
        int mid = (left + right) >> 1;
        if(vTriangles[this->vTriangleMinYawIndex[mid]].minYaw <= yaw) left = mid + 1;
        else right = mid;
    }
    for(int i = 0; i < left; ++i) searchSymbol[this->vTriangleMinYawIndex[i]] = true;

    left = 0;
    right = this->vTriangleMaxYawIndex.size() - 1;
    while(left < right) {
        int mid = (left + right) >> 1;
        if(vTriangles[this->vTriangleMaxYawIndex[mid]].maxYaw < yaw) left = mid + 1;
        else right = mid;
    }
    for(int i = left; i < this->vTriangleMaxYawIndex.size(); ++i)
        if(searchSymbol[this->vTriangleMaxYawIndex[i]])
            vPredictIndex.push_back(this->vTriangleMaxYawIndex[i]);

    //遍历三角形并判断内外
    for(auto&& oTriangleIndex : vPredictIndex) {

        Triangle& oTriangle = vTriangles[oTriangleIndex];
        float fIntersectDepth = -(vRayStart.dot(oTriangle.n()) + oTriangle.D()) / vRay.dot(oTriangle.n());

        if(fIntersectDepth <= 0) continue;

        Eigen::Vector3f oIntersectPoint = vRayStart + fIntersectDepth * vRay;
        if(oTriangle.ab.cross(oIntersectPoint - oTriangle.a).dot(oTriangle.n()) < 0) continue;
        if(oTriangle.bc.cross(oIntersectPoint - oTriangle.b).dot(oTriangle.n()) < 0) continue;
        if(oTriangle.ca.cross(oIntersectPoint - oTriangle.c).dot(oTriangle.n()) < 0) continue;

        float fDistance = - oQueryPoint.getVector3fMap().dot(oTriangle.n()) - oTriangle.D();
        fDistance = fabs(fDistance);

        std::unique_lock<std::mutex> lock(TriangleMesh::mSetPointIntensity);
        if(oQueryPoint.io == 0.0f) oQueryPoint.distance = fDistance;
        else oQueryPoint.distance = std::min(oQueryPoint.distance, fDistance);
        if(fIntersectDepth >= 1) oQueryPoint.io = 1.0;
        else if(oTriangle.confidence > 0.02f && fDistance < 1.0f) oQueryPoint.io = -1.0f;

        break;
    }
}

void TriangleMesh::RayUpdateVoxelOnlyDistance(const Eigen::Vector3f& vRayStart, pcl::DistanceIoVoxel& oQueryPoint) {

    std::vector<Triangle>& vTriangles = this->mesh;
    std::vector<int> vPredictIndex;
    vPredictIndex.reserve(vTriangles.size());
    Eigen::Vector3f vRay = oQueryPoint.getVector3fMap() - vRayStart;

    // search
    std::vector<char> searchSymbol(vTriangles.size(), false);
    double yaw = GetYaw(vRay);
    int left = 0, right = this->vTriangleMinYawIndex.size();
    while(left < right) {
        int mid = (left + right) >> 1;
        if(vTriangles[this->vTriangleMinYawIndex[mid]].minYaw <= yaw) left = mid + 1;
        else right = mid;
    }
    for(int i = 0; i < left; ++i) searchSymbol[this->vTriangleMinYawIndex[i]] = true;

    left = 0;
    right = this->vTriangleMaxYawIndex.size() - 1;
    while(left < right) {
        int mid = (left + right) >> 1;
        if(vTriangles[this->vTriangleMaxYawIndex[mid]].maxYaw < yaw) left = mid + 1;
        else right = mid;
    }
    for(int i = left; i < this->vTriangleMaxYawIndex.size(); ++i)
        if(searchSymbol[this->vTriangleMaxYawIndex[i]])
            vPredictIndex.push_back(this->vTriangleMaxYawIndex[i]);

    //遍历三角形并判断内外
    for(auto&& oTriangleIndex : vPredictIndex) {

        Triangle& oTriangle = vTriangles[oTriangleIndex];
        float fIntersectDepth = -(vRayStart.dot(oTriangle.n()) + oTriangle.D()) / vRay.dot(oTriangle.n());

        if(fIntersectDepth <= 0) continue;

        Eigen::Vector3f oIntersectPoint = vRayStart + fIntersectDepth * vRay;
        if(oTriangle.ab.cross(oIntersectPoint - oTriangle.a).dot(oTriangle.n()) < 0) continue;
        if(oTriangle.bc.cross(oIntersectPoint - oTriangle.b).dot(oTriangle.n()) < 0) continue;
        if(oTriangle.ca.cross(oIntersectPoint - oTriangle.c).dot(oTriangle.n()) < 0) continue;

        float fDistance = - oQueryPoint.getVector3fMap().dot(oTriangle.n()) - oTriangle.D();
        fDistance = fabs(fDistance);

        std::unique_lock<std::mutex> lock(TriangleMesh::mSetPointIntensity);
        if(oQueryPoint.io == 0.0f) oQueryPoint.distance = fDistance;
        else oQueryPoint.distance = std::min(oQueryPoint.distance, fDistance);

        break;
    }
}

/////////////////////////////////////////////////////////
///////////////////// Mesh Updater //////////////////////
/////////////////////////////////////////////////////////
void MeshUpdater::UpdateVolume(
    const SectorMesh& oSectorMesh, 
    DistanceIoVolume* const pDistanceIoVolume, 
    std::vector<pcl::PointCloud<pcl::DistanceIoVoxel>::Ptr>& vCorners,
    size_t iLevel) {

	std::vector<Tools::TaskPtr> tasks;
    std::vector<pcl::PointCloud<pcl::DistanceIoVoxel>::Ptr> vNewCorners;
    
    // first level
    if(vCorners.size() == 0) {
        for(auto& pTriangles : oSectorMesh) {
            vCorners.emplace_back(new pcl::PointCloud<pcl::DistanceIoVoxel>(std::move( 
                pDistanceIoVolume->CreateAndGetCornersAABB(pTriangles->min.getVector3fMap(), pTriangles->max.getVector3fMap(), iLevel))));
            auto pCorners = vCorners.back();
            tasks.emplace_back(m_oThreadPool.AddTask([&,pTriangles,pCorners](){
                pTriangles->FindInnerOuterSimple(*pCorners);
                pDistanceIoVolume->Update(*pCorners);
            }));
        }
    }
    // other level
    else {
        std::mutex mCornerCombine;
        // assert vCorners.size() == oSectorMesh.size()
        for(int i = 0; i < vCorners.size(); ++i) {
            auto sub_corners = pDistanceIoVolume->CreateAndGetSubdivideCorners(*vCorners[i], iLevel);
            vNewCorners.emplace_back(new pcl::PointCloud<pcl::DistanceIoVoxel>(*vCorners[i]));
            auto pNewCorner = vNewCorners.back();
            for(auto sub_corner : sub_corners) {
                tasks.emplace_back(m_oThreadPool.AddTask([&,i,sub_corner,pNewCorner](){
                    oSectorMesh.at(i)->FindInnerOuterSimple(*sub_corner);
                    pDistanceIoVolume->Update(*sub_corner);
                    std::unique_lock<std::mutex> lock(mCornerCombine);
                    *pNewCorner += *sub_corner;
                }));
            }
        }
    }

    // wait tasks finish
    for(auto& task : tasks) task->Join();

    // recursive
    if(vNewCorners.size()) vCorners.assign(vNewCorners.begin(), vNewCorners.end());
    if(iLevel > 0) UpdateVolume(oSectorMesh, pDistanceIoVolume, vCorners, iLevel-1);
}

void MeshUpdater::UpdateVisibleVolume(
    const SectorMesh& oSectorMesh, 
    DistanceIoVolume* const pDistanceIoVolume,
    std::vector<pcl::PointCloud<pcl::DistanceIoVoxel>::Ptr>& vCorners,
    size_t iLevel)
{
    const size_t step = 1 << iLevel;

	std::vector<Tools::TaskPtr> tasks;

    HashPos oPos;
    for(const TriangleMesh::Ptr& pTriangles : oSectorMesh) {

        // 1. 提取mesh中的点云, 对于每个点，找到对应level的体素
        HashPosSet vVoxelPoses;
        for(const pcl::PointXYZI& oPoints : pTriangles->cloud) {
            if(oPoints.intensity > pDistanceIoVolume->GetStaticExpandDistance()) continue;
            pDistanceIoVolume->PointBelongVoxelPos(oPoints, oPos);
            AlignToStepFloor(oPos, step);
            vVoxelPoses.emplace(oPos);
        }

        // 2. 根据mesh更新所有对应体素的角点
        auto pCornerPoints = pDistanceIoVolume->CreateAndGetCornersByPos(vVoxelPoses, iLevel);
        vCorners.emplace_back(pCornerPoints);
        tasks.emplace_back(m_oThreadPool.AddTask([&, pTriangles, pCornerPoints](){
            pTriangles->FindInnerOuterSimple(*pCornerPoints);
            pDistanceIoVolume->UpdateLimitDistance(*pCornerPoints, iLevel);
        }));
    }

    // wait for task finish
    for(auto& task : tasks) task->Join();
}


const static HashPos vNeighborDirs[] = {
    {1, 0, 0}, {0, 1, 0}, {0, 0, 1},
    {-1, 0, 0}, {0, -1, 0}, {0, 0, -1}, // 6-neighbor
    {1, 1, 0}, {1, 0, 1}, {0, 1, 1},
    {-1, -1, 0}, {-1, 0, -1}, {0, -1, -1},
    {1, -1, 0}, {1, 0, -1}, {0, 1, -1},
    {-1, 1, 0}, {-1, 0, 1}, {0, -1, 1}, // 18-neighbor
    {1, 1, 1}, {-1, 1, 1}, {1, -1, 1}, {1, 1, -1},
    {-1, -1, -1}, {1, -1, -1}, {-1, 1, -1}, {-1, -1, 1}, // 26-neighbor
};

void MeshUpdater::MakeDynamicObject(DistanceIoVolume* pDistanceIoVolume, const SectorMeshPtr& pSectorMesh) {

    // Search For sdf of points and show if the point is judged to be dynamic
    pcl::PointCloud<pcl::PointXYZI> vSearchedPoints, vDynamicPoints;
    std::vector<float> vSearchedAssociation;
    for(auto && pTriangle : *pSectorMesh) {
        auto& vPoints = pTriangle->cloud;
        for(auto& oPoint : vPoints) {
            oPoint.intensity = pDistanceIoVolume->SearchSdf(oPoint.getVector3fMap());
            vSearchedPoints.push_back(oPoint);
            float color_value = 0.4f;
            if(oPoint.intensity > pDistanceIoVolume->GetStaticExpandDistance()) {
                color_value = 1.0f;
                vDynamicPoints.push_back(oPoint);
            }
            vSearchedAssociation.push_back(color_value);
        }
    }
    m_oRpManager.PublishPointCloud(vSearchedPoints, vSearchedAssociation, "/searched_point_cloud");

    // voxelize
    HashPosSet vDynamicVoxels;
    std::unordered_map<HashPos, std::vector<pcl::PointXYZI>, HashFunc> vPosPoints;
    for(auto&& oPoint : vDynamicPoints) {
        HashPos oPos;
        pDistanceIoVolume->PointBelongVoxelPos(oPoint, oPos);
        vDynamicVoxels.emplace(oPos);
        vPosPoints[oPos].push_back(oPoint);
    }
    m_oRpManager.PublishBlockSet(vDynamicVoxels, pDistanceIoVolume->GetVoxelLength(), "/dynamic_voxels");

    // cluster
    constexpr int iNeighborNum = 18;
    UnionSet oDynamicSetMaker;
    for(auto&& oPos : vDynamicVoxels) {
        oDynamicSetMaker.Find(oPos);
        for(int i = 0; i < iNeighborNum; ++i) {
            HashPos oNeighborPos = oPos + vNeighborDirs[i];
            if(vDynamicVoxels.count(oNeighborPos)) {
                oDynamicSetMaker.Union(oPos, oNeighborPos);
            }
        }
    }

    // make-object
    auto vDynamicSets = oDynamicSetMaker.GetSets();
    std::vector<tools::BoundingBox> vBoundingBoxList;
    for(auto&& [_,vDynamicSet] : vDynamicSets) {
        tools::BoundingBox oBoundingBox;
        for(auto&& oPos : vDynamicSet) {
            for(auto&& oPoint : vPosPoints[oPos]) {
                if(!oBoundingBox.inited) {
                    oBoundingBox.GetMinBound() = oPoint.getVector3fMap();
                    oBoundingBox.GetMaxBound() = oPoint.getVector3fMap();
                    oBoundingBox.inited = true;
                }
                else {
                    oBoundingBox.GetMinBound() = oBoundingBox.GetMinBound().cwiseMin(oPoint.getVector3fMap());
                    oBoundingBox.GetMaxBound() = oBoundingBox.GetMaxBound().cwiseMax(oPoint.getVector3fMap());
                }
                ++oBoundingBox.point_count;
            }
        }
        vBoundingBoxList.push_back(oBoundingBox);
    }

    // cluster filter
    constexpr float density_min_threshold = 1.0f;
    constexpr float density_max_threshold = 128.0f;
    std::cout << "filter begin" << std::endl;
    if(!vBoundingBoxList.empty()) {
        for(int i = 0; i < vBoundingBoxList.size();) {
            auto&& oBox = vBoundingBoxList[i];
            Eigen::Vector3f vBoxSize = oBox.GetMaxBound() - oBox.GetMinBound();
            float density = oBox.point_count / vBoxSize.prod() * vBoxSize.minCoeff();
            std::cout << oBox.point_count << " " << density << " " 
                << (density >= density_min_threshold && density <= density_max_threshold) << std::endl;
            if(density < density_min_threshold || density > density_max_threshold) {
                vBoundingBoxList.erase(vBoundingBoxList.begin()+i);
            }
            else ++i;
        }
    }
    std::cout << "filter end" << std::endl;
    m_oRpManager.PublishBoundingBoxList(vBoundingBoxList, "/dynamic_clusters", 0);

    m_oObjectManager.ReceiveClustersAndUpdate(vBoundingBoxList, m_fFrameCounter);
    m_oRpManager.PublishObjectList(m_oObjectManager.GetActivedObjects(m_fFrameCounter), "/dynamic_objects", 0);
}


void MeshUpdater::MeshFusion(
    const pcl::PointNormal& oLidarPos,
    std::vector<pcl::PolygonMesh>& vSingleMeshList,
    std::vector<std::vector<float>>& vMeshConfidence,
    VolumeBase& oVolume,
    bool bKeepVoxel)
{
    m_oFuseTimer.NewLine();

    DistanceIoVolume* pDistanceIoVolume = dynamic_cast<DistanceIoVolume*>(&oVolume);
    if(pDistanceIoVolume == nullptr) {
        ROS_ERROR("[updater/MeshUpdater] The volume type is not DistanceIoVolume!");
        return;
    }

    ++m_fFrameCounter;

    std::unordered_map<HashPos, pcl::PointXYZI, HashFunc> vHashCorner;
    SectorMeshPtr pSectorMesh(new SectorMesh);
    // m_vFrameMeshes.FrameEnqueue(pSectorMesh);

	// std::vector<Tools::TaskPtr> tasks;
    int iConfidenceIndex = 0;
    for(auto && oSingleMesh : vSingleMeshList) {
        // triangle mesh init
        TriangleMesh::Ptr pTriangles = TriangleMesh::GetFromPolygonMesh(oSingleMesh);
        pTriangles->GetAABB();
        pTriangles->GetYawSortedIndex();
        pTriangles->SetTriangleConfidence(vMeshConfidence.at(iConfidenceIndex++));
        pSectorMesh->push_back(pTriangles);
        m_oFuseTimer.DebugTime("convert_mesh");
    }

    // voxelize and cluster, out put the dynamic objects
    MakeDynamicObject(pDistanceIoVolume, pSectorMesh);
    m_oFuseTimer.DebugTime("dynamic_extract");

    // update local volume
    std::unique_ptr<DistanceIoVolume> pLocalVolume(new DistanceIoVolume);
    std::vector<pcl::PointCloud<pcl::DistanceIoVoxel>::Ptr> corners;
    pcl::PointXYZ oLength;
    oLength.getVector3fMap() = oVolume.GetVoxelLength();
    pLocalVolume->SetResolution(oLength);
    UpdateVolume(*pSectorMesh, pLocalVolume.get(), corners, 3);
    UpdateVisibleVolume(*pSectorMesh, pLocalVolume.get(), corners);
    m_oFuseTimer.DebugTime("single_io");

    // sector 分布
    // 2 | 1
    // 3 | 0
    // show local voxel results
    pcl::PointCloud<pcl::DistanceIoVoxel> vAllCorners;
    std::vector<float> associate, associate2;
    for(int i = 0; i < corners.size(); ++i) {
        for(auto point : *corners[i]) {
            if(point.io == 0.0) continue;
            HashPos oPos;
            pLocalVolume->PointBelongVoxelPos(point, oPos);
            const pcl::DistanceIoVoxel* pVoxel = pLocalVolume->GetVoxel(oPos);
            if(pVoxel == nullptr) continue;
            vAllCorners.push_back(*pVoxel);
            associate.push_back(std::min(pVoxel->distance * 0.5f + 0.4f, 1.0f));
        }
    }
    ROS_INFO_PURPLE("corner size: %d, volume point: %d,%d", vAllCorners.size(), 
        pLocalVolume->m_vVolumeData.size(), pLocalVolume->m_vVolumeData.back().size());
    m_oRpManager.PublishPointCloud(vAllCorners, associate, "/corner_is_free_debug");
    
    // local to global
    pDistanceIoVolume->Fuse(*pLocalVolume);
    m_oFuseTimer.DebugTime("voxelize");

    // show global voxel result
    vAllCorners.clear();
    associate.clear();
    for(auto&& vVoxelList : pDistanceIoVolume->m_vVolumeData) {
        for(auto&& oVoxel : vVoxelList) {
            // if(oVoxel.io == 0.0) continue;
            if(oVoxel.distance > 1.5f) continue;
            vAllCorners.push_back(oVoxel);
            associate.push_back(std::min(oVoxel.distance + 0.4f, 1.0f));
            associate2.push_back(oVoxel.io > 0.5f ? 0.8f : -1.0f);
        }
    }
    ROS_INFO_PURPLE("global size: %d, volume point: %d,%d", vAllCorners.size(), 
        pDistanceIoVolume->m_vVolumeData.size(), pDistanceIoVolume->m_vVolumeData.back().size());
    m_oRpManager.PublishPointCloud(vAllCorners, associate, "/global_distance_volume");
    m_oRpManager.PublishPointCloud(vAllCorners, associate2, "/global_io_volume");

    m_oFuseTimer.CoutCurrentLine();
}

#define HISTORY_VERSION_MESHUPDATER
#ifndef HISTORY_VERSION_MESHUPDATER

void MeshUpdater::MeshFusionV4(
    const pcl::PointNormal& oLidarPos,
    std::vector<pcl::PolygonMesh>& vSingleMeshList,
    VolumeBase& oVolume,
    bool bKeepVoxel)
{
    m_oFuseTimer.NewLine();

    DistanceIoVolume* pDistanceIoVolume = dynamic_cast<DistanceIoVolume*>(&oVolume);
    if(pDistanceIoVolume == nullptr) {
        ROS_ERROR("[updater/MeshUpdater] The volume type is not DistanceIoVolume!");
        return;
    }

    std::unordered_map<HashPos, pcl::PointXYZI, HashFunc> vHashCorner;
    SectorMeshPtr pSectorMesh(new SectorMesh);
    m_vFrameMeshes.FrameEnqueue(pSectorMesh);

	std::vector<Tools::TaskPtr> tasks;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> corners;
    
    for(auto && oSingleMesh : vSingleMeshList) {

        // triangle mesh init
        TriangleMesh::Ptr pTriangles = TriangleMesh::GetFromPolygonMesh(oSingleMesh);
        pTriangles->GetAABB();
        pTriangles->GetYawSortedIndex();
        pSectorMesh->push_back(pTriangles);
        m_oFuseTimer.DebugTime("convert_mesh");

        constexpr int expand = 1;
        pcl::PointCloud<pcl::PointXYZI>::Ptr pCorners(new pcl::PointCloud<pcl::PointXYZI>());
        int lenx = 0, leny = 0, lenz = 0;
        for(float x = pTriangles->min.x - expand; x <= pTriangles->max.x + 1 + expand; x += pDistanceIoVolume->GetVoxelLength().x()) {
            lenx++; leny = 0;
        for(float y = pTriangles->min.y - expand; y <= pTriangles->max.y + 1 + expand; y += pDistanceIoVolume->GetVoxelLength().y()) {
            leny++; lenz = 0;
        for(float z = pTriangles->min.z - expand; z <= pTriangles->max.z + 1 + expand; z += pDistanceIoVolume->GetVoxelLength().z()) {
            lenz++;
            pcl::PointXYZI oPoint;
            oPoint.getVector3fMap() = Eigen::Vector3f(x, y, z);
            oPoint.intensity = 0.0f;
            pCorners->push_back(oPoint);
        }}}
        corners.push_back(pCorners);
        // ROS_INFO_PURPLE("%d %d %d %d", lenx, leny, lenz, lenx*leny*lenz);

        // 由于计算下一层之前，必须先计算出上一层的内容，因此只能做到分层并行，这里暂时把并行去掉
        // tasks.emplace_back(m_oThreadPool.AddTask([&,pTriangles,pCorners](){

            // find points is inner or outof the mesh
            pTriangles->FindInnerOuterSimple(*pCorners);
        // }));

        Eigen::Vector3f sub_offset[7] = {
            {0.5, 0.5, 0.5}, // center
            {0  , 0.5, 0.5}, {0.5, 0  , 0.5}, {0.5, 0.5, 0  }, // face
            {0.5, 0  , 0  }, {0  , 0.5, 0  }, {0  , 0  , 0.5}, // edge
        };
        int index_offset[] = {
            0, 
            1, 
            lenz, 
            lenz * leny, 
            1 + lenz, 
            1 + lenz * leny, 
            lenz + lenz * leny, 
            1 + lenz + lenz * leny
        };
        for(int i = 0; i < 7; ++i) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr pSubCorners(new pcl::PointCloud<pcl::PointXYZI>(*pCorners));
            // 前三行代表x,y,z, 每一列是一个点，进行平移。
            pSubCorners->getMatrixXfMap().topRows(3).colwise() += pDistanceIoVolume->GetVoxelLength().cwiseProduct(sub_offset[i]);
            corners.push_back(pSubCorners);

            // 删除非细分的点
            int delete_count = 0;
            for(int k = 0; k < pSubCorners->size()-delete_count; ++k) {
                int mask = 0;
                for(int j = 0; j < 8; ++j) {
                    int query_index = k + index_offset[j];
                    mask = mask << 1;
                    if(query_index >= pCorners->size()) continue;
                    mask |= pCorners->at(query_index).intensity == 1.0f? 1 : 0;
                }
                bool subdivision = mask > 0 && mask < 0xff;
                if(!subdivision) {
                    ++delete_count;
                    std::swap(pSubCorners->at(k), pSubCorners->at(pSubCorners->size()-delete_count));
                }
            }
            pSubCorners->erase(pSubCorners->end()-delete_count, pSubCorners->end());
            ROS_INFO_PURPLE("delete_count: %d/%d->%d", delete_count, pCorners->size(), pSubCorners->size());

            tasks.emplace_back(m_oThreadPool.AddTask([&,pTriangles,pSubCorners](){

                // find points is inner or outof the mesh
                pTriangles->FindInnerOuterSimple(*pSubCorners);
            }));
        }
    }
    
    // wait for task ending
	for(auto& task : tasks) {
		task->Join();
	}
    m_oFuseTimer.DebugTime("judge points");

    pcl::PointCloud<pcl::PointXYZI> vAllCorners;
    std::vector<float> associate;
    for(int i = 0; i < corners.size(); ++i) {
        for(auto point : *corners[i]) {
            if(point.intensity == 1.0) {
                vAllCorners.push_back(point);
                associate.push_back((i % 8 ? 1 : 0) * 0.4);
            }
        }
        // vAllCorners += *corners[i];
        // associate.insert(associate.end(), corners[i]->size(), (i % 8 ? 1 : 0) * 0.4);
    }
    ROS_INFO_PURPLE("corner size: %d", vAllCorners.size());
    
    m_oFuseTimer.DebugTime("voxelize");
    m_oRpManager.PublishPointCloud(vAllCorners, associate, "/corner_is_free_debug");

    m_oFuseTimer.CoutCurrentLine();
}

/////////////////////////////////////////////////////////////////////
// 思路：使用前后帧对比的方法，将点云和网格记录到队列中，判断内外使用二分+射线方法
void MeshUpdater::MeshFusionV3(
    const pcl::PointNormal& oLidarPos,
    std::vector<pcl::PolygonMesh>& vSingleMeshList,
    VolumeBase& oVolume,
    bool bKeepVoxel)
{
    m_oFuseTimer.NewLine();

    SimpleVolume* pSimpleVolume = dynamic_cast<SimpleVolume*>(&oVolume);
    if(pSimpleVolume == nullptr) {
        ROS_ERROR("[updater/MeshUpdater] The volume type is not SimpleVolume!");
        return;
    }

    std::unordered_map<HashPos, pcl::PointXYZI, HashFunc> vHashCorner;
    pcl::PointCloud<pcl::PointNormal> oMeshCenters;
    SectorMeshPtr pSectorMesh(new SectorMesh);
    m_vFrameMeshes.FrameEnqueue(pSectorMesh);

    SectorMeshPtr pStartMesh = m_vFrameMeshes.GetStartFrame();

	std::vector<Tools::TaskPtr> tasks;

    for(auto && oSingleMesh : vSingleMeshList) {

        // triangle mesh init
        TriangleMesh::Ptr pTriangles = TriangleMesh::GetFromPolygonMesh(oSingleMesh);
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
                pTriangles->FindInnerOuter(pStartSector->cloud);
            }));
        }
    }

    if(pStartMesh != nullptr) {

        for(TriangleMesh::Ptr& pTriangles : *pStartMesh) {

            for(auto& pCurrentSector : *pSectorMesh) {
                
                tasks.emplace_back(m_oThreadPool.AddTask([&,pTriangles](){

                    // find points is inner or outof the mesh
                    pTriangles->FindInnerOuter(pCurrentSector->cloud);
                }));
            }

        }
    }
    
    // wait for task ending
	for(auto& task : tasks) {
		task->Join();
	}
    m_oFuseTimer.DebugTime("judge points");

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
            // associate.push_back(intensity == 11 || intensity == 14 ? 0.4 : -1);
            // associate.push_back(intensity == 15 ? 1 : -1);
            associate.push_back(intensity == 5 ? 1 : -1);
            if(intensity >= 0 && intensity < 16)
                m_vDebugOutClouds[intensity].push_back(oPoint);
        }
        pSimpleVolume->VoxelizePointsAndFusion(pStartSector->cloud);
    }
    m_oFuseTimer.DebugTime("voxelize");

    pSimpleVolume->PublishType();
    m_oRpManager.PublishPointCloud(vAllCorners, associate, "/corner_is_free_debug");

    m_oFuseTimer.CoutCurrentLine();
}

////////////////////////////////////////////////////////////////
// 思路：对于每个Sector，更新其内部空间的体素角点，判断内外使用遍历射线方法
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
    HashPosSet vUpdatedPos;

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

/////////////////////////////////////////////////////////////////////////
// 思路：对于每个Sector，更新其内部空间的体素角点，判断内外使用GHPR反投影，遍历内外法
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
    HashPosSet vUpdatedPos;
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

#endif
}