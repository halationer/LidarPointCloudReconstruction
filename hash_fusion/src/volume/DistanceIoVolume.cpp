#include <memory>
#include <shared_mutex>
#include <sstream>

#include "volume/DistanceIoVolume.h"
#include "tools/RosPublishManager.h"

/**
 * @brief set voxel resolution and related params
 * @param oLength - 3d size of a voxel
*/
void DistanceIoVolume::SetResolution(pcl::PointXYZ & oLength) { 
	m_vVoxelSize = oLength.getVector3fMap(); 
	m_vVoxelHalfSize = m_vVoxelSize * 0.5f;
	m_vVoxelSizeInverse = m_vVoxelSize.cwiseInverse();
}

/**
 * @brief get hash pos according to point position
 * @param oPoint point with position
 * @return oPos - output block pos
*/
template<class PointType>
void DistanceIoVolume::PointBelongVoxelPos(const PointType & oPoint, HashPos & oVoxelPos) const {
    
    oVoxelPos.x = floor(oPoint.x * m_vVoxelSizeInverse.x());
	oVoxelPos.y = floor(oPoint.y * m_vVoxelSizeInverse.y());
	oVoxelPos.z = floor(oPoint.z * m_vVoxelSizeInverse.z());
}
template void DistanceIoVolume::PointBelongVoxelPos(const pcl::PointXYZ & oPoint, HashPos & oVoxelPos) const;
template void DistanceIoVolume::PointBelongVoxelPos(const pcl::PointXYZI & oPoint, HashPos & oVoxelPos) const;
template void DistanceIoVolume::PointBelongVoxelPos(const pcl::PointNormal & oPoint, HashPos & oVoxelPos) const;

/**
 * @brief get hash pos according to point position
 * @param oPoint point with position
 * @return oPos - output block pos
*/
void DistanceIoVolume::PointBelongVoxelPos(const pcl::PointNormal & oPoint, HashPos & oVoxelPos) const {
    
    oVoxelPos.x = floor(oPoint.x * m_vVoxelSizeInverse.x());
	oVoxelPos.y = floor(oPoint.y * m_vVoxelSizeInverse.y());
	oVoxelPos.z = floor(oPoint.z * m_vVoxelSizeInverse.z());
}
void DistanceIoVolume::PointBelongVoxelPos(const Eigen::Vector3f & vPoint, HashPos & oVoxelPos) const {
    oVoxelPos.x = floor(vPoint.x() * m_vVoxelSizeInverse.x());
	oVoxelPos.y = floor(vPoint.y() * m_vVoxelSizeInverse.y());
	oVoxelPos.z = floor(vPoint.z() * m_vVoxelSizeInverse.z());
}

/**
 * @brief voxelize the points and fuse them
 * @param vCloud the input lidar cloud with normals
*/
void DistanceIoVolume::VoxelizePointsAndFusion(pcl::PointCloud<pcl::PointXYZI> & vCloud) {
        
    for(auto&& oPoint : vCloud) {
        const int& intensity = reinterpret_cast<int&>(oPoint.intensity);
        HashPos oPos;
        PointBelongVoxelPos(oPoint, oPos);
        if(!m_vVolume.count(oPos)) m_vVolume.emplace(oPos, new DistanceIoVoxel);
        m_vVolume[oPos]->point.getVector3fMap() = oPoint.getVector3fMap();
        // m_vVolume[oPos]->AddStatus(intensity);
    }
}

/**
 * @brief create voxel corner in range, publish the pointers to updater in order to update in/out
 * @param vMin bounding box min point
 * @param vMax bounding box max point
 * @param iLevel level of the octree, from bottom to top: 0 - k
*/
std::vector<DistanceIoVoxel::Ptr> DistanceIoVolume::CreateAndGetCorners(const Eigen::Vector3f& vMin, const Eigen::Vector3f& vMax, size_t iLevel){

    HashPos oMinPos, oMaxPos;
    PointBelongVoxelPos(vMin, oMinPos);
    PointBelongVoxelPos(vMax, oMaxPos);

    size_t step = 1 << iLevel;
    AlignToStepFloor(oMinPos, step);
    AlignToStepCeil(oMaxPos, step);

    int pointNum = (oMaxPos.x - oMinPos.x + 1) * 
                (oMaxPos.y - oMinPos.y + 1) * 
                (oMaxPos.z - oMinPos.z + 1);

    std::vector<DistanceIoVoxel::Ptr> vCorners;
    vCorners.reserve(pointNum);
    for(int x = oMinPos.x; x <= oMaxPos.x; ++x) {
    for(int y = oMinPos.y; y <= oMaxPos.y; ++y) {
    for(int z = oMinPos.z; z <= oMaxPos.z; ++z) {
        HashPos oPos(x, y, z);
        vCorners.push_back(CreateAndGetVoxel(oPos));
    }}}

    return vCorners;
}

static Eigen::Vector3f sub_offset[7] = {
    {0.5, 0.5, 0.5}, // center
    {0  , 0.5, 0.5}, {0.5, 0  , 0.5}, {0.5, 0.5, 0  }, // face
    {0.5, 0  , 0  }, {0  , 0.5, 0  }, {0  , 0  , 0.5}, // edge
};

static Eigen::Vector3f 

std::vector<DistanceIoVoxel::Ptr> DistanceIoVolume::CreateAndGetSubdivideCorners(const std::vector<DistanceIoVoxel::Ptr>& vCorners, size_t iLevel){

    size_t step = 1 << iLevel;

    for(int i = 0; i < 7; ++i) {
        std::vector<DistanceIoVoxel::Ptr> vSubCorners(new std::vector<DistanceIoVoxel::Ptr>(vCorners));
        // 前三行代表x,y,z, 每一列是一个点，进行平移。
        vSubCorners.getMatrixXfMap().topRows(3).colwise() += GetVoxelLength().cwiseProduct(sub_offset[i] * step);
        // corners.push_back(pSubCorners);

        // 删除非细分的点
        int delete_count = 0;
        for(int k = 0; k < vSubCorners->size()-delete_count; ++k) {
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

/**
 * @brief fuse local to global, use weight strategy
 * @param oLocal local volume object
*/
void DistanceIoVolume::Fuse(const DistanceIoVolume& oLocal) {


}

/**
 * 
*/
DistanceIoVoxel::Ptr DistanceIoVolume::CreateAndGetVoxel(HashPos& oPos) {
    
    if(!m_vVolume.count(oPos)) {
        pcl::PointNormal oCornerPoint = HashPosTo3DPos<pcl::PointNormal>(oPos);
        m_vVolume.insert(std::make_pair(oPos, DistanceIoVoxel::Ptr(new DistanceIoVoxel(oCornerPoint))));
    }

    return m_vVolume[oPos];
}


std::array<HashPos, 8> DistanceIoVolume::GetCornerPoses(const HashPos& oPos, size_t iLevel) {

    int step = 1 << iLevel;

    std::array<HashPos, 8> vCornerPoses;
	vCornerPoses.emplace_back(oPos.x, 		    oPos.y, 		oPos.z);
	vCornerPoses.emplace_back(oPos.x, 		    oPos.y + step,	oPos.z);
	vCornerPoses.emplace_back(oPos.x + step, 	oPos.y + step, 	oPos.z);
	vCornerPoses.emplace_back(oPos.x + step,	oPos.y, 		oPos.z);
	vCornerPoses.emplace_back(oPos.x, 		    oPos.y, 		oPos.z + step);
	vCornerPoses.emplace_back(oPos.x, 		    oPos.y + step, 	oPos.z + step);
	vCornerPoses.emplace_back(oPos.x + step,	oPos.y + step,	oPos.z + step);
	vCornerPoses.emplace_back(oPos.x + step, 	oPos.y,	 	    oPos.z + step);

    return vCornerPoses;
}