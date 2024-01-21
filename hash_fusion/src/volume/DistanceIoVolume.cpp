#include <memory>
#include <shared_mutex>
#include <sstream>

#include "volume/DistanceIoVolume.h"
#include "tools/RosPublishManager.h"
#include "tools/OutputUtils.h"
#include "tools/DebugManager.h"

/**
 * @brief set voxel resolution and related params
 * @param oLength - 3d size of a voxel
*/
void DistanceIoVolume::SetResolution(pcl::PointXYZ & oLength) { 
	m_vVoxelSize = oLength.getVector3fMap(); 
	m_vVoxelHalfSize = m_vVoxelSize * 0.5f;
	m_vVoxelSizeInverse = m_vVoxelSize.cwiseInverse();
    m_fStaticExpandDistance = m_vVoxelHalfSize.maxCoeff();
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
template void DistanceIoVolume::PointBelongVoxelPos(const pcl::DistanceIoVoxel & oPoint, HashPos & oVoxelPos) const;

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
        CreateAndGetVoxel(oPos).getVector3fMap() = oPoint.getVector3fMap();
        // m_vVolume[oPos]->AddStatus(intensity);
    }
}

/**
 * @brief create voxel corner in range, publish the pointers to updater in order to update in/out
 * @param vMin bounding box min point
 * @param vMax bounding box max point
 * @param iLevel level of the octree, from bottom to top: 0 - k
*/
pcl::PointCloud<pcl::DistanceIoVoxel> DistanceIoVolume::CreateAndGetCornersAABB(const Eigen::Vector3f& vMin, const Eigen::Vector3f& vMax, size_t iLevel){

    // TimeDebugger oTimeDebugger;
    // oTimeDebugger.NewLine();

    HashPos oMinPos, oMaxPos;
    PointBelongVoxelPos(vMin, oMinPos);
    PointBelongVoxelPos(vMax, oMaxPos);

    size_t step = 1 << iLevel;
    AlignToStepFloor(oMinPos, step);
    AlignToStepCeil(oMaxPos, step);

    int pointNum = ((oMaxPos.x - oMinPos.x)/step + 1) * 
                ((oMaxPos.y - oMinPos.y)/step + 1) * 
                ((oMaxPos.z - oMinPos.z)/step + 1);

    pcl::PointCloud<pcl::DistanceIoVoxel> vCorners;
    vCorners.points.reserve(pointNum);
    for(int x = oMinPos.x; x <= oMaxPos.x; x += step) {
    for(int y = oMinPos.y; y <= oMaxPos.y; y += step) {
    for(int z = oMinPos.z; z <= oMaxPos.z; z += step) {
        HashPos oPos(x, y, z);
        // vCorners.points.emplace_back(CreateAndGetVoxel(oPos));
        vCorners.points.emplace_back(HashPosTo3DPos<pcl::DistanceIoVoxel>(oPos));
    }}}

    // oTimeDebugger.DebugTime("create_corners");
    // oTimeDebugger.CoutCurrentLine();

    return vCorners;
}

/**
 * @brief create voxel corners by voxel poses
 * @param vVoxelPoses - poses of the voxels that ready to be updated
 * @param iLevel - the level of octree, larger means bigger voxel
*/
pcl::PointCloud<pcl::DistanceIoVoxel>::Ptr DistanceIoVolume::CreateAndGetCornersByPos(const HashPosSet& vVoxelPoses, size_t iLevel){

    HashPosSet vCornerPoses;
    for(const HashPos& oVoxelPos : vVoxelPoses) {
        auto vPoses = GetCornerPoses(oVoxelPos, iLevel);
        for(const HashPos& oPos : vPoses) vCornerPoses.emplace(oPos);
    }

    pcl::PointCloud<pcl::DistanceIoVoxel>::Ptr pCornerPoints(new pcl::PointCloud<pcl::DistanceIoVoxel>());
    for(const HashPos& oCornerPos : vCornerPoses) {
        pCornerPoints->push_back(CreateAndGetVoxel(oCornerPos));
    }

    return pCornerPoints;
}

static HashPos sub_pos[19] = {
    {1, 1, 1},
    {0, 1, 1}, {1, 0, 1}, {1, 1, 0}, {2, 1, 1}, {1, 2, 1}, {1, 1, 2},
    {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {2, 0, 1}, {2, 1, 0}, {1, 2, 0}, {0, 2, 1}, {1, 0, 2}, {0, 1, 2}, {1, 2, 2}, {2, 1, 2}, {2, 2, 1},
};

std::vector<pcl::PointCloud<pcl::DistanceIoVoxel>::Ptr> DistanceIoVolume::CreateAndGetSubdivideCorners(const pcl::PointCloud<pcl::DistanceIoVoxel>& vCorners, size_t iLevel){

    // TimeDebugger oTimeDebugger;
    // oTimeDebugger.NewLine();

    size_t step = 1 << iLevel;
    ++iLevel;

    std::vector<pcl::PointCloud<pcl::DistanceIoVoxel>::Ptr> corners;
    corners.reserve(7);

    HashPosSet record;
    for(auto&& oCorner : vCorners) {
        HashPos oPos;
        PointBelongVoxelPos(oCorner, oPos);
        record.emplace(oPos);
    }

    // oTimeDebugger.DebugTime("init_set");

    // record subdivide corners
    HashPosSet poses;
    for(auto&& oCorner : vCorners) {
        HashPos oPos;
        PointBelongVoxelPos(oCorner, oPos);
        size_t mask = 0;
        std::array<HashPos, 8> vCornerPoses = GetCornerPoses(oPos, iLevel);
        for(int j = 0; j < 8; ++j) {
            HashPos& oCornerPos = vCornerPoses[j];
            mask = mask << 1;
            if(record.count(oCornerPos)) {
            // if(m_vVolume.count(oCornerPos)) {
                pcl::DistanceIoVoxel& vCornerPoint = CreateAndGetVoxel(oCornerPos);
                if(vCornerPoint.io == 1.0f) mask |= 1;
                else if(mask) break;
            } else {
                mask = 0;
                break;
            }
        }
        if(mask > 0 && mask < 0xff) {
            for(auto&& pos : sub_pos) {
                poses.emplace(oPos + pos * step);
            }
        }
    }
    // oTimeDebugger.DebugTime("record_subdivide");

    // init corners vector
    for(int i = 0; i < 7; ++i) 
        corners.emplace_back(new pcl::PointCloud<pcl::DistanceIoVoxel>);

    // take corner
    int index = 0;
    for(auto&& pose : poses) {
        // corners[++index%7]->push_back(CreateAndGetVoxel(pose));
        corners[++index%7]->push_back(HashPosTo3DPos<pcl::DistanceIoVoxel>(pose));
    }

    // oTimeDebugger.DebugTime("subdivide");
    // oTimeDebugger.CoutCurrentLine();

    return corners;
}

void DistanceIoVolume::Update(pcl::PointCloud<pcl::DistanceIoVoxel>& vCorners) {

    for(auto& oCorner : vCorners) {
        
        float& update_io = oCorner.io;
        if(update_io == 0.0f && oCorner.distance < 1.0f)
            update_io = -1.0f;

        if(update_io == 0.0f) continue;
        HashPos oPos;
        PointBelongVoxelPos(oCorner, oPos);
        auto& oVoxel = CreateAndGetVoxel(oPos);
        
        // combine code
        std::unique_lock<std::mutex> lock(m_mVolumeDataMutex);
        oVoxel.io = oVoxel.io == 0.0f ? update_io : std::max(oVoxel.io, update_io);
        oVoxel.distance = std::max(oVoxel.distance, oCorner.distance);
    }
}

void DistanceIoVolume::UpdateLimitDistance(const pcl::PointCloud<pcl::DistanceIoVoxel>& vCorners, size_t iLevel) {

    const float limit_distance = m_fStaticExpandDistance * (1 << iLevel);

    for(auto& oCorner : vCorners) {

        if(oCorner.io == 0.0f) continue;
        HashPos oPos;
        PointBelongVoxelPos(oCorner, oPos);
        auto& oVoxel = CreateAndGetVoxel(oPos);

        // combine code
        std::unique_lock<std::mutex> lock(m_mVolumeDataMutex);
        oVoxel.io = oVoxel.io == 0.0f ? oCorner.io : std::max(oVoxel.io, oCorner.io);
        oVoxel.distance = std::max(oVoxel.distance, oCorner.distance);
        oVoxel.distance = std::min(oVoxel.distance, limit_distance);
    }
}

/**
 * @brief fuse local to global, use weight strategy
 * @param oLocal local volume object
*/
void DistanceIoVolume::Fuse(DistanceIoVolume& oLocal) {

    // fuse
    // 对于一种情况似乎需要特殊处理，当一个位置之前是细分区域，
    // 而这个位置在当前帧是非细分区域，这时的更新就要下放到低层级，这一步似乎非常拖慢速度
    // lazy 标记或许是一种方法?
    for(auto&& [oPos,oVoxelIndex] : oLocal.m_vVolume) {
        const pcl::DistanceIoVoxel& oLocalVoxel = oLocal.GetVoxelData(oVoxelIndex);
        if(oLocalVoxel.io == 0.0f) continue;
        pcl::DistanceIoVoxel& oGlobalVoxel = CreateAndGetVoxel(oPos);
        oGlobalVoxel.Update(oLocalVoxel.distance, oLocalVoxel.io == 1.0f);
    }
}

/**
 * 
*/
float DistanceIoVolume::SearchSdf(const Eigen::Vector3f& vPoint) {

    HashPos oPos;
    PointBelongVoxelPos(vPoint, oPos);
    // AlignToStepFloor(oPos, 1);

    std::unique_lock<std::mutex> lock(m_mVolumeDataMutex);
    for(int level = 0; level <= 3; ++level) {
        float res = InterpolateCorners(oPos, vPoint, level);
        if(res != -INFINITY) return res;
    }

    return -INFINITY;
}

/**
 * 
*/
pcl::DistanceIoVoxel& DistanceIoVolume::CreateAndGetVoxel(const HashPos& oPos) {
    
    if(!m_vVolume.count(oPos)) {
        std::unique_lock<std::mutex> lock(m_mVolumeDataMutex);
        if(!m_vVolume.count(oPos)) {
            if(m_vVolumeData.size() == 0 || m_vVolumeData.back().size() == m_vVolumeData.back().points.capacity()) {
                VolumeDataMoveNext();
            }
            m_vVolumeData.back().push_back(HashPosTo3DPos<pcl::DistanceIoVoxel>(oPos));
            m_vVolume.insert(std::make_pair(oPos, pcl::VoxelIndex(m_vVolumeData.size()-1, m_vVolumeData.back().size()-1)));
        }
    }

    pcl::VoxelIndex& index = m_vVolume[oPos];
    return GetVoxelData(index);
}

const pcl::DistanceIoVoxel* DistanceIoVolume::GetVoxel(const HashPos& oPos) {

    if(!m_vVolume.count(oPos)) return nullptr;
    pcl::VoxelIndex& index = m_vVolume[oPos];
    return &GetVoxelData(index);
}


std::array<HashPos, 8> DistanceIoVolume::GetCornerPoses(const HashPos& oPos, size_t iLevel) const {

    int step = 1 << iLevel;

    HashPos oBaseCorner(oPos);
    AlignToStepFloor(oBaseCorner, step);

    std::array<HashPos, 8> vCornerPoses {
        HashPos(oBaseCorner.x, 		    oBaseCorner.y, 		    oBaseCorner.z),
        HashPos(oBaseCorner.x + step,   oBaseCorner.y,	        oBaseCorner.z),
        HashPos(oBaseCorner.x, 	        oBaseCorner.y + step, 	oBaseCorner.z),
        HashPos(oBaseCorner.x + step,	oBaseCorner.y + step, 	oBaseCorner.z),
        HashPos(oBaseCorner.x, 		    oBaseCorner.y, 		    oBaseCorner.z + step),
        HashPos(oBaseCorner.x + step,   oBaseCorner.y, 	        oBaseCorner.z + step),
        HashPos(oBaseCorner.x,	        oBaseCorner.y + step,	oBaseCorner.z + step),
        HashPos(oBaseCorner.x + step, 	oBaseCorner.y + step,	oBaseCorner.z + step),
    };

    return vCornerPoses;
}

float DistanceIoVolume::GetSdf(const pcl::DistanceIoVoxel& oVoxel) const {

    return (oVoxel.io > 0.5f ? 1 : -1) * oVoxel.distance;
}

float DistanceIoVolume::InterpolateCorners(const HashPos& oPos, const Eigen::Vector3f& vPoint, size_t iLevel) {

    auto vCorners = GetCornerPoses(oPos, iLevel);
    std::vector<const pcl::DistanceIoVoxel*> pVoxels;
    for(auto&& oCorner : vCorners) {
        if(!m_vVolume.count(oCorner))
            return -INFINITY;
        else pVoxels.emplace_back(GetVoxel(oCorner));
    }

    float sum = 0;
    int step = 1 << iLevel;
    Eigen::Vector3f oRef = (vPoint - pVoxels[0]->getVector3fMap()).cwiseProduct(m_vVoxelSizeInverse) / step;
    float a = GetSdf(*pVoxels[0]) * (1 - oRef.x()) + GetSdf(*pVoxels[1]) * oRef.x();
    float b = GetSdf(*pVoxels[2]) * (1 - oRef.x()) + GetSdf(*pVoxels[3]) * oRef.x();
    float c = GetSdf(*pVoxels[4]) * (1 - oRef.x()) + GetSdf(*pVoxels[5]) * oRef.x();
    float d = GetSdf(*pVoxels[6]) * (1 - oRef.x()) + GetSdf(*pVoxels[7]) * oRef.x();
    float e = a * (1 - oRef.y()) + b * oRef.y();
    float f = c * (1 - oRef.y()) + d * oRef.y();
    return e * (1 - oRef.z()) + f * oRef.z();
}

namespace pcl {
  std::ostream& operator << (std::ostream& os, const DistanceIoVoxel& p) {
    os << p.getVector3fMap().transpose() << " " << p.io << " " << p.distance << " " << p.weight;
  }
}