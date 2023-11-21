#include <memory>
#include <shared_mutex>
#include <sstream>

#include "volume/DistanceIoVolume.h"
#include "tools/RosPublishManager.h"
#include "tools/OutputUtils.h"

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
pcl::PointCloud<pcl::DistanceIoVoxel> DistanceIoVolume::CreateAndGetCorners(const Eigen::Vector3f& vMin, const Eigen::Vector3f& vMax, size_t iLevel){

    HashPos oMinPos, oMaxPos;
    PointBelongVoxelPos(vMin, oMinPos);
    PointBelongVoxelPos(vMax, oMaxPos);

    size_t step = 1 << iLevel;
    AlignToStepFloor(oMinPos, step);
    AlignToStepCeil(oMaxPos, step);

    int pointNum = (oMaxPos.x - oMinPos.x + 1) * 
                (oMaxPos.y - oMinPos.y + 1) * 
                (oMaxPos.z - oMinPos.z + 1);

    pcl::PointCloud<pcl::DistanceIoVoxel> vCorners;
    vCorners.points.reserve(pointNum);
    for(int x = oMinPos.x; x <= oMaxPos.x; x += step) {
    for(int y = oMinPos.y; y <= oMaxPos.y; y += step) {
    for(int z = oMinPos.z; z <= oMaxPos.z; z += step) {
        HashPos oPos(x, y, z);
        vCorners.points.emplace_back(CreateAndGetVoxel(oPos));
    }}}

    return vCorners;
}

static Eigen::Vector3f sub_offset[7] = {
    {0.5, 0.5, 0.5}, // center
    {0  , 0.5, 0.5}, {0.5, 0  , 0.5}, {0.5, 0.5, 0  }, // face
    {0.5, 0  , 0  }, {0  , 0.5, 0  }, {0  , 0  , 0.5}, // edge
};

// static Eigen::Vector3f 

std::vector<pcl::PointCloud<pcl::DistanceIoVoxel>::Ptr> DistanceIoVolume::CreateAndGetSubdivideCorners(const pcl::PointCloud<pcl::DistanceIoVoxel>& vCorners, size_t iLevel){

    size_t step = 1 << iLevel;

    std::vector<pcl::PointCloud<pcl::DistanceIoVoxel>::Ptr> corners;
    corners.reserve(7);

    for(int i = 0; i < 7; ++i) {
        pcl::PointCloud<pcl::DistanceIoVoxel>::Ptr pSubCorners(new pcl::PointCloud<pcl::DistanceIoVoxel>);
        pSubCorners->points.assign(vCorners.points.begin(), vCorners.points.end());
        // 前三行代表x,y,z, 每一列是一个点，进行平移。
        // std::cout << pSubCorners->back().getVector3fMap() << " ";
        pSubCorners->getMatrixXfMap().topRows(3).colwise() += GetVoxelLength().cwiseProduct(sub_offset[i] * step);
        // std::cout << pSubCorners->back().getVector3fMap() << "\n";
        corners.push_back(pSubCorners);

        // 删除非细分的点
        int delete_count = 0;
        for(int k = 0; k < pSubCorners->size()-delete_count; ++k) {
            int mask = 0;
            HashPos oPos;
            PointBelongVoxelPos(pSubCorners->at(k), oPos);
            AlignToStepFloor(oPos, step);
            std::array<HashPos, 8> vCornerPoses = GetCornerPoses(oPos, iLevel);
            for(int j = 0; j < 8; ++j) {
                HashPos& oCornerPos = vCornerPoses[j];
                mask = mask << 1;
                if(m_vVolume.count(oCornerPos)) {
                    pcl::DistanceIoVoxel& vCornerPoint = CreateAndGetVoxel(oCornerPos);
                    if(vCornerPoint.io == 1.0f) mask |= 1;
                } else {
                    mask = 0;
                    break;
                }
            }
            bool subdivision = mask > 0 && mask < 0xff;
            if(!subdivision) {
                ++delete_count;
                std::swap(pSubCorners->at(k), pSubCorners->at(pSubCorners->size()-delete_count));
                --k;
            }
        }
        pSubCorners->erase(pSubCorners->end()-delete_count, pSubCorners->end());

        // ROS_INFO_PURPLE("sub_divide: %d->%d", delete_count, vCorners.size(), pSubCorners->size());
    }

    return corners;
}

std::mutex m_mVolumeDataMutex;
void DistanceIoVolume::Update(const pcl::PointCloud<pcl::DistanceIoVoxel>& vCorners) {

    for(auto& oCorner : vCorners) {
        HashPos oPos;
        PointBelongVoxelPos(oCorner, oPos);
        if(oCorner.io == 1.0f) {
            auto& oVoxel = CreateAndGetVoxel(oPos);
            m_mVolumeDataMutex.lock();
            // fuse code
            oVoxel.io++;
            m_mVolumeDataMutex.unlock();
        }
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
pcl::DistanceIoVoxel& DistanceIoVolume::CreateAndGetVoxel(HashPos& oPos) {
    
    if(!m_vVolume.count(oPos)) {
        if(m_vVolumeData.size() == 0 || m_vVolumeData.back().size() == m_vVolumeData.back().points.capacity()) {
            VolumeDataMoveNext();
        }
        m_vVolumeData.back().push_back(HashPosTo3DPos<pcl::DistanceIoVoxel>(oPos));
        m_vVolume.insert(std::make_pair(oPos, pcl::VoxelIndex(m_vVolumeData.size()-1, m_vVolumeData.back().size()-1)));
    }
    pcl::VoxelIndex& index = m_vVolume[oPos];
    return m_vVolumeData[index.arr_index][index.cloud_index];
}


std::array<HashPos, 8> DistanceIoVolume::GetCornerPoses(const HashPos& oPos, size_t iLevel) {

    int step = 1 << iLevel;

    std::array<HashPos, 8> vCornerPoses {
        HashPos(oPos.x, 		    oPos.y, 		oPos.z),
        HashPos(oPos.x, 		    oPos.y + step,	oPos.z),
        HashPos(oPos.x + step, 	    oPos.y + step, 	oPos.z),
        HashPos(oPos.x + step,	    oPos.y, 		oPos.z),
        HashPos(oPos.x, 		    oPos.y, 		oPos.z + step),
        HashPos(oPos.x, 		    oPos.y + step, 	oPos.z + step),
        HashPos(oPos.x + step,	    oPos.y + step,	oPos.z + step),
        HashPos(oPos.x + step, 	    oPos.y,	 	    oPos.z + step),
    };

    return vCornerPoses;
}

namespace pcl {
  std::ostream& operator << (std::ostream& os, const DistanceIoVoxel& p) {
    os << p.getVector3fMap().transpose() << " " << p.io << " " << p.distance << " " << p.weight;
  }
}