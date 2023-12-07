#include <memory>
#include <shared_mutex>
#include <sstream>

#include "volume/SimpleVolume.h"
#include "tools/RosPublishManager.h"

/**
 * @brief set voxel resolution and related params
 * @param oLength - 3d size of a voxel
*/
void SimpleVolume::SetResolution(pcl::PointXYZ & oLength) { 
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
void SimpleVolume::PointBelongVoxelPos(const PointType & oPoint, HashPos & oVoxelPos) const {
    
    oVoxelPos.x = floor(oPoint.x * m_vVoxelSizeInverse.x());
	oVoxelPos.y = floor(oPoint.y * m_vVoxelSizeInverse.y());
	oVoxelPos.z = floor(oPoint.z * m_vVoxelSizeInverse.z());
}
template void SimpleVolume::PointBelongVoxelPos(const pcl::PointXYZ & oPoint, HashPos & oVoxelPos) const;
template void SimpleVolume::PointBelongVoxelPos(const pcl::PointXYZI & oPoint, HashPos & oVoxelPos) const;
template void SimpleVolume::PointBelongVoxelPos(const pcl::PointNormal & oPoint, HashPos & oVoxelPos) const;

/**
 * @brief get hash pos according to point position
 * @param oPoint point with position
 * @return oPos - output block pos
*/
void SimpleVolume::PointBelongVoxelPos(const pcl::PointNormal & oPoint, HashPos & oVoxelPos) const {
    
    oVoxelPos.x = floor(oPoint.x * m_vVoxelSizeInverse.x());
	oVoxelPos.y = floor(oPoint.y * m_vVoxelSizeInverse.y());
	oVoxelPos.z = floor(oPoint.z * m_vVoxelSizeInverse.z());
}
void SimpleVolume::PointBelongVoxelPos(const Eigen::Vector3f & vPoint, HashPos & oVoxelPos) const {
    oVoxelPos.x = floor(vPoint.x() * m_vVoxelSizeInverse.x());
	oVoxelPos.y = floor(vPoint.y() * m_vVoxelSizeInverse.y());
	oVoxelPos.z = floor(vPoint.z() * m_vVoxelSizeInverse.z());
}

/**
 * @brief voxelize the points and fuse them
 * @param vCloud the input lidar cloud with normals
*/
void SimpleVolume::VoxelizePointsAndFusion(pcl::PointCloud<pcl::PointXYZI> & vCloud) {
        
    for(auto&& oPoint : vCloud) {
        const int& intensity = reinterpret_cast<int&>(oPoint.intensity);
        HashPos oPos;
        PointBelongVoxelPos(oPoint, oPos);
        if(!m_vVolume.count(oPos)) m_vVolume.emplace(oPos, new StatusVoxel);
        m_vVolume[oPos]->point.getVector3fMap() = oPoint.getVector3fMap();
        m_vVolume[oPos]->AddStatus(intensity);
    }
}

void SimpleVolume::PublishType(int intensity) {

    HashPosDic vTypePoses;
    for(auto&& [oPos, pVoxel] : m_vVolume) {
        int influence = pVoxel->GetStatus(intensity);
        if(influence > 0) {
            vTypePoses.emplace(oPos, influence > 4 ? 1 : 0);
        }
    }
    RosPublishManager::GetInstance().PublishVoxelSet(vTypePoses, m_vVoxelSize, "/mesh_voxel_debug");
}


void SimpleVolume::PublishType() {

    HashPosDic vTypePoses;
    for(auto&& [oPos, pVoxel] : m_vVolume) {
        int influence5 = pVoxel->GetStatus(5);

        // int influence6 = pVoxel->GetStatus(6);
        // int influence14 = pVoxel->GetStatus(14);
        // int sum = influence6 + influence14 - influence5;
        // if(sum > 0) {
        //     vTypePoses.emplace(oPos, sum > 4 ? 1 : 0);
        // }

        int influence15 = pVoxel->GetStatus(15);
        if(influence15 > 0) {
            vTypePoses.emplace(oPos, influence15 > 4 ? 1 : 0);
        }

        // int influence2 = pVoxel->GetStatus(2);
        // int influence6 = pVoxel->GetStatus(6);
        // int influence9 = pVoxel->GetStatus(9);
        // int sum = influence2 + influence6 + influence9;
        // if(influence2 > 4 && sum > 8 && influence5 > 0) {
        //     vTypePoses.emplace(oPos, sum >= 12 ? 1 : 0);
        // }
    }
    RosPublishManager::GetInstance().PublishVoxelSet(vTypePoses, m_vVoxelSize, "/mesh_voxel_debug");
}
