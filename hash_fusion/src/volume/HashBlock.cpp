#include <memory>
#include <shared_mutex>

#include "HashBlock.h"
#include "Fusion.h"
#include "SignedDistance.h"

/**
 * @brief init default params
*/
HashBlock::HashBlock() : 
    m_iFrameCount(0), 
    m_iMaxRecentKeep(500), 
    m_fLidarSpeed(0), 
    m_fRadiusExpandFactor(1.25f), 
    m_iRecentTimeToGetRadius(30) {}

/**
 * @brief placeholder
*/
HashBlock::~HashBlock() {}


/**
 * @brief get hash pos according to point position
 * @param oPoint point with position
 * @return oPos - output block pos
*/
template<class PointType>
void HashBlock::PointBelongBlockPos(const PointType & oPoint, HashPos & oBlockPos) const {
    
    oBlockPos.x = floor(oPoint.x * m_vBlockSizeInverse.x());
	oBlockPos.y = floor(oPoint.y * m_vBlockSizeInverse.y());
	oBlockPos.z = floor(oPoint.z * m_vBlockSizeInverse.z());
}
template void HashBlock::PointBelongBlockPos(const pcl::PointXYZ & oPoint, HashPos & oBlockPos) const;
template void HashBlock::PointBelongBlockPos(const pcl::PointNormal & oPoint, HashPos & oBlockPos) const;


/**
 * @brief get hash pos according to point position
 * @param oPoint point with position
 * @return oPos - output block pos
*/
template<class PointType>
void HashBlock::PointBelongVoxelPos(const PointType & oPoint, HashPos & oVoxelPos) const {
    
    oVoxelPos.x = floor(oPoint.x * m_vVoxelSizeInverse.x());
	oVoxelPos.y = floor(oPoint.y * m_vVoxelSizeInverse.y());
	oVoxelPos.z = floor(oPoint.z * m_vVoxelSizeInverse.z());
}
template void HashBlock::PointBelongVoxelPos(const pcl::PointXYZ & oPoint, HashPos & oVoxelPos) const;
template void HashBlock::PointBelongVoxelPos(const pcl::PointNormal & oPoint, HashPos & oVoxelPos) const;


/**
 * @brief get voxel index according to point position, please use it before in-block check
 * @param oPoint point with position
 * @return oPos - output block pos
*/
template<class PointType>
void HashBlock::PointBelongVoxelIndex(const PointType & oPoint, int & iIndex) const {
    
	HashPos oBlockPos;
	PointBelongBlockPos(oPoint, oBlockPos);
	Eigen::Vector3f vPointOffset(oBlockPos.x, oBlockPos.y, oBlockPos.z);
	vPointOffset = vPointOffset.cwiseProduct(-m_vBlockSize);
	vPointOffset += oPoint.getVector3fMap();
	Eigen::Vector3i vPointPos = vPointOffset.cwiseProduct(m_vVoxelSizeInverse).array().floor().cast<int>();
	iIndex = (vPointPos.z() * m_vVoxelNumsPerBlock.y() + vPointPos.y()) * m_vVoxelNumsPerBlock.x() + vPointPos.z();
}
template void HashBlock::PointBelongVoxelIndex(const pcl::PointXYZ & oPoint, int & iIndex) const;
template void HashBlock::PointBelongVoxelIndex(const pcl::PointNormal & oPoint, int & iIndex) const;


/**
 * @brief voxelize the points and fuse them
 * @param vCloud the input lidar cloud with normals
*/
void HashBlock::VoxelizePointsAndFusion(pcl::PointCloud<pcl::PointNormal> & vCloud) {
    
	// put points into blocks
    HashIndices vPointContainer;
	HashPos oCurrentPos;
	for(int i = 0; i < vCloud.size(); ++i) {
		const pcl::PointNormal & oCurrentPoint = vCloud.at(i);
		PointBelongBlockPos(oCurrentPoint, oCurrentPos);
		vPointContainer[oCurrentPos].emplace_back(i);
	}

	// create or update blocks
	std::unique_lock<std::shared_mutex> volume_write_lock(m_mVolumeLock);
	++m_iFrameCount;
    for(auto&& [oPos, vIndices] : vPointContainer) {
		Block& vBlock = m_vBlockVolume[oPos];
		if(vBlock.empty()) vBlock.resize(m_iVoxelFullNumPerBlock);
        FusePointToBlock(vCloud, vBlock, vIndices);
    }
}

/**
 * @brief voxelize the points in the block
 * @param vCloud the input lidar cloud with normals
 * @param vBlock the reference of processed block
 * @param vIndices indices of points in the processed block
*/
void HashBlock::FusePointToBlock(pcl::PointCloud<pcl::PointNormal> & vCloud, Block& vBlock, Indices& vIndices) {

	// put point into voxels
    std::unordered_map<size_t, std::vector<int>> vPointContainer;
	int iVoxelIndex = 0;
	for(int iCloudIndex = 0; iCloudIndex < vIndices.size(); ++iCloudIndex) {
		const pcl::PointNormal & oCurrentPoint = vCloud.at(iCloudIndex);
		PointBelongVoxelIndex(oCurrentPoint, iVoxelIndex);
		vPointContainer[iVoxelIndex].emplace_back(iCloudIndex);
	}

	// fusion the points
	Fusion oVoxelFusion;
	for(auto&& [iVoxelIndex, iCloudIndices] : vPointContainer) {

		// init base point
		VoxelBase& oCurrentVoxel = vBlock[iVoxelIndex];
		pcl::PointNormal oBasePoint = oCurrentVoxel.bIsOccupied ? oCurrentVoxel.point : pcl::PointNormal();

		// fusion points
		pcl::PointNormal oFusedPoint = oVoxelFusion.NormalFusionWeighted(iCloudIndices, vCloud, oBasePoint);
		float& fConflictTime = oFusedPoint.data_c[1];
		m_pUpdateStrategy->Support(fConflictTime);
		float& fTimeStamp = oFusedPoint.data_c[2];
		fTimeStamp = m_iFrameCount;
		oCurrentVoxel.point = oFusedPoint;
	}
}

/**
 * @brief decrease the confidence of dynamic point and record conflict
 * @param vVolumeCloud - the conflict result that calculated by the 'surfel fusion' liked method
 * @return m_vVolume - update the volume
*/
void HashBlock::UpdateConflictResult(const pcl::PointCloud<pcl::PointNormal> & vVolumeCloud, const bool bKeepVoxel) {

	std::unique_lock<std::shared_mutex> volume_write_lock(m_mVolumeLock);

	HashPos oPos;
	for(auto && oVoxelPoint : vVolumeCloud) {

		HashPos oBlockPos;
		int iVoxelIndex;
		PointBelongBlockPos(oVoxelPoint, oBlockPos);
		PointBelongVoxelIndex(oVoxelPoint, iVoxelIndex);

		const float& fDeconfidence = oVoxelPoint.data_n[3];

		if(fDeconfidence < 0 && m_vBlockVolume.count(oBlockPos)) {

			Block& vBlock = m_vBlockVolume[oBlockPos];
			VoxelBase& oCurrentVoxel = vBlock[iVoxelIndex];
			if(!oCurrentVoxel.bIsOccupied) continue;

			float& fConflictTime = oCurrentVoxel.point.data_c[1];
			bool bToDelete = m_pUpdateStrategy->Conflict(fConflictTime);

			if(bToDelete && !bKeepVoxel) {
				oCurrentVoxel.bIsOccupied = false;
			} 
			else {
				float& fConfidence = oCurrentVoxel.point.data_n[3];
				fConfidence -= fDeconfidence;
				if(fConfidence < 0) fConfidence = 0;
			}
		}
	}
}


/**
 * @brief get the whole volume without dynamic voxels
 * @return vVolumeCopy - the output volume result
*/
void HashBlock::GetStaticVolume(HashVolume & vVolumeCopy) const {

	vVolumeCopy.clear();
	std::shared_lock<std::shared_mutex> volume_read_lock(m_mVolumeLock);
	for(auto && [oBlockPos, vBlock] : m_vBlockVolume) {
		for(auto && vVoxel : vBlock) {
			if(vVoxel.bIsOccupied) {
				HashPos oVoxelPos;
				PointBelongVoxelPos(vVoxel.point, oVoxelPos);
				vVolumeCopy[oVoxelPos] = vVoxel.point;
			}
		}
	}
}


/**
 * @brief get the whole cloud of volume
 * @return vVolumeCloud - the output cloud result
*/
void HashBlock::GetVolumeCloud(pcl::PointCloud<pcl::PointNormal> & vVolumeCloud) const {

	vVolumeCloud.clear();
	std::shared_lock<std::shared_mutex> volume_read_lock(m_mVolumeLock);
	for(auto && [oBlockPos, vBlock] : m_vBlockVolume) {
		for(auto && vVoxel : vBlock) {
			if(vVoxel.bIsOccupied) {
				vVolumeCloud.push_back(vVoxel.point);
			}
		}
	}
}