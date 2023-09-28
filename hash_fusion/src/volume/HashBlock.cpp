#include <memory>
#include <shared_mutex>
#include <sstream>

#include "volume/HashBlock.h"
#include "Fusion.h"
#include "SignedDistance.h"
#include "tools/CubeIntersection.h"

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
 * @brief set voxel resolution and related params
 * @param oLength - 3d size of a voxel
*/
void HashBlock::SetResolution(pcl::PointXYZ & oLength) { 
	constexpr int voxel_num_per_block = 16;
	m_vVoxelNumsPerBlock = Eigen::Vector3i(voxel_num_per_block, voxel_num_per_block, voxel_num_per_block);
	m_iVoxelFullNumPerBlock = m_vVoxelNumsPerBlock.prod();
	m_vVoxelSize = oLength.getVector3fMap(); 
	m_vVoxelHalfSize = m_vVoxelSize * 0.5f;
	m_vVoxelSizeInverse = m_vVoxelSize.cwiseInverse();
	m_vBlockSize = m_vVoxelSize.cwiseProduct(m_vVoxelNumsPerBlock.cast<float>());
	m_vBlockSizeInverse = m_vBlockSize.cwiseInverse();
}


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
template void HashBlock::PointBelongBlockPos(const pcl::PointXYZI & oPoint, HashPos & oBlockPos) const;
template void HashBlock::PointBelongBlockPos(const pcl::PointNormal & oPoint, HashPos & oBlockPos) const;


/**
 * @brief get hash pos according to point position
 * @param oPoint point with position
 * @return oPos - output block pos
*/
void HashBlock::PointBelongVoxelPos(const pcl::PointNormal & oPoint, HashPos & oVoxelPos) const {
    
    oVoxelPos.x = floor(oPoint.x * m_vVoxelSizeInverse.x());
	oVoxelPos.y = floor(oPoint.y * m_vVoxelSizeInverse.y());
	oVoxelPos.z = floor(oPoint.z * m_vVoxelSizeInverse.z());
}
void HashBlock::PointBelongVoxelPos(const Eigen::Vector3f & vPoint, HashPos & oVoxelPos) const {
    oVoxelPos.x = floor(vPoint.x() * m_vVoxelSizeInverse.x());
	oVoxelPos.y = floor(vPoint.y() * m_vVoxelSizeInverse.y());
	oVoxelPos.z = floor(vPoint.z() * m_vVoxelSizeInverse.z());
}

/**
 * @brief get voxel index according to point position, please use it before in-block check
 * @param oPoint point with position
 * @return oPos - output block pos
*/
template<class PointType>
void HashBlock::PointBelongVoxelIndex(const PointType & oPoint, int & iIndex) const {
    
    // std::cout << output::format_red << "intput point is: " << oPoint.getVector3fMap().transpose() << " | ";
	HashPos oBlockPos;
	PointBelongBlockPos(oPoint, oBlockPos);
	// std::cout << "block is: " << oBlockPos << " | ";
	Eigen::Vector3f vPointOffset(oBlockPos.x, oBlockPos.y, oBlockPos.z);
	vPointOffset = vPointOffset.cwiseProduct(-m_vBlockSize);
	vPointOffset += oPoint.getVector3fMap();
	Eigen::Vector3i vPointPos = vPointOffset.cwiseProduct(m_vVoxelSizeInverse).array().floor().cast<int>();
	// std::cout << "voxel is: " << vPointPos.transpose() << " | ";
	iIndex = (vPointPos.z() * m_vVoxelNumsPerBlock.y() + vPointPos.y()) * m_vVoxelNumsPerBlock.x() + vPointPos.x();
	// std::cout << "index is: " << iIndex << output::format_white << std::endl;
}
template void HashBlock::PointBelongVoxelIndex(const pcl::PointXYZ & oPoint, int & iIndex) const;
template void HashBlock::PointBelongVoxelIndex(const pcl::PointNormal & oPoint, int & iIndex) const;



template<class PointType>
void HashBlock::PointBelongVoxelIndex(const PointType & oPoint, HashPos & oBlockPos, int & iVoxelIndex) const {
	
	PointBelongBlockPos(oPoint, oBlockPos);
	Eigen::Vector3f vPointOffset(oBlockPos.x, oBlockPos.y, oBlockPos.z);
	vPointOffset = vPointOffset.cwiseProduct(-m_vBlockSize);
	vPointOffset += oPoint.getVector3fMap();
	Eigen::Vector3i vPointPos = vPointOffset.cwiseProduct(m_vVoxelSizeInverse).array().floor().cast<int>();
	iVoxelIndex = (vPointPos.z() * m_vVoxelNumsPerBlock.y() + vPointPos.y()) * m_vVoxelNumsPerBlock.x() + vPointPos.x();
}
template void HashBlock::PointBelongVoxelIndex(const pcl::PointXYZ & oPoint, HashPos & oBlockPos, int & iVoxelIndex) const;
template void HashBlock::PointBelongVoxelIndex(const pcl::PointNormal & oPoint, HashPos & oBlockPos, int & iVoxelIndex) const;


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
		Block::Ptr& pBlock = m_vBlockVolume[oPos];
		if(pBlock == nullptr) 
			pBlock = new Block(m_iVoxelFullNumPerBlock, oPos);
        FusePointToBlock(vCloud, *pBlock, vIndices);
    }

	/* 目前的问题是，有相当一部分的voxel，其占用队列的更新停滞了
	// debug free voxels
	static int update_count = 0;
	if(update_count++ % 30 != 0) return;
	std::vector<float> associate;
	pcl::PointCloud<pcl::PointNormal> vFreeCloud;
	for(auto&& [oBlockPos, pBlock] : m_vBlockVolume) {
		for(int i = 0; i < pBlock->size(); ++i) {
			VoxelBase& oVoxel = pBlock->at(i);
			if(oVoxel.IsFreeSpace()) {
				pcl::PointNormal oPoint;
				oPoint.getVector3fMap() = HashVoxelIndexTo3DPos(i) + HashBlockPosTo3DPos(pBlock->pos);
				vFreeCloud.push_back(oPoint);
				associate.push_back(__builtin_popcount(oVoxel.iStatusQueue) * 0.1f);
			}
		}
	}
	RosPublishManager::GetInstance().PublishPointCloud(vFreeCloud, associate, "/debug_free_voxel");
	//*/
}


/**
 * @brief voxelize the points in the block
 * @param vCloud the input lidar cloud with normals
 * @param oBlock the reference of processed block
 * @param vIndices indices of points in the processed block
*/
void HashBlock::FusePointToBlock(pcl::PointCloud<pcl::PointNormal> & vCloud, Block& oBlock, Indices& vIndices) {

	// put point into voxels
    std::unordered_map<size_t, std::vector<int>> vPointContainer;
	int iVoxelIndex = 0;
	for(size_t& iCloudIndex : vIndices) {
		const pcl::PointNormal & oCurrentPoint = vCloud.at(iCloudIndex);
		PointBelongVoxelIndex(oCurrentPoint, iVoxelIndex);
		vPointContainer[iVoxelIndex].emplace_back(iCloudIndex);
	}

	// fusion the points
	Fusion oVoxelFusion;
	for(auto&& [iVoxelIndex, iCloudIndices] : vPointContainer) {

		// init queue
		VoxelBase& oCurrentVoxel = oBlock[iVoxelIndex];
		oCurrentVoxel.PushQueue(VoxelBase::Occupied);
		if(oCurrentVoxel.TypeByQueue() == VoxelBase::Free) {
			continue;
		}

		// fusion points
		pcl::PointNormal oBasePoint = oCurrentVoxel.IsUnknown() ? pcl::PointNormal() : oCurrentVoxel.point;
		pcl::PointNormal oFusedPoint = oVoxelFusion.NormalFusionWeighted(iCloudIndices, vCloud, oBasePoint);
		float& fConflictTime = oFusedPoint.data_c[1];
		m_pUpdateStrategy->Support(fConflictTime);
		float& fTimeStamp = oFusedPoint.data_c[2];
		fTimeStamp = m_iFrameCount;
		oCurrentVoxel.point = oFusedPoint;
		oCurrentVoxel.SetOccupied();
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

			Block& oBlock = *m_vBlockVolume[oBlockPos];
			VoxelBase& oCurrentVoxel = oBlock[iVoxelIndex];
			if(!oCurrentVoxel.IsOccupied()) continue;

			float& fConflictTime = oCurrentVoxel.point.data_c[1];
			bool bToDelete = m_pUpdateStrategy->Conflict(fConflictTime);

			if(bToDelete && !bKeepVoxel) {
				oCurrentVoxel.SetFreeSpace();
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
 * @brief get the whole volume include both dynamic and static voxels
 * @return vVolumeCopy - the output volume result
*/
void HashBlock::GetAllVolume(HashVolume & vVolumeCopy) const {

	vVolumeCopy.clear();
	std::shared_lock<std::shared_mutex> volume_read_lock(m_mVolumeLock);
	
	for(auto && [oBlockPos, pBlock] : m_vBlockVolume) {
		for(auto && vVoxel : *pBlock) {
			if(!vVoxel.IsUnknown()) {
				HashPos oVoxelPos;
				PointBelongVoxelPos(vVoxel.point, oVoxelPos);
				vVolumeCopy[oVoxelPos] = vVoxel.point;
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
	for(auto && [oBlockPos, pBlock] : m_vBlockVolume) {
		for(auto && oVoxel : *pBlock) {
			if(oVoxel.IsOccupied()
				&& m_pUpdateStrategy->IsStatic(oVoxel.point.data_c[1])) {
				HashPos oVoxelPos;
				PointBelongVoxelPos(oVoxel.point, oVoxelPos);
				vVolumeCopy[oVoxelPos] = oVoxel.point;
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
	for(auto && [oBlockPos, pBlock] : m_vBlockVolume) {
		for(auto && vVoxel : *pBlock) {
			if(vVoxel.IsOccupied()) {
				vVolumeCloud.push_back(vVoxel.point);
			}
		}
	}
}


/**
 * @brief get recent & static voxels in volume
 * @param iRecentTime - how long (second) the recent time is 
 * @return vVolumeCopy - the return volume
*/
void HashBlock::GetRecentVolume(HashVolume & vVolumeCopy, const int iRecentTime) const {

	vVolumeCopy.clear();
	std::shared_lock<std::shared_mutex> volume_read_lock(m_mVolumeLock);

	for(auto && [oBlockPos, pBlock] : m_vBlockVolume) {
		for(auto && oVoxel : *pBlock) {

			if(oVoxel.IsOccupied()
				&& m_iFrameCount - oVoxel.point.data_c[2] <= (uint32_t)iRecentTime 
				&& m_pUpdateStrategy->IsStatic(oVoxel.point.data_c[1])) {

					HashPos oPos;
					PointBelongVoxelPos(oVoxel.point, oPos);
					vVolumeCopy[oPos] = oVoxel.point;
			}
		}
	}
}


/**
 * @brief get recent & static & connected voxels in volume
 * @param iRecentTime how long (second) the recent time is
 * @param iConnectMinSize min connected set size of static voxel set
 * @return vVolumeCopy - the return volume
*/
void HashBlock::GetRecentConnectVolume(HashVolume & vVolumeCopy, const int iRecentTime, const int iConnectMinSize) {

	vVolumeCopy.clear();
	std::shared_lock<std::shared_mutex> union_read_lock(m_mUnionSetLock);
	std::shared_lock<std::shared_mutex> volume_read_lock(m_mVolumeLock);
	for(auto && [oBlockPos, pBlock] : m_vBlockVolume) {
		for(auto && oVoxel : *pBlock) {

			HashPos oPos;
			PointBelongVoxelPos(oVoxel.point, oPos);

			if(oVoxel.IsOccupied()
				&& m_iFrameCount - oVoxel.point.data_c[2] <= (uint32_t)iRecentTime 
				&& m_pUpdateStrategy->IsStatic(oVoxel.point.data_c[1])
				&& m_oUnionSet.GetSetSize(oPos) > iConnectMinSize) {

					vVolumeCopy[oPos] = oVoxel.point;
			}
		}
	}
}


/**
 * @brief get static & connected voxels in volume
 * @param iRecentTime - how long (second) the recent time is 
 * @param iConnectMinSize min connected set size of static voxel set
 * @return vVolumeCopy - the return volume
*/
void HashBlock::GetAllConnectVolume(HashVolume & vVolumeCopy, const int iConnectMinSize) {

	vVolumeCopy.clear();
	std::shared_lock<std::shared_mutex> union_read_lock(m_mUnionSetLock);
	std::shared_lock<std::shared_mutex> volume_read_lock(m_mVolumeLock);
	for(auto && [oBlockPos, pBlock] : m_vBlockVolume) {
		for(auto && oVoxel : *pBlock) {

			HashPos oPos;
			PointBelongVoxelPos(oVoxel.point, oPos);

			if(oVoxel.IsOccupied()
				&& m_pUpdateStrategy->IsStatic(oVoxel.point.data_c[1])
				&& m_oUnionSet.GetSetSize(oPos) > iConnectMinSize) {

					vVolumeCopy[oPos] = oVoxel.point;
			}
		}
	}
}


/**
 * @brief get local area & static voxels in volume
 * @param vCenter - center position of the area
 * @param fRadius - radius of the area
*/
void HashBlock::GetLocalVolume(HashVolume & vVolumeCopy, const Eigen::Vector3f vCenter, const float fRadius) const {

	vVolumeCopy.clear();
	std::shared_lock<std::shared_mutex> volume_read_lock(m_mVolumeLock);

	for(auto && [oBlockPos, pBlock] : m_vBlockVolume) {
		for(auto && oVoxel : *pBlock) {

			HashPos oPos;
			PointBelongVoxelPos(oVoxel.point, oPos);

			Eigen::Vector3f vCurrent(oPos.x, oPos.y, oPos.z);
			if(oVoxel.IsOccupied()
				&& (vCurrent - vCenter).norm() <= fRadius 
				&& m_pUpdateStrategy->IsStatic(oVoxel.point.data_c[1]))
				vVolumeCopy[oPos] = oVoxel.point;
		}
	}
}


/**
 * @brief building union set of current local volume
 * @param fStrictDotRef the threshold of normal similarity (larger)
 * @param fSoftDotRef the threshold of normal similarity (smaller)
 * @param fConfidenceLevelLength the scaling of confidence, larger means less confidence, due to different speed of vehicle, we should set this param.
 * @return void, but m_oUnionSet is calculated
*/
void HashBlock::RebuildUnionSet(const float fStrictDotRef, const float fSoftDotRef, const float fConfidenceLevelLength) {

	// copy avoid of block
	HashVolume vVolumeCopy;
	GetRecentVolume(vVolumeCopy, m_iRecentTimeToGetRadius);
	
	// get center and radius
	Eigen::Vector3f vCenter(0, 0, 0);
	for(auto && [oPos,_] : vVolumeCopy) {
		Eigen::Vector3f vCurrent(oPos.x, oPos.y, oPos.z);
		vCenter += vCurrent;
	}
	vCenter /= vVolumeCopy.size();
	float fRadius = 0.0f;
	for(auto && [oPos, _] : vVolumeCopy) {
		Eigen::Vector3f vCurrent(oPos.x, oPos.y, oPos.z);
		fRadius = max(fRadius, (vCurrent - vCenter).norm());
	}
	fRadius *= m_fRadiusExpandFactor;
	GetLocalVolume(vVolumeCopy, vCenter, fRadius);

	RebuildUnionSetCore(vVolumeCopy, fStrictDotRef, fSoftDotRef, fConfidenceLevelLength);
}


/**
 * @brief building union set of the whole volume, usually for output
 * @param fStrictDotRef the threshold of normal similarity (larger)
 * @param fSoftDotRef the threshold of normal similarity (smaller)
 * @param fConfidenceLevelLength the scaling of confidence, larger means less confidence, due to different speed of vehicle, we should set this param.
 * @return void, but m_oUnionSet is calculated
*/
void HashBlock::RebuildUnionSetAll(const float fStrictDotRef, const float fSoftDotRef, const float fConfidenceLevelLength) {

	HashVolume vVolumeCopy;
	GetStaticVolume(vVolumeCopy);
	RebuildUnionSetCore(vVolumeCopy, fStrictDotRef, fSoftDotRef, fConfidenceLevelLength);
}


/**
 * @brief the core function of building union set of a volume
 * @param vVolumeCopy the volume input
 * @param fStrictDotRef the threshold of normal similarity (larger)
 * @param fSoftDotRef the threshold of normal similarity (smaller)
 * @param fConfidenceLevelLength the scaling of confidence, larger means less confidence, due to different speed of vehicle, we should set this param.
 * @return void, but m_oUnionSet is calculated
*/
void HashBlock::RebuildUnionSetCore(HashVolume & vVolumeCopy, const float fStrictDotRef, const float fSoftDotRef, const float fConfidenceLevelLength) {

	std::unique_lock<std::shared_mutex> union_write_lock(m_mUnionSetLock);
	m_oUnionSet.Clear();
	
	std::unique_ptr<CubeIntersection> oIntersection(new CubeIntersection());
	for(auto && [oPos, oPoint] : vVolumeCopy) {

		// 置信度分级 0 - 4
		int confidence = oPoint.data_n[3] / fConfidenceLevelLength;
		if(confidence > 4) confidence = 4; // 4 is the max conf level
		// 时间戳
		int time_stamp = oPoint.data_c[2];
		constexpr int time_diff_ref = 30;
		
		// 面片连通性 Ax + By + Cz + D = 0 (+法向限制）
		float normal_dot_ref = m_pUpdateStrategy->IsSoftDynamic(oPoint.data_c[1]) ? fStrictDotRef : fSoftDotRef;
		// 找到左下角角点
		pcl::PointXYZ oPosPoint = HashPosTo3DPos(oPos);

		// 计算面相交
		oIntersection->Reset(oPoint, GetVoxelLength(), oPosPoint.getVector3fMap());
		bool connect_neg[3] = {
			oIntersection->CrossNegX() || confidence == 4,
			oIntersection->CrossNegY() || confidence == 4,
			oIntersection->CrossNegZ() || confidence == 4
		};

		// 遍历相交的邻体素
		const Eigen::Vector3i vPosFix[3] = {
			{-1, 0, 0},
			{0, -1, 0},
			{0, 0, -1}
		};
		for(int iAxisId = 0; iAxisId < 3; ++iAxisId) {

			HashPos oNearPos(oPos.x + vPosFix[iAxisId].x(), oPos.y + vPosFix[iAxisId].y(), oPos.z + vPosFix[iAxisId].z());

			if(connect_neg[iAxisId] && vVolumeCopy.count(oNearPos)) {

				pcl::PointNormal& oNearPoint = vVolumeCopy[oNearPos];
				pcl::PointXYZ oPosPoint = HashPosTo3DPos(oNearPos);

				// 物体若置信度相似，且更新时间相似，说明是连在一起的
				int current_confidence = oNearPoint.data_n[3] / fConfidenceLevelLength;
				if(current_confidence > 4) current_confidence = 4;
				int current_time_stamp = oNearPoint.data_c[2];
				bool confidence_and_time_connect = current_confidence == 4 && confidence == 4 && abs(current_time_stamp - time_stamp) < time_diff_ref;
				if(confidence_and_time_connect) {
					m_oUnionSet.Union(oPos, oNearPos);
					continue;
				}

				// 法向相似性计算
				Eigen::Vector3f vNearNormal = oNearPoint.getNormalVector3fMap();
				float current_dot_ref = m_pUpdateStrategy->IsSoftDynamic(vVolumeCopy[oNearPos].data_c[1]) ? fStrictDotRef : normal_dot_ref;
				bool normal_dot = vNearNormal.dot(oPoint.getNormalVector3fMap()) > current_dot_ref;
				if(!normal_dot) continue;

				// 相交计算
				oIntersection->Reset(oNearPoint, GetVoxelLength(), oPosPoint.getVector3fMap());
				bool cross_pos = oIntersection->CrossPos(iAxisId);
				if(cross_pos) m_oUnionSet.Union(oPos, oNearPos);
			}
		}
	}
}


/**
 * @brief give a hash voxel pos, find the voxel
 * @param oPos hash voxel pos
 * @return the reference of the voxel
*/
VoxelBase* HashBlock::GetVoxelPtr(const HashPos & oPos) {

	HashPos oBlockPos;
	int iVoxelIndex;
	pcl::PointXYZ oVoxelBase = HashPosTo3DPos(oPos);
	oVoxelBase.getVector3fMap() += m_vVoxelHalfSize;
	PointBelongVoxelIndex(oVoxelBase, oBlockPos, iVoxelIndex);
	if(m_vBlockVolume.count(oBlockPos) && m_vBlockVolume[oBlockPos]->size() > iVoxelIndex)
		return &(m_vBlockVolume[oBlockPos]->at(iVoxelIndex));
	return nullptr;
}


/**
 * @brief update union conflict, when connect judge a voxel dynamic, reduce the weight of voxel
 * @param iRemoveSetSizeRef min size of static voxel connection set
 * @param fRemoveTimeRef keep the just came in voxel, because they are always in a small connection set, whatever it is dynamic or not
*/
void HashBlock::UpdateUnionConflict(const int iRemoveSetSizeRef, const float fRemoveTimeRef) {

	std::shared_lock<std::shared_mutex> union_read_lock(m_mUnionSetLock);
	auto vVoxelSets = m_oUnionSet.GetSets();

	std::unique_lock<std::shared_mutex> volume_write_lock(m_mVolumeLock);
	for(auto && [oPos, oPosList] : vVoxelSets) {
		if(oPosList.size() < iRemoveSetSizeRef && !m_oUnionSet.InMaxSet(oPos)) {
			for(auto && oPosInSet : oPosList) {
				
				VoxelBase* oVoxel = GetVoxelPtr(oPosInSet);
				if(!oVoxel->IsOccupied()) continue;

				float& fTimeStamp = oVoxel->point.data_c[2];
				if(m_iFrameCount - fTimeStamp > fRemoveTimeRef) {
					float& fConflictTime = oVoxel->point.data_c[1];
					bool bToDelete = m_pUpdateStrategy->Conflict(fConflictTime);
				}
			}
		}
	}
}


/**
 * @brief publish the union set for visual and debug
 * @return oOutputUnionSet - output marker msg to show different set of voxels
*/
void HashBlock::DrawUnionSet(visualization_msgs::MarkerArray& oOutputUnionSet) {

	constexpr int remove_set_size_ref = 10;
	std::shared_lock<std::shared_mutex> union_read_lock(m_mUnionSetLock);
	auto vVoxelSets = m_oUnionSet.GetSets();

	srand(6552);
	int index = 10;

	for(auto && [oPos, oPosList] : vVoxelSets) {

		// if(oPosList.size() < remove_set_size_ref && !m_oUnionSet.InMaxSet(oPos)) {
		if(oPosList.size() >= remove_set_size_ref) {

			visualization_msgs::Marker oCurrentSet;
			oCurrentSet.header.frame_id = "map";
			oCurrentSet.header.stamp = ros::Time::now();
			oCurrentSet.type = visualization_msgs::Marker::CUBE_LIST;
			oCurrentSet.action = visualization_msgs::Marker::MODIFY;
			oCurrentSet.id = index++; 

			oCurrentSet.scale.x = GetVoxelLength().x();
			oCurrentSet.scale.y = GetVoxelLength().y();
			oCurrentSet.scale.z = GetVoxelLength().z();

			oCurrentSet.pose.position.x = 0.0;
			oCurrentSet.pose.position.y = 0.0;
			oCurrentSet.pose.position.z = 0.0;

			oCurrentSet.pose.orientation.x = 0.0;
			oCurrentSet.pose.orientation.y = 0.0;
			oCurrentSet.pose.orientation.z = 0.0;
			oCurrentSet.pose.orientation.w = 1.0;

			oCurrentSet.color.a = 1.0;
			oCurrentSet.color.r = random() / (float)RAND_MAX;
			oCurrentSet.color.g = random() / (float)RAND_MAX;
			oCurrentSet.color.b = random() / (float)RAND_MAX;

			for(auto && oPosInSet : oPosList) {

				auto o3DPos = HashPosTo3DPos(oPosInSet);
				
				geometry_msgs::Point point;
				point.x = o3DPos.x + GetVoxelLength().x() / 2;
				point.y = o3DPos.y + GetVoxelLength().y() / 2;
				point.z = o3DPos.z + GetVoxelLength().z() / 2;
				oCurrentSet.points.push_back(point);
			}

			oOutputUnionSet.markers.push_back(oCurrentSet);
		}
	}

	while(index < 1000) {

		visualization_msgs::Marker oCurrentSet;
		oCurrentSet.header.frame_id = "map";
		oCurrentSet.header.stamp = ros::Time::now();
		oCurrentSet.type = visualization_msgs::Marker::CUBE_LIST;
		oCurrentSet.action = visualization_msgs::Marker::MODIFY;
		oCurrentSet.id = index++; 

		oCurrentSet.scale.x = GetVoxelLength().x();
		oCurrentSet.scale.y = GetVoxelLength().y();
		oCurrentSet.scale.z = GetVoxelLength().z();

		oCurrentSet.pose.position.x = 0.0;
		oCurrentSet.pose.position.y = 0.0;
		oCurrentSet.pose.position.z = 0.0;

		oCurrentSet.pose.orientation.x = 0.0;
		oCurrentSet.pose.orientation.y = 0.0;
		oCurrentSet.pose.orientation.z = 0.0;
		oCurrentSet.pose.orientation.w = 1.0;
		
		oCurrentSet.color.a = 0.8;
		
		oOutputUnionSet.markers.push_back(oCurrentSet);
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief output the hash volume status
 * @return debug message, include size/bucket_size/load_factor
*/
std::string HashBlock::PrintVolumeStatus() const{
	std::stringstream ss;
	ss << "Block size is: " << m_vBlockVolume.size()
		<< " | Bucket size is: " << m_vBlockVolume.bucket_count()
		<< " | Load factor is: " << m_vBlockVolume.load_factor();
	return ss.str();
}


void HashBlock::GetBlockCopy(const HashPos& oBlockPos, Block& oBlock) {
	
	std::shared_lock<std::shared_mutex> volume_read_lock(m_mVolumeLock);

	// auto generate new block
	if(m_vBlockVolume[oBlockPos] == nullptr) {
		m_vBlockVolume[oBlockPos] = new Block(m_iVoxelFullNumPerBlock, oBlockPos);
	}
	oBlock.copy(*m_vBlockVolume[oBlockPos]);
}

void HashBlock::UpdateConflictResult(Block& oBlock, const bool bKeepVoxel) {

	std::unique_lock<std::shared_mutex> volume_write_lock(m_mVolumeLock);

	for(int i = 0; i < oBlock.size(); ++i) {

		const VoxelBase& oVoxel = oBlock.at(i);

		const float& fDeconfidence = oVoxel.point.data_n[3];
		if(fDeconfidence >= 0) continue;

		VoxelBase& oCurrentVoxel = m_vBlockVolume[oBlock.pos]->at(i);
		oCurrentVoxel.PushQueue(VoxelBase::Free);
		if(oCurrentVoxel.IsUnknown()) {
			oCurrentVoxel.SetFreeSpace();
			continue;
		}

		if(oCurrentVoxel.IsFreeSpace()) continue;

		float& fConflictTime = oCurrentVoxel.point.data_c[1];
		bool bToDelete = m_pUpdateStrategy->Conflict(fConflictTime);

		if(bToDelete && !bKeepVoxel) {
			oCurrentVoxel.SetFreeSpace();
		} 
		else {
			float& fConfidence = oCurrentVoxel.point.data_n[3];
			fConfidence -= fDeconfidence;
			if(fConfidence < 0) fConfidence = 0;
		}
	}
}