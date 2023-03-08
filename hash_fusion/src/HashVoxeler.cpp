#include"HashVoxeler.h"

#include "Fusion.h"

bool operator == (const HashPos & a, const HashPos & b) {
	return a.x == b.x && a.y == b.y && a.z == b.z;
}

std::ostream & operator << (std::ostream & out, const HashPos & pos) {
	out << "(" << pos.x << "," << pos.y << "," << pos.z << ")";
	return out;
}

HashVoxeler::HashVoxeler() : m_iFrameCount(0) {}

HashVoxeler::~HashVoxeler() {}


void HashVoxeler::GetVolume(HashVoxeler::HashVolume & vVolumeCopy) const {
	
	std::shared_lock<std::shared_mutex> lock(m_mVolumeLock);
	vVolumeCopy = m_vVolume;
}

void HashVoxeler::GetVolumeCloud(pcl::PointCloud<pcl::PointNormal> & vVolumeCloud) const {

	vVolumeCloud.clear();
	std::shared_lock<std::shared_mutex> lock(m_mVolumeLock);
	for(auto && [_,oVoxelPoint] : m_vVolume)
		vVolumeCloud.push_back(oVoxelPoint);
}


void HashVoxeler::GetRecentVolume(HashVoxeler::HashVolume & vVolumeCopy, const int iRecentTime) const {

	std::shared_lock<std::shared_mutex> lock(m_mVolumeLock);
	vVolumeCopy.clear();
	for(auto && [oPos, oVoxel] : m_vVolume)
		if(m_iFrameCount - oVoxel.data_c[2] <= iRecentTime && m_pUpdateStrategy->IsStatic(oVoxel.data_c[1]))
			vVolumeCopy[oPos] = oVoxel;
}

/*=======================================
GetResolution
Input: oLength - length of each dim of voxel (m)
Output: m_oVoxelLength - member of the class, record the voxel length info
Function: Set the resolution of the voxel in 3 dims
========================================*/
void HashVoxeler::SetResolution(pcl::PointXYZ & oLength){

	m_oVoxelLength.x = oLength.x;
	m_oVoxelLength.y = oLength.y;
	m_oVoxelLength.z = oLength.z;
}

void HashVoxeler::SetStrategy(enum vus eStrategy) {

	m_pUpdateStrategy = std::shared_ptr<VolumeUpdateStrategy>(CreateStrategy(eStrategy));	
}


/*=======================================
PointBelongVoxelPos
Input: oPoint - the input point to be judged
Output: oPos - which voxel pos does the point belong
Function: calculate the voxel pos the point belongs
========================================*/
template<class PointType>
void HashVoxeler::PointBelongVoxelPos(const PointType & oPoint, HashPos & oPos) {
	
	oPos.x = floor(oPoint.x / m_oVoxelLength.x);
	oPos.y = floor(oPoint.y / m_oVoxelLength.y);
	oPos.z = floor(oPoint.z / m_oVoxelLength.z);
}
template void HashVoxeler::PointBelongVoxelPos(const pcl::PointXYZ & oPoint, HashPos & oPos);
template void HashVoxeler::PointBelongVoxelPos(const pcl::PointNormal & oPoint, HashPos & oPos);


/*=======================================
VoxelizePoints
Input: vCloud - the input point cloud
Output: m_vVolume - the volume result of all points
Function: put the points into their voxels and fusion
========================================*/
void HashVoxeler::VoxelizePointsAndFusion(pcl::PointCloud<pcl::PointNormal> & vCloud) {
	
	std::unique_lock<std::shared_mutex> lock(m_mVolumeLock);
	++m_iFrameCount;

	// put points into voxels
    std::unordered_map<HashPos, std::vector<int>, HashFunc> vPointContainer;
	HashPos oCurrentPos;
	for(int i = 0; i < vCloud.size(); ++i) {
		const pcl::PointNormal & oCurrentPoint = vCloud.at(i);
		PointBelongVoxelPos(oCurrentPoint, oCurrentPos);
		vPointContainer[oCurrentPos].emplace_back(i);
	}

	// fusion the points
	Fusion oVoxelFusion;
	for(auto&& [oPos, IndexList] : vPointContainer) {
		pcl::PointNormal oBasePoint;
		if(m_vVolume.count(oPos)) oBasePoint = m_vVolume[oPos];
		pcl::PointNormal oFusedPoint = oVoxelFusion.NormalFusionWeighted(IndexList, vCloud, oBasePoint);
		float& fConflictTime = oFusedPoint.data_c[1];
		m_pUpdateStrategy->Support(fConflictTime);
		float& fTimeStamp = oFusedPoint.data_c[2];
		fTimeStamp = m_iFrameCount;
		m_vVolume[oPos] = oFusedPoint;
	}
}


/*=======================================
UpdateConflictResult
Input: vVolumeCloud - the conflict result that calculated by the 'surfel fusion' liked method
Output: m_vVolume - update the volume
Function: decrease the confidence of dynamic point and record conflict
========================================*/
void HashVoxeler::UpdateConflictResult(const pcl::PointCloud<pcl::PointNormal> & vVolumeCloud) {

	std::unique_lock<std::shared_mutex> lock(m_mVolumeLock);

	HashPos oPos;
	for(auto && oVoxelPoint : vVolumeCloud) {

		PointBelongVoxelPos(oVoxelPoint, oPos);
		const float& fDeconfidence = oVoxelPoint.data_n[3];

		if(fDeconfidence < 0 && m_vVolume.count(oPos)) {

			float& fConflictTime = m_vVolume[oPos].data_c[1];
			bool bToDelete = m_pUpdateStrategy->Conflict(fConflictTime);

			if(bToDelete) {
				m_vVolume.erase(oPos);
			} 
			else {
				float& fConfidence = m_vVolume[oPos].data_n[3];
				fConfidence -= fDeconfidence;
				if(fConfidence < 0) fConfidence = 0;
			}
		}
	}
}


/*=======================================
CornerIdxs
Input: oVoxel - a given voxel hash-pos
Output: vCornerPoses - eight corner hash-pos of this voxel
Function: return a voxel corner hash-posed
========================================*/
void HashVoxeler::GetCornerPoses(const HashPos & oVoxel, std::vector<HashPos> & vCornerPoses){

	vCornerPoses.clear();

	// vCornerPoses.emplace_back(oVoxel.x, 		oVoxel.y, 		oVoxel.z	);
	// vCornerPoses.emplace_back(oVoxel.x + 1, 	oVoxel.y, 		oVoxel.z	);
	// vCornerPoses.emplace_back(oVoxel.x, 		oVoxel.y + 1, 	oVoxel.z	);
	// vCornerPoses.emplace_back(oVoxel.x, 		oVoxel.y, 		oVoxel.z + 1);
	// vCornerPoses.emplace_back(oVoxel.x + 1, 	oVoxel.y, 		oVoxel.z + 1);
	// vCornerPoses.emplace_back(oVoxel.x + 1, 	oVoxel.y + 1, 	oVoxel.z	);
	// vCornerPoses.emplace_back(oVoxel.x, 		oVoxel.y + 1,	oVoxel.z + 1);
	// vCornerPoses.emplace_back(oVoxel.x + 1, 	oVoxel.y + 1, 	oVoxel.z + 1);

	vCornerPoses.emplace_back(oVoxel.x, 		oVoxel.y, 		oVoxel.z	);
	vCornerPoses.emplace_back(oVoxel.x, 		oVoxel.y + 1,	oVoxel.z	);
	vCornerPoses.emplace_back(oVoxel.x + 1, 	oVoxel.y + 1, 	oVoxel.z	);
	vCornerPoses.emplace_back(oVoxel.x + 1,		oVoxel.y, 		oVoxel.z	);
	vCornerPoses.emplace_back(oVoxel.x, 		oVoxel.y, 		oVoxel.z + 1);
	vCornerPoses.emplace_back(oVoxel.x, 		oVoxel.y + 1, 	oVoxel.z + 1);
	vCornerPoses.emplace_back(oVoxel.x + 1,		oVoxel.y + 1,	oVoxel.z + 1);
	vCornerPoses.emplace_back(oVoxel.x + 1, 	oVoxel.y,	 	oVoxel.z + 1);
}

void HashVoxeler::HashPosTo3DPos(const HashPos & oCornerPos, const pcl::PointXYZ & oVoxelLength, Eigen::Vector3f & oCorner3DPos) {
	
	oCorner3DPos.x() = oCornerPos.x * oVoxelLength.x;
	oCorner3DPos.y() = oCornerPos.y * oVoxelLength.y;
	oCorner3DPos.z() = oCornerPos.z * oVoxelLength.z;
}

pcl::PointXYZ HashVoxeler::HashPosTo3DPos(const HashPos & oPos) {

	return pcl::PointXYZ(oPos.x * m_oVoxelLength.x, oPos.y * m_oVoxelLength.y, oPos.z * m_oVoxelLength.z);
}