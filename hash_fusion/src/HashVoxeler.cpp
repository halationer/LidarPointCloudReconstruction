#include"HashVoxeler.h"

#include "Fusion.h"

bool operator == (const HashPos & a, const HashPos & b) {
	return a.x == b.x && a.y == b.y && a.z == b.z;
}

std::ostream & operator << (std::ostream & out, const HashPos & pos) {
	out << "(" << pos.x << "," << pos.y << "," << pos.z << ")";
	return out;
}

HashVoxeler::HashVoxeler() :
					m_fMinX(FLT_MAX),m_fMinY(FLT_MAX),m_fMinZ(FLT_MAX),
                    m_fMaxX(-FLT_MAX),m_fMaxY(-FLT_MAX),m_fMaxZ(-FLT_MAX),
					m_fDefault(0.0), m_fExpandNum(1.0),
					m_pCornerCloud(new pcl::PointCloud<pcl::PointXYZ>),
					m_pVoxelNormals(new pcl::PointCloud<pcl::PointNormal>){}

HashVoxeler::~HashVoxeler(){}


void HashVoxeler::GetVolume(HashVoxeler::HashVolume & vVolumeCopy) const {
	
	std::shared_lock<std::shared_mutex> lock(m_mVolumeLock);
	vVolumeCopy = m_vVolume;
}

/*=======================================
GetResolution
Input: oLength - length of each dim of voxel (m)
Output: m_oVoxelLength - member of the class, record the voxel length info
Function: Set the resolution of the voxel in 3 dims
========================================*/
void HashVoxeler::GetResolution(pcl::PointXYZ & oLength){

	m_oVoxelLength.x = oLength.x;
	m_oVoxelLength.y = oLength.y;
	m_oVoxelLength.z = oLength.z;
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
		m_vVolume[oPos] = oFusedPoint;
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

void HashVoxeler::HashPosTo3DPos(const HashPos & oCornerPos, const pcl::PointXYZ & oVoxelSize, Eigen::Vector3f & oCorner3DPos) {
	
	oCorner3DPos.x() = oCornerPos.x * oVoxelSize.x;
	oCorner3DPos.y() = oCornerPos.y * oVoxelSize.y;
	oCorner3DPos.z() = oCornerPos.z * oVoxelSize.z;
}

pcl::PointXYZ HashVoxeler::HashPosTo3DPos(const HashPos & oPos) {

	return pcl::PointXYZ(oPos.x * m_oVoxelLength.x, oPos.y * m_oVoxelLength.y, oPos.z * m_oVoxelLength.z);
}