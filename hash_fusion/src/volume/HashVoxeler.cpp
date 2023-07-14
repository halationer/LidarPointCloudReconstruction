#include "HashVoxeler.h"
#include "Fusion.h"
#include "SignedDistance.h"


HashVoxeler::HashVoxeler() : 
	m_iFrameCount(0), m_iMaxRecentKeep(500), m_fLidarSpeed(0), m_fRadiusExpandFactor(1.25f), m_iRecentTimeToGetRadius(30) {}

HashVoxeler::~HashVoxeler() {}

/*=======================================
GetRadiusExpand
Function: Get m_fRadiusExpandFactor
========================================*/
float HashVoxeler::GetRadiusExpand() const {
	
	std::shared_lock<std::shared_mutex> center_read_lock(m_mCenterLock);
	return m_fRadiusExpandFactor;
}

/*=======================================
UpdateCenter
Input: 	oLidarCenter - current lidar center
Output: m_fLidarSpeed - lidar distance between this time and last time
		m_fRadiusExpandFactor - expand the radius of union set building
		m_oLidarCenter - current lidar center
Function: update vehicle center and related variables
========================================*/
void HashVoxeler::UpdateLidarCenter(Eigen::Vector3f& oLidarCenter) {

	constexpr float speed_trans_factor = 0.4f;

	// std::cout << "\n" << m_fLidarSpeed << " is the speed; ";
	// std::cout << "and the factor is: " << m_fRadiusExpandFactor << std::endl;

	std::unique_lock<std::shared_mutex> center_write_lock(m_mCenterLock);
	m_fLidarSpeed = (m_oLidarCenter - oLidarCenter).norm(); 
	m_fRadiusExpandFactor = 3.25f - std::clamp(m_fLidarSpeed * speed_trans_factor, 1.25f, 2.0f);
	m_oLidarCenter = oLidarCenter;
}


/*=======================================
GetAllVolume
Input: 	m_vVolume - the whole volume
Output: vVolumeCopy - the output volume result
Function: get the whole volume
========================================*/
void HashVoxeler::GetAllVolume(HashVoxeler::HashVolume & vVolumeCopy) const {

	vVolumeCopy.clear();
	std::shared_lock<std::shared_mutex> volume_read_lock(m_mVolumeLock);
	vVolumeCopy = m_vVolume;
}


/*=======================================
GetStaticVolume
Input: 	m_vVolume - the whole volume
Output: vVolumeCopy - the output volume result
Function: get the whole volume without dynamic voxels
========================================*/
void HashVoxeler::GetStaticVolume(HashVoxeler::HashVolume & vVolumeCopy) const {

	vVolumeCopy.clear();
	std::shared_lock<std::shared_mutex> volume_read_lock(m_mVolumeLock);
	for(auto && [oPos, oVoxel] : m_vVolume)
		if(m_pUpdateStrategy->IsStatic(oVoxel.data_c[1]))
			vVolumeCopy[oPos] = oVoxel;
}


/*=======================================
GetStrictStaticVolume
Input: 	m_vVolume - the whole volume
Output: vVolumeCopy - the output volume result
Function: get the whole volume without probably dynamic voxels
========================================*/
void HashVoxeler::GetStrictStaticVolume(HashVoxeler::HashVolume & vVolumeCopy) const {

	vVolumeCopy.clear();
	std::shared_lock<std::shared_mutex> volume_read_lock(m_mVolumeLock);
	for(auto && [oPos, oVoxel] : m_vVolume)
		if(!m_pUpdateStrategy->IsSoftDynamic(oVoxel.data_c[1]))
			vVolumeCopy[oPos] = oVoxel;
}


/*=======================================
GetVolumeCloud
Input: 	m_vVolume - the whole surfel volume
Output: vVolumeCloud - the output surfel cloud
Function: transfer and get a surfel cloud of the volume
========================================*/
void HashVoxeler::GetVolumeCloud(pcl::PointCloud<pcl::PointNormal> & vVolumeCloud) const {

	vVolumeCloud.clear();
	std::shared_lock<std::shared_mutex> volume_read_lock(m_mVolumeLock);
	for(auto && [_,oVoxelPoint] : m_vVolume)
		vVolumeCloud.push_back(oVoxelPoint);
}


/*=======================================
GetRecentVolume
Input: 	iRecentTime - recent time
		m_vRecentVolume - surfel volume of max recent time
Output: vVolumeCopy - the output volume result
Function: get the volume of recent time (only static voxels)
========================================*/
void HashVoxeler::GetRecentVolume(HashVoxeler::HashVolume & vVolumeCopy, const int iRecentTime) const {

	vVolumeCopy.clear();
	std::shared_lock<std::shared_mutex> volume_read_lock(m_mVolumeLock);
	for(auto && [oPos, oVoxel] : m_vRecentVolume)
		if(m_iFrameCount - oVoxel.data_c[2] <= (uint32_t)iRecentTime && m_pUpdateStrategy->IsStatic(oVoxel.data_c[1]))
			vVolumeCopy[oPos] = oVoxel;
}


/*=======================================
GetRecentAllVolume
Input: 	iRecentTime - recent time
		m_vRecentVolume - surfel volume of max recent time
Output: vVolumeCopy - the output volume result
Function: get the volume of recent time
========================================*/
void HashVoxeler::GetRecentAllVolume(HashVoxeler::HashVolume & vVolumeCopy, const int iRecentTime) const {

	vVolumeCopy.clear();
	std::shared_lock<std::shared_mutex> volume_read_lock(m_mVolumeLock);
	for(auto && [oPos, oVoxel] : m_vRecentVolume)
		if(m_iFrameCount - oVoxel.data_c[2] <= (uint32_t)iRecentTime)
			vVolumeCopy[oPos] = oVoxel;
}

/*=======================================
GetAllConnectVolume - Only for small scene or final output
Input: 	iConnectMinSize - the volume copy will get voxels in the union set which size is larger than this number
		m_vRecentVolume - surfel volume of max recent time
		m_oUnionSet - the connectivity record union-find set
Output: vVolumeCopy - the output volume result
Function: get the voxels in the largest set of m_oUnionSet
========================================*/
void HashVoxeler::GetAllConnectVolume(HashVoxeler::HashVolume & vVolumeCopy, const int iConnectMinSize) {

	vVolumeCopy.clear();
	std::shared_lock<std::shared_mutex> union_read_lock(m_mUnionSetLock);
	std::shared_lock<std::shared_mutex> volume_read_lock(m_mVolumeLock);
	for(auto && [oPos, oVoxel] : m_vRecentVolume)
		if(m_pUpdateStrategy->IsStatic(oVoxel.data_c[1]) && m_oUnionSet.GetSetSize(oPos) > iConnectMinSize)
				vVolumeCopy[oPos] = oVoxel;
}

/*=======================================
GetRecentConnectVolume
Input: 	iRecentTime - recent time
		iConnectMinSize - the volume copy will get voxels in the union set which size is larger than this number
		m_vRecentVolume - surfel volume of max recent time
		m_oUnionSet - the connectivity record union-find set
Output: vVolumeCopy - the output volume result
Function: get the volume of recent time and its voxels are in the largest set of m_oUnionSet
========================================*/
void HashVoxeler::GetRecentConnectVolume(HashVoxeler::HashVolume & vVolumeCopy, const int iRecentTime, const int iConnectMinSize) {

	vVolumeCopy.clear();
	std::shared_lock<std::shared_mutex> union_read_lock(m_mUnionSetLock);
	std::shared_lock<std::shared_mutex> volume_read_lock(m_mVolumeLock);
	for(auto && [oPos, oVoxel] : m_vRecentVolume)
		if(m_iFrameCount - oVoxel.data_c[2] <= (uint32_t)iRecentTime 
			&& m_pUpdateStrategy->IsStatic(oVoxel.data_c[1]) 
			&& m_oUnionSet.GetSetSize(oPos) > iConnectMinSize)
				vVolumeCopy[oPos] = oVoxel;
}


/*=======================================
GetRecentMaxConnectVolume
Input: 	iRecentTime - recent time
		m_vRecentVolume - surfel volume of max recent time
		m_oUnionSet - the connectivity record union-find set
Output: vVolumeCopy - the output volume result
Function: get the volume of recent time and its voxels are in the largest set of m_oUnionSet - should rebuild the union set before call this func 
========================================*/
void HashVoxeler::GetRecentMaxConnectVolume(HashVoxeler::HashVolume & vVolumeCopy, const int iRecentTime) {

	vVolumeCopy.clear();
	std::shared_lock<std::shared_mutex> union_read_lock(m_mUnionSetLock);
	std::shared_lock<std::shared_mutex> volume_read_lock(m_mVolumeLock);
	for(auto && [oPos, oVoxel] : m_vRecentVolume)
		if(m_iFrameCount - oVoxel.data_c[2] <= (uint32_t)iRecentTime && m_pUpdateStrategy->IsStatic(oVoxel.data_c[1]) && m_oUnionSet.InMaxSet(oPos))
			vVolumeCopy[oPos] = oVoxel;
	// GetRecentVolume(vVolumeCopy, iRecentTime); 
}


/*=======================================
GetLocalVolume
Input: 	vCenter - the center point of an area
		fRadius - the radius of an area
		m_vVolume - the whole surfel volume
Output: vVolumeCopy - the output volume result
Function: get the voxels in the area
========================================*/
void HashVoxeler::GetLocalVolume(HashVoxeler::HashVolume & vVolumeCopy, const Eigen::Vector3f vCenter, const float fRadius) const {

	vVolumeCopy.clear();
	std::shared_lock<std::shared_mutex> volume_read_lock(m_mVolumeLock);
	for(auto && [oPos, oVoxel] : m_vVolume) {
		Eigen::Vector3f vCurrent(oPos.x, oPos.y, oPos.z);
		if((vCurrent - vCenter).norm() <= fRadius && m_pUpdateStrategy->IsStatic(oVoxel.data_c[1]))
			vVolumeCopy[oPos] = oVoxel;
	}
}


/*=======================================
GetLocalConnectVolume
Input: 	vCenter - the center point of an area
		fRadius - the radius of an area
		m_vVolume - the whole surfel volume
Output: vVolumeCopy - the output volume result
Function: get the voxels in the area
========================================*/
void HashVoxeler::GetLocalConnectVolume(HashVoxeler::HashVolume & vVolumeCopy, const Eigen::Vector3f vCenter, const float fRadius, const int iConnectMinSize) {

	vVolumeCopy.clear();
	std::shared_lock<std::shared_mutex> volume_read_lock(m_mVolumeLock);
	for(auto && [oPos, oVoxel] : m_vVolume) {
		Eigen::Vector3f vCurrent(oPos.x, oPos.y, oPos.z);
		if((vCurrent - vCenter).norm() <= fRadius && m_pUpdateStrategy->IsStatic(oVoxel.data_c[1]) && m_oUnionSet.GetSetSize(oPos) > iConnectMinSize)
			vVolumeCopy[oPos] = oVoxel;
	}
}


/*=======================================
IsNoneFlow
Input: 	oPos - the pos of a voxel
Output: bool - whether the voxel is flow static
Function: return the flow status of the voxel
========================================*/
bool HashVoxeler::IsNoneFlow(const HashPos& oPos) {

	return m_vVolume[oPos].data_c[0] < 0.3;
}


/*=======================================
FlowScore
Input: 	oPos - the pos of a voxel
Output: bool - whether the voxel is flow static
Function: return the flow status of the voxel
========================================*/
float HashVoxeler::FlowScore(const HashPos& oPos) {

	if(m_vPointFlow.count(oPos) == 0) return 0.3;
	pcl::PointNormal& oPoint = m_vVolume[oPos];
	pcl::PointNormal& oPointFlow = m_vPointFlow[oPos];
	Eigen::Vector3f vTranslate(oPointFlow.x, oPointFlow.y, oPointFlow.z);
	Eigen::Vector3f vNormal(oPoint.normal_x, oPoint.normal_y, oPoint.normal_z);
	return abs(vTranslate.dot(vNormal));
}


/*=======================================
Clear
Input: 	m_vVolume - voxel set of reconstruction
Function: remove all flowing voxels 
========================================*/
void HashVoxeler::ClearFlow() {
	
	std::unique_lock<std::shared_mutex> volume_write_lock(m_mVolumeLock);
	vector<HashPos> vToDeletePos;
	for(auto && [oPos,_] : m_vRecentVolume)
		if(!IsNoneFlow(oPos)) vToDeletePos.push_back(oPos);

	for(auto && oPos : vToDeletePos) {
		m_vRecentVolume.erase(oPos);
		m_vVolume.erase(oPos);
	}
}


/*=======================================
GetRecentNoneFlowVolume
Input: 	iRecentTime - recent time
		m_vRecentVolume - surfel volume of max recent time
Output: vVolumeCopy - the output volume result
Function: get the volume of recent time and its voxels is flow static 
========================================*/
void HashVoxeler::GetRecentNoneFlowVolume(HashVoxeler::HashVolume & vVolumeCopy, const int iRecentTime) {
	
	std::shared_lock<std::shared_mutex> volume_read_lock(m_mVolumeLock);
	vVolumeCopy.clear();
	for(auto && [oPos, oVoxel] : m_vRecentVolume)
		if(m_iFrameCount - oVoxel.data_c[2] <= (uint32_t)iRecentTime && m_pUpdateStrategy->IsStatic(oVoxel.data_c[1]) && IsNoneFlow(oPos))
			vVolumeCopy[oPos] = oVoxel;
}


/*=======================================
GetRecentHighDistributionVolume
Input: 	iRecentTime - recent time
		m_vRecentVolume - surfel volume of max recent time
Output: vVolumeCopy - the output volume result
Function: get the volume of recent time and its voxels contains points with different normals (not similar normals) 
========================================*/
void HashVoxeler::GetRecentHighDistributionVolume(HashVoxeler::HashVolume & vVolumeCopy, const int iRecentTime) {

	std::shared_lock<std::shared_mutex> volume_read_lock(m_mVolumeLock);
	vVolumeCopy.clear();
	for(auto && [oPos, oVoxel] : m_vRecentVolume)
		if(m_iFrameCount - oVoxel.data_c[2] <= (uint32_t)iRecentTime && m_pUpdateStrategy->IsStatic(oVoxel.data_c[1]) && oVoxel.data_c[3] >= m_fExpandDistributionRef)
			vVolumeCopy[oPos] = oVoxel;
}


/*=======================================
RebuildUnionSetAll - only for small scene or final output
Input: m_vVolume - the surfel volume
	   fStrictDotRef - the dot value to judge the normal similarity - when the voxel is pass throughed last time
	   fSoftDotRef - the dot value to judge the normal similarity - when the voxel is static
	   fConfidenceLevelLength - the confidence have four level, let cll = this param, then 0, 1*cll, 2*cll, 3*cll, 4*cll
Output: m_oUnionSet - use recent volume to build union set
Function: rebuild union set according to recent surfel volume
========================================*/
void HashVoxeler::RebuildUnionSetAll(const float fStrictDotRef, const float fSoftDotRef, const float fConfidenceLevelLength) {

	HashVoxeler::HashVolume vVolumeCopy;
	GetStaticVolume(vVolumeCopy);
	RebuildUnionSetCore(vVolumeCopy, fStrictDotRef, fSoftDotRef, fConfidenceLevelLength);
}


/*=======================================
RebuildUnionSet
Input: m_vRecentVolume - the surfel volume of max recent time
	   fStrictDotRef - the dot value to judge the normal similarity - when the voxel is pass throughed last time
	   fSoftDotRef - the dot value to judge the normal similarity - when the voxel is static
	   fConfidenceLevelLength - the confidence have four level, let cll = this param, then 0, 1*cll, 2*cll, 3*cll, 4*cll
Output: m_oUnionSet - use recent volume to build union set
Function: rebuild union set according to recent surfel volume
========================================*/
void HashVoxeler::RebuildUnionSet(const float fStrictDotRef, const float fSoftDotRef, const float fConfidenceLevelLength) {

	// a. N1=N2             - 连通
	// b. 两个高置信 & t1=t2 - 连通
	// c. N1!=N2 & t1!=t2   - 不连通
	// d. 置信度不同 & N1!=N2 - 不连通
	// e. 两个低置信 & N1!=N2 - 不连通

	// copy avoid of block
	HashVoxeler::HashVolume vVolumeCopy;
	GetRecentVolume(vVolumeCopy, m_iRecentTimeToGetRadius);
	
	// XXX: get center and radius
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
	fRadius *= GetRadiusExpand();
	GetLocalVolume(vVolumeCopy, vCenter, fRadius);

	RebuildUnionSetCore(vVolumeCopy, fStrictDotRef, fSoftDotRef, fConfidenceLevelLength);
}


/*=======================================
RebuildUnionSetCore
Input: vVolumeCopy - the surfel volume of max recent time
	   fStrictDotRef - the dot value to judge the normal similarity - when the voxel is pass throughed last time
	   fSoftDotRef - the dot value to judge the normal similarity - when the voxel is static
	   fConfidenceLevelLength - the confidence have four level, let cll = this param, then 0, 1*cll, 2*cll, 3*cll, 4*cll
Output: m_oUnionSet - use recent volume to build union set
Function: rebuild union set according to recent surfel volume
========================================*/
void HashVoxeler::RebuildUnionSetCore(HashVoxeler::HashVolume & vVolumeCopy, const float fStrictDotRef, const float fSoftDotRef, const float fConfidenceLevelLength) {

	std::unique_lock<std::shared_mutex> union_write_lock(m_mUnionSetLock);
	m_oUnionSet.Clear();

	// SignedDistance oSdf(iRecentTime);
	// auto vSdf = oSdf.NormalBasedGlance(*this);

	// constexpr int xyz_order[][4] = {{0, 1, 4, 5}, {0, 3, 4, 7}, {0, 1, 2, 3}};
	// constexpr int xyz_delta[][3] = {{-1, 0, 0}, {0, -1, 0}, {0, 0, -1}};

	for(auto && [oPos, oPoint] : vVolumeCopy) {
		/* 隐式场连通性
		std::vector<HashPos> vCornerPoses;
		GetCornerPoses(oPos, vCornerPoses);
		for(int dim = 0; dim < 3; ++dim) {
			for(int i = 0; i < 3; ++i) {
				if(vSdf[vCornerPoses[xyz_order[dim][i]]] * vSdf[vCornerPoses[xyz_order[dim][i+1]]] < 0) {
					HashPos oNearPos(oPos.x + xyz_delta[dim][0], oPos.y + xyz_delta[dim][1], oPos.z + xyz_delta[dim][2]);
					if(vVolumeCopy.count(oNearPos)) m_oUnionSet.Union(oPos, oNearPos);
					break;
				}
			}	
		}
		//*/

		/* 法向相似性
		Eigen::Vector3f vNormal(oPoint.normal_x, oPoint.normal_y, oPoint.normal_z);
		for(int dx = -1; dx <= 0; ++dx) {
			for(int dy = -1; dy <= 0; ++dy) {
				for(int dz = -1; dz <= 0; ++dz) {
					HashPos oNearPos(oPos.x + dx, oPos.y + dy, oPos.z + dz);
					if(vVolumeCopy.count(oNearPos) == 0) continue;
					Eigen::Vector3f vNearNormal(vVolumeCopy[oNearPos].normal_x, vVolumeCopy[oNearPos].normal_y, vVolumeCopy[oNearPos].normal_z);
					if(vNormal.dot(vNearNormal) > 0.9f) m_oUnionSet.Union(oPos, oNearPos);
				}
			}
		}
		//*/

		// 置信度分级 0 - 4
		int confidence = oPoint.data_n[3] / fConfidenceLevelLength;
		if(confidence > 4) confidence = 4; // 4 is the max conf level
		// 时间戳
		int time_stamp = oPoint.data_c[2];
		constexpr int time_diff_ref = 30;
		
		// /* 面片连通性 Ax + By + Cz + D = 0 (+法向限制）
		float normal_dot_ref = m_pUpdateStrategy->IsSoftDynamic(oPoint.data_c[1]) ? fStrictDotRef : fSoftDotRef;
		Eigen::Vector3f vNormal(oPoint.normal_x, oPoint.normal_y, oPoint.normal_z);
		float A = oPoint.normal_x, B = oPoint.normal_y, C = oPoint.normal_z;
		float neg_D = A * oPoint.x + B * oPoint.y + C * oPoint.z;
		pcl::PointXYZ oPosPoint = HashPosTo3DPos(oPos);
		int cross_x0 = A ? (neg_D - B * oPosPoint.y - C * oPosPoint.z) / A - oPosPoint.x : -1;
		int cross_y0 = B ? (neg_D - A * oPosPoint.x - C * oPosPoint.z) / B - oPosPoint.y : -1;
		int cross_z0 = C ? (neg_D - A * oPosPoint.x - B * oPosPoint.y) / C - oPosPoint.z : -1;
		int cross_x1 = A ? (neg_D - B * (oPosPoint.y + m_oVoxelLength.y) - C * oPosPoint.z) / A - oPosPoint.x : -1;
		int cross_y1 = B ? (neg_D - A * oPosPoint.x - C * (oPosPoint.z + m_oVoxelLength.z)) / B - oPosPoint.y : -1;
		int cross_z1 = C ? (neg_D - A * (oPosPoint.x + m_oVoxelLength.x) - B * oPosPoint.y) / C - oPosPoint.z : -1;
		cross_x0 /= m_oVoxelLength.x;
		cross_y0 /= m_oVoxelLength.y;
		cross_z0 /= m_oVoxelLength.z;
		cross_x1 /= m_oVoxelLength.x;
		cross_y1 /= m_oVoxelLength.y;
		cross_z1 /= m_oVoxelLength.z;
		#define CROSS_VALID(x) ((x)>=0 && (x)<=1)
		bool cross_neg_dx = CROSS_VALID(cross_y0) || CROSS_VALID(cross_z0) || CROSS_VALID(cross_y1) || confidence == 4;
		bool cross_neg_dy = CROSS_VALID(cross_x0) || CROSS_VALID(cross_z0) || CROSS_VALID(cross_z1) || confidence == 4;
		bool cross_neg_dz = CROSS_VALID(cross_x0) || CROSS_VALID(cross_y0) || CROSS_VALID(cross_x1) || confidence == 4;
		if(cross_neg_dx) {
			HashPos oNearPos(oPos.x - 1, oPos.y, oPos.z);
			if(vVolumeCopy.count(oNearPos)) {
				pcl::PointNormal& oNearPoint = vVolumeCopy[oNearPos];

				int current_confidence = oNearPoint.data_n[3] / fConfidenceLevelLength;
				if(current_confidence > 4) current_confidence = 4;
				int current_time_stamp = oNearPoint.data_c[2];
				// 物体若置信度相似，且更新时间相似，说明是连在一起的
				bool confidence_connect = current_confidence == 4 && confidence == 4 && abs(current_time_stamp - time_stamp) < time_diff_ref;

				float A = oNearPoint.normal_x, B = oNearPoint.normal_y, C = oNearPoint.normal_z;
				float neg_D = A * oNearPoint.x + B * oNearPoint.y + C * oNearPoint.z;
				pcl::PointXYZ oPosPoint = HashPosTo3DPos(oNearPos);
				int cross_y3 = B ? (neg_D - A * (oPosPoint.x + m_oVoxelLength.x) - C * oPosPoint.z) / B - oPosPoint.y : -1;
				int cross_y2 = B ? (neg_D - A * (oPosPoint.x + m_oVoxelLength.x) - C * (oPosPoint.z + m_oVoxelLength.z)) / B - oPosPoint.y : -1;
				int cross_z1 = C ? (neg_D - A * (oPosPoint.x + m_oVoxelLength.x) - B * oPosPoint.y) / C - oPosPoint.z : -1;
				cross_y3 /= m_oVoxelLength.y;
				cross_y2 /= m_oVoxelLength.y;
				cross_z1 /= m_oVoxelLength.z;
				bool cross_pos_dx = CROSS_VALID(cross_y3) || CROSS_VALID(cross_y2) || CROSS_VALID(cross_z1);
				Eigen::Vector3f vNearNormal(vVolumeCopy[oNearPos].normal_x, vVolumeCopy[oNearPos].normal_y, vVolumeCopy[oNearPos].normal_z);
				float current_dot_ref = m_pUpdateStrategy->IsSoftDynamic(vVolumeCopy[oNearPos].data_c[1]) ? fStrictDotRef : normal_dot_ref;
				bool normal_dot = vNearNormal.dot(vNormal) > current_dot_ref;

				if(cross_pos_dx && normal_dot || confidence_connect) m_oUnionSet.Union(oPos, oNearPos);
			}
		}
		if(cross_neg_dy) {
			HashPos oNearPos(oPos.x, oPos.y - 1, oPos.z);
			if(vVolumeCopy.count(oNearPos)) {
				pcl::PointNormal& oNearPoint = vVolumeCopy[oNearPos];

				int current_confidence = oNearPoint.data_n[3] / fConfidenceLevelLength;
				if(current_confidence > 4) current_confidence = 4;
				int current_time_stamp = oNearPoint.data_c[2];
				bool confidence_connect = current_confidence == 4 && confidence == 4 && abs(current_time_stamp - time_stamp) < time_diff_ref;

				float A = oNearPoint.normal_x, B = oNearPoint.normal_y, C = oNearPoint.normal_z;
				float neg_D = A * oNearPoint.x + B * oNearPoint.y + C * oNearPoint.z;
				pcl::PointXYZ oPosPoint = HashPosTo3DPos(oNearPos);
				int cross_z3 = C ? (neg_D - A * oPosPoint.x - B * (oPosPoint.y + m_oVoxelLength.y)) / C - oPosPoint.z : -1;
				int cross_z2 = C ? (neg_D - A * (oPosPoint.x + m_oVoxelLength.x) - B * (oPosPoint.y + m_oVoxelLength.y)) / C - oPosPoint.z : -1;
				int cross_x1 = A ? (neg_D - B * (oPosPoint.y + m_oVoxelLength.y) - C * oPosPoint.z) / A - oPosPoint.x : -1;
				cross_z3 /= m_oVoxelLength.z;
				cross_z2 /= m_oVoxelLength.z;
				cross_x1 /= m_oVoxelLength.x;
				bool cross_pos_dy = CROSS_VALID(cross_z3) || CROSS_VALID(cross_z2) || CROSS_VALID(cross_x1);
				Eigen::Vector3f vNearNormal(vVolumeCopy[oNearPos].normal_x, vVolumeCopy[oNearPos].normal_y, vVolumeCopy[oNearPos].normal_z);
				float current_dot_ref = m_pUpdateStrategy->IsSoftDynamic(vVolumeCopy[oNearPos].data_c[1]) ? fStrictDotRef : normal_dot_ref;
				bool normal_dot = vNearNormal.dot(vNormal) > current_dot_ref;

				if(cross_pos_dy && normal_dot || confidence_connect) m_oUnionSet.Union(oPos, oNearPos);
			}
		}
		if(cross_neg_dz) {
			HashPos oNearPos(oPos.x, oPos.y, oPos.z - 1);
			if(vVolumeCopy.count(oNearPos)) {
				pcl::PointNormal& oNearPoint = vVolumeCopy[oNearPos];

				int current_confidence = oNearPoint.data_n[3] / fConfidenceLevelLength;
				if(current_confidence > 4) current_confidence = 4;
				int current_time_stamp = oNearPoint.data_c[2];
				bool confidence_connect = current_confidence == 4 && confidence == 4 && abs(current_time_stamp - time_stamp) < time_diff_ref;

				float A = oNearPoint.normal_x, B = oNearPoint.normal_y, C = oNearPoint.normal_z;
				float neg_D = A * oNearPoint.x + B * oNearPoint.y + C * oNearPoint.z;
				pcl::PointXYZ oPosPoint = HashPosTo3DPos(oNearPos);
				int cross_y1 = B ? (neg_D - A * oPosPoint.x - C * (oPosPoint.z + m_oVoxelLength.z)) / B - oPosPoint.y : -1;
				int cross_y2 = B ? (neg_D - A * (oPosPoint.x + m_oVoxelLength.x) - C * (oPosPoint.z + m_oVoxelLength.z)) / B - oPosPoint.y : -1;
				int cross_x3 = A ? (neg_D - B * oPosPoint.y - C * (oPosPoint.z + m_oVoxelLength.z)) / A - oPosPoint.x : -1;
				cross_y1 /= m_oVoxelLength.y;
				cross_y2 /= m_oVoxelLength.y;
				cross_x3 /= m_oVoxelLength.x;
				bool cross_pos_dz = CROSS_VALID(cross_y1) || CROSS_VALID(cross_y2) || CROSS_VALID(cross_x3);
				Eigen::Vector3f vNearNormal(vVolumeCopy[oNearPos].normal_x, vVolumeCopy[oNearPos].normal_y, vVolumeCopy[oNearPos].normal_z);
				float current_dot_ref = m_pUpdateStrategy->IsSoftDynamic(vVolumeCopy[oNearPos].data_c[1]) ? fStrictDotRef : normal_dot_ref;
				bool normal_dot = vNearNormal.dot(vNormal) > current_dot_ref;

				if(cross_pos_dz && normal_dot || confidence_connect) m_oUnionSet.Union(oPos, oNearPos);
			}
		}
		//*/
	}
}


void HashVoxeler::DrawVolume(const HashVolume & vVolume, visualization_msgs::MarkerArray & oOutputVolume) {

	constexpr int index = 1e4;

	visualization_msgs::Marker oVolumeMarker;
	oVolumeMarker.header.frame_id = "map";
	oVolumeMarker.header.stamp = ros::Time::now();
	oVolumeMarker.type = visualization_msgs::Marker::CUBE_LIST;
	oVolumeMarker.action = visualization_msgs::Marker::MODIFY;
	oVolumeMarker.id = index; 

	oVolumeMarker.scale.x = m_oVoxelLength.x;
	oVolumeMarker.scale.y = m_oVoxelLength.y;
	oVolumeMarker.scale.z = m_oVoxelLength.z;

	oVolumeMarker.pose.position.x = 0.0;
	oVolumeMarker.pose.position.y = 0.0;
	oVolumeMarker.pose.position.z = 0.0;

	oVolumeMarker.pose.orientation.x = 0.0;
	oVolumeMarker.pose.orientation.y = 0.0;
	oVolumeMarker.pose.orientation.z = 0.0;
	oVolumeMarker.pose.orientation.w = 1.0;

	oVolumeMarker.color.a = 1.0;
	oVolumeMarker.color.r = random() / (float)RAND_MAX;
	oVolumeMarker.color.g = random() / (float)RAND_MAX;
	oVolumeMarker.color.b = random() / (float)RAND_MAX;

	for(auto && [oPos, _] : vVolume) {

		auto o3DPos = HashPosTo3DPos(oPos);
		geometry_msgs::Point point;
		point.x = o3DPos.x + m_oVoxelLength.x / 2;
		point.y = o3DPos.y + m_oVoxelLength.y / 2;
		point.z = o3DPos.z + m_oVoxelLength.z / 2;
		oVolumeMarker.points.push_back(point);
	}

	oOutputVolume.markers.push_back(oVolumeMarker);
}


void HashVoxeler::DrawUnionSet(visualization_msgs::MarkerArray& oOutputUnionSet) {

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

			oCurrentSet.scale.x = m_oVoxelLength.x;
			oCurrentSet.scale.y = m_oVoxelLength.y;
			oCurrentSet.scale.z = m_oVoxelLength.z;

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
				point.x = o3DPos.x + m_oVoxelLength.x / 2;
				point.y = o3DPos.y + m_oVoxelLength.y / 2;
				point.z = o3DPos.z + m_oVoxelLength.z / 2;
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

		oCurrentSet.scale.x = m_oVoxelLength.x;
		oCurrentSet.scale.y = m_oVoxelLength.y;
		oCurrentSet.scale.z = m_oVoxelLength.z;

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


/*=======================================
UpdateUnionConflict
Input: m_oUnionSet - the union set saved in volume
Output: m_vVolume - update the volume according to the union set
Function: if the size of the connected set of a voxel is below a const int, then remove the voxel
========================================*/
void HashVoxeler::UpdateUnionConflict(const int iRemoveSetSizeRef, const float fRemoveTimeRef) {

	std::shared_lock<std::shared_mutex> union_read_lock(m_mUnionSetLock);
	auto vVoxelSets = m_oUnionSet.GetSets();

	std::unique_lock<std::shared_mutex> volume_write_lock(m_mVolumeLock);
	for(auto && [oPos, oPosList] : vVoxelSets) {
		if(oPosList.size() < iRemoveSetSizeRef && !m_oUnionSet.InMaxSet(oPos)) {
			for(auto && oPosInSet : oPosList) {
				float& fTimeStamp = m_vVolume[oPosInSet].data_c[2];
				if(m_iFrameCount - fTimeStamp > fRemoveTimeRef) {
					float& fConflictTime = m_vVolume[oPosInSet].data_c[1];
					bool bToDelete = m_pUpdateStrategy->Conflict(fConflictTime);
					// if(bToDelete) {
					// 	m_vVolume.erase(oPosInSet);
					// 	// delete recent voxel
					// 	if(m_vRecentVolume.count(oPosInSet)) {
					// 		m_vRecentVolume.erase(oPosInSet);
					// 	}
					// } 
					// else {
					// 	float& fConfidence = m_vVolume[oPosInSet].data_n[3];
					// 	fConfidence *= 0.5;
					// 	if(fConfidence < 0) fConfidence = 0;
					// 	// sync recent voxel
					// 	if(m_vRecentVolume.count(oPosInSet)) {
					// 		m_vRecentVolume[oPosInSet] = m_vVolume[oPosInSet];
					// 	}
					// }
				}
					
				// sync recent voxel
				if(m_vRecentVolume.count(oPosInSet)) {
					m_vRecentVolume[oPosInSet] = m_vVolume[oPosInSet];
				}
			}
		}
	}
}


/*=======================================
PointBelongVoxelPos
Input: oPoint - the input point to be judged
Output: oPos - which voxel pos does the point belong
Function: calculate the voxel pos the point belongs
========================================*/
void HashVoxeler::PointBelongVoxelPos(const pcl::PointNormal & oPoint, HashPos & oPos) const {
	
	oPos.x = floor(oPoint.x / m_oVoxelLength.x);
	oPos.y = floor(oPoint.y / m_oVoxelLength.y);
	oPos.z = floor(oPoint.z / m_oVoxelLength.z);
}


/*=======================================
GetVoxelPos
Input: oPoint - the input point to be judged
		oVoxelLength - length of voxel edge
Output: oPos - which voxel pos does the point belong
Function: calculate the voxel pos the point belongs
========================================*/
template<class PointType>
HashPos HashVoxeler::GetVoxelPos(const PointType & oPoint, const pcl::PointXYZ & oVoxelLength) {

	HashPos oPos;
	oPos.x = floor(oPoint.x / oVoxelLength.x);
	oPos.y = floor(oPoint.y / oVoxelLength.y);
	oPos.z = floor(oPoint.z / oVoxelLength.z);
	return oPos;
}
template HashPos HashVoxeler::GetVoxelPos(const pcl::PointNormal & oPoint, const pcl::PointXYZ & oVoxelLength);


/*=======================================
VoxelizePoints
Input: vCloud - the input point cloud
Output: m_vVolume - the volume result of all points
Function: put the points into their voxels and fusion, update the union set
========================================*/
void HashVoxeler::VoxelizePointsAndFusion(pcl::PointCloud<pcl::PointNormal> & vCloud) {
	
	std::unique_lock<std::shared_mutex> volume_write_lock(m_mVolumeLock);
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
	for(auto&& [oPos, vIndexList] : vPointContainer) {

		bool bNewVoxel = !m_vVolume.count(oPos);

		// init base point
		pcl::PointNormal oBasePoint;
		if(!bNewVoxel) {
			oBasePoint = m_vVolume[oPos];
		}

		// comptue point flow
		if(!bNewVoxel) {
			pcl::PointNormal oFusedDirect = oVoxelFusion.NormalFusionWeighted(vIndexList, vCloud, pcl::PointNormal());

			// oFlow = (oFlow + oFusedDirect - oBasePoint) * 0.5;
			pcl::PointNormal& oFlow = m_vPointFlow[oPos];
			__m128 a = _mm_load_ps(oFusedDirect.data);
			a = _mm_add_ps(a, _mm_load_ps(oFlow.data));
			a = _mm_sub_ps(a, _mm_load_ps(oBasePoint.data));
			a = _mm_mul_ps(a, _mm_set_ps(0.5f,0.5f,0.5f,0.5f));
			_mm_store_ps(oFlow.data, a);
		}

		// fusion points
		pcl::PointNormal oFusedPoint = oVoxelFusion.NormalFusionWeighted(vIndexList, vCloud, oBasePoint);
		float& fFlowScore = oFusedPoint.data_c[0];
		fFlowScore = FlowScore(oPos);
		float& fConflictTime = oFusedPoint.data_c[1];
		m_pUpdateStrategy->Support(fConflictTime);
		float& fTimeStamp = oFusedPoint.data_c[2];
		fTimeStamp = m_iFrameCount;
		m_vVolume[oPos] = oFusedPoint;

		// update recent volume
		m_vRecentVolume[oPos] = oFusedPoint;
	}

	// delete not recent voxels
	std::vector<HashPos> vPosToDelete;
	for(auto && [oPos,oPoint] : m_vRecentVolume) {
		float& fTimeStamp = oPoint.data_c[2];
		if(m_iFrameCount - fTimeStamp >= m_iMaxRecentKeep)
			vPosToDelete.emplace_back(oPos);
	}
	for(auto && oPos : vPosToDelete)
		m_vRecentVolume.erase(oPos);
}


/*=======================================
UpdateConflictResult
Input: vVolumeCloud - the conflict result that calculated by the 'surfel fusion' liked method
Output: m_vVolume - update the volume
Function: decrease the confidence of dynamic point and record conflict
========================================*/
void HashVoxeler::UpdateConflictResult(const pcl::PointCloud<pcl::PointNormal> & vVolumeCloud, const bool bKeepVoxel) {

	std::unique_lock<std::shared_mutex> volume_write_lock(m_mVolumeLock);

	HashPos oPos;
	for(auto && oVoxelPoint : vVolumeCloud) {

		PointBelongVoxelPos(oVoxelPoint, oPos);
		const float& fDeconfidence = oVoxelPoint.data_n[3];

		if(fDeconfidence < 0 && m_vVolume.count(oPos)) {

			float& fConflictTime = m_vVolume[oPos].data_c[1];
			bool bToDelete = m_pUpdateStrategy->Conflict(fConflictTime);

			if(bToDelete && !bKeepVoxel) {
				m_vVolume.erase(oPos);
				// delete recent voxel
				if(m_vRecentVolume.count(oPos)) {
					m_vRecentVolume.erase(oPos);
				}
			} 
			else {
				float& fConfidence = m_vVolume[oPos].data_n[3];
				fConfidence -= fDeconfidence;
				if(fConfidence < 0) fConfidence = 0;
				// sync recent voxel
				if(m_vRecentVolume.count(oPos)) {
					m_vRecentVolume[oPos] = m_vVolume[oPos];
				}
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

	vCornerPoses.emplace_back(oVoxel.x, 		oVoxel.y, 		oVoxel.z	);
	vCornerPoses.emplace_back(oVoxel.x, 		oVoxel.y + 1,	oVoxel.z	);
	vCornerPoses.emplace_back(oVoxel.x + 1, 	oVoxel.y + 1, 	oVoxel.z	);
	vCornerPoses.emplace_back(oVoxel.x + 1,		oVoxel.y, 		oVoxel.z	);
	vCornerPoses.emplace_back(oVoxel.x, 		oVoxel.y, 		oVoxel.z + 1);
	vCornerPoses.emplace_back(oVoxel.x, 		oVoxel.y + 1, 	oVoxel.z + 1);
	vCornerPoses.emplace_back(oVoxel.x + 1,		oVoxel.y + 1,	oVoxel.z + 1);
	vCornerPoses.emplace_back(oVoxel.x + 1, 	oVoxel.y,	 	oVoxel.z + 1);
}

/*=======================================
HashPosTo3DPos(static)
Input: 	oCornerPos - the 3d index of a voxel cube corner
		oVoxelLength - the edge length of a voxel cube
Output: oCorner3DPos - the 3d gt position of the corner point 
Function: transfer a HashPos variable to 3d gt position (eigen styled)
========================================*/
void HashVoxeler::HashPosTo3DPos(const HashPos & oCornerPos, const pcl::PointXYZ & oVoxelLength, Eigen::Vector3f & oCorner3DPos) {
	
	oCorner3DPos.x() = oCornerPos.x * oVoxelLength.x;
	oCorner3DPos.y() = oCornerPos.y * oVoxelLength.y;
	oCorner3DPos.z() = oCornerPos.z * oVoxelLength.z;
}


/*=======================================
HashPosTo3DPos
Input: 	oPos - the 3d index of a voxel cube corner
Output: pcl::PointXYZ - the 3d gt position of the corner point
Function: transfer a HashPos variable to 3d gt position (pcl styled)
========================================*/
pcl::PointXYZ HashVoxeler::HashPosTo3DPos(const HashPos & oPos) const {

	return pcl::PointXYZ(oPos.x * m_oVoxelLength.x, oPos.y * m_oVoxelLength.y, oPos.z * m_oVoxelLength.z);
}