#ifndef VOLUME_BASE_H
#define VOLUME_BASE_H

#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/kdtree/kdtree.h>
#include<unordered_map>
#include<Eigen/Core>
#include<vector>
#include<shared_mutex>
#include<memory>
#include <visualization_msgs/MarkerArray.h>

#include"tools/VolumeUpdateStrategy.h"
#include"tools/UnionSet.h"
#include"tools/HashPos.h"

struct VoxelBase {
	
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Eigen::Vector3f position;
    // Eigen::Vector3f normal;
	pcl::PointNormal point;

	bool bIsOccupied;
};


// class VoxelIteratorBase {
// public:
//     VoxelBase& get() { return ptr; }
//     virtual VoxelIteratorBase& operator++() = 0;
//     virtual VoxelIteratorBase  operator++(VoxelIteratorBase&) = 0;
// private:
//     VoxelBase* ptr;
// }

// volume base class (abstract)
class VolumeBase{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	typedef std::unordered_map<HashPos, pcl::PointNormal, HashFunc> HashVolume;
	
	enum VolumeType {
		HASH_VOXELER = 0,
		HASH_BLOCK
	};

	static VolumeBase* CreateVolume(enum VolumeType volumeType);
	static void GetCornerPoses(const HashPos & oVoxel, std::vector<HashPos> & vCornerPoses);
	static void HashPosTo3DPos(const HashPos & oCornerPos, const Eigen::Vector3f & oVoxelLength, Eigen::Vector3f & oCorner3DPos);

	virtual void InitLog() = 0;
	virtual Eigen::Vector3f GetVoxelLength() const = 0;

	// update vehicle move status
	virtual void UpdateLidarCenter(Eigen::Vector3f& oLidarCenter){}

	// get volume
	virtual void GetAllVolume(HashVolume & vVolumeCopy) const{}
	virtual void GetStaticVolume(HashVolume & vVolumeCopy) const{}
	virtual void GetStrictStaticVolume(HashVolume & vVolumeCopy) const{}
	virtual void GetVolumeCloud(pcl::PointCloud<pcl::PointNormal> & vVolumeCloud) const{}
	virtual void GetRecentVolume(HashVolume & vVolumeCopy, const int iRecentTime) const{}
	virtual void GetRecentAllVolume(HashVolume & vVolumeCopy, const int iRecentTime) const{}
	virtual void GetLocalVolume(HashVolume & vVolumeCopy, const Eigen::Vector3f vCenter, const float fRadius) const{}

	// get filtered volume
	virtual void GetAllConnectVolume(HashVolume & vVolumeCopy, const int iConnectMinSize){}
	virtual void GetRecentConnectVolume(HashVolume & vVolumeCopy, const int iRecentTime, const int iConnectMinSize){}
	virtual void GetRecentMaxConnectVolume(HashVolume & vVolumeCopy, const int iRecentTime){}
	virtual void GetRecentNoneFlowVolume(HashVolume & vVolumeCopy, const int iRecentTime){}
	virtual void GetRecentHighDistributionVolume(HashVolume & vVolumeCopy, const int iRecentTime){}
	virtual void GetLocalConnectVolume(HashVolume & vVolumeCopy, const Eigen::Vector3f vCenter, const float fRadius, const int iConnectMinSize){}

	virtual void DrawVolume(const HashVolume & vVolume, visualization_msgs::MarkerArray & oOutputVolume){}

	// build union set
	virtual void RebuildUnionSetAll(const float fStrictDotRef, const float fSoftDotRef, const float fConfidenceLevelLength){}
	virtual void RebuildUnionSet(const float fStrictDotRef, const float fSoftDotRef, const float fConfidenceLevelLength){}
	virtual void RebuildUnionSetCore(HashVolume & vVolumeCopy, const float fStrictDotRef, const float fSoftDotRef, const float fConfidenceLevelLength){}
	virtual void DrawUnionSet(visualization_msgs::MarkerArray& oOutputUnionSet){}
	virtual void UpdateUnionConflict(const int iRemoveSetSizeRef, const float fRemoveTimeRef){}

	virtual void ClearFlow(){}

	// set the resolution of voxel
	virtual void SetResolution(pcl::PointXYZ & oLength) {}
	// Set the strategy of fusion and remove dynamic points
	virtual void SetStrategy(enum vus eStrategy) {}
	// Copy strategy from another
	virtual void GetStrategy(VolumeBase& oVolume) {}

	// voxelize the points and fuse them
	virtual void VoxelizePointsAndFusion(pcl::PointCloud<pcl::PointNormal> & vCloud) = 0;

	// decrease the confidence of dynamic point and record conflict
	virtual void UpdateConflictResult(const pcl::PointCloud<pcl::PointNormal> & vVolumeCloud, const bool bKeepVoxel = false) = 0;

	// transfer the position 
	// !!! notice : template should not be virtual
	virtual void PointBelongVoxelPos(const pcl::PointNormal & oPoint, HashPos & oPos) const = 0;
	// template<class PointType>
	// virtual static HashPos GetVoxelPos(const PointType & oPoint, const pcl::PointXYZ & oVoxelLength){}

	// calcualte corner poses
	// !!! notice : template should not be static
	// virtual static void GetCornerPoses(const HashPos & oVoxelPos, std::vector<HashPos> & vCornerPoses){}
	// virtual static void HashPosTo3DPos(const HashPos & oCornerPos, const pcl::PointXYZ & oVoxelLength, Eigen::Vector3f & oCorner3DPos){}
	virtual pcl::PointXYZ HashPosTo3DPos(const HashPos & oPos){}

};


#endif