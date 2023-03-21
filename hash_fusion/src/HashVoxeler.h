#ifndef HASH_VOLUME_H
#define HASH_VOLUME_H

#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/kdtree/kdtree.h>
#include<unordered_map>
#include<Eigen/Core>
#include<vector>
#include<shared_mutex>

#include"tools/VolumeUpdateStrategy.h"
#include"tools/UnionSet.h"
#include"tools/HashPos.h"

// volume of surfels
class HashVoxeler{

// center component
private:
	mutable std::shared_mutex m_mCenterLock;

	// vehicle move status record
	Eigen::Vector3f m_oLidarCenter;
	float m_fLidarSpeed;
	float m_fRadiusExpandFactor;

	float GetRadiusExpand() const;
public:
	// update vehicle move status
	void UpdateLidarCenter(Eigen::Vector3f& oLidarCenter);

// params
public:
	int m_iRecentTimeToGetRadius;

// main component
public:
	// frame keep time in m_vRecentVolume
	const int m_iMaxRecentKeep;

	// total frame received
	int m_iFrameCount;

	// size (or resolution) of each voxel
	pcl::PointXYZ m_oVoxelLength;

	typedef std::unordered_map<HashPos, pcl::PointNormal, HashFunc> HashVolume;

private:
	// multi-thread lock
	mutable std::shared_mutex m_mVolumeLock;
	mutable std::shared_mutex m_mUnionSetLock;

	/*  data structure
		point.data_c[1] - conflict times
		point.data_c[2] - time stamp
		point.data_c[3] - normal_distribution_distance
		point.data-n[3] - confidence */ 
    HashVoxeler::HashVolume m_vVolume;

	// keep another volume that only save recent
	HashVoxeler::HashVolume m_vRecentVolume;

	// connection relation recorder
	UnionSet m_oUnionSet;

	// confidence update strategy
	std::shared_ptr<VolumeUpdateStrategy> m_pUpdateStrategy;

	// point flow recorder
	std::unordered_map<HashPos, pcl::PointNormal, HashFunc> m_vPointFlow;
	bool IsNoneFlow(const HashPos& oPos);

public:

	// Constructor and destructor.
	HashVoxeler();

	~HashVoxeler();

	// get volume
	void GetStaticVolume(HashVoxeler::HashVolume & vVolumeCopy) const;
	void GetStrictStaticVolume(HashVoxeler::HashVolume & vVolumeCopy) const;
	void GetVolumeCloud(pcl::PointCloud<pcl::PointNormal> & vVolumeCloud) const;
	void GetRecentVolume(HashVoxeler::HashVolume & vVolumeCopy, const int iRecentTime) const;
	void GetLocalVolume(HashVoxeler::HashVolume & vVolumeCopy, const Eigen::Vector3f vCenter, const float fRadius) const;

	// get filtered volume
	void GetRecentMaxConnectVolume(HashVoxeler::HashVolume & vVolumeCopy, const int iRecentTime);
	void GetRecentNoneFlowVolume(HashVoxeler::HashVolume & vVolumeCopy, const int iRecentTime);

	// build union set
	void RebuildUnionSet();
	void UpdateUnionConflict();

	// set the settings of voxel
	void SetResolution(pcl::PointXYZ & oLength);
	void SetStrategy(enum vus eStrategy);

	// voxelize the points and fuse them
	void VoxelizePointsAndFusion(pcl::PointCloud<pcl::PointNormal> & vCloud);

	// decrease the confidence of dynamic point and record conflict
	void UpdateConflictResult(const pcl::PointCloud<pcl::PointNormal> & vVolumeCloud);

	// transfer the position
	template<class PointType>
	void PointBelongVoxelPos(const PointType & oPoint, HashPos & oPos);

	// calcualte corner poses
	static void GetCornerPoses(const HashPos & oVoxelPos, std::vector<HashPos> & vCornerPoses);
	static void HashPosTo3DPos(const HashPos & oCornerPos, const pcl::PointXYZ & oVoxelLength, Eigen::Vector3f & oCorner3DPos);
	pcl::PointXYZ HashPosTo3DPos(const HashPos & oPos);
};


#endif