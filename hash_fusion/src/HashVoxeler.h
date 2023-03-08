#ifndef HASH_VOLUME_H
#define HASH_VOLUME_H

#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/kdtree/kdtree.h>
#include<unordered_map>
#include<Eigen/Core>
#include<vector>
#include<shared_mutex>

#include"VolumeUpdateStrategy.h"

struct HashPos {
    int x, y, z;
    HashPos():x(0),y(0),z(0){}
    HashPos(int x, int y, int z):x(x),y(y),z(z){}
};
bool operator == (const HashPos & a, const HashPos & b);
std::ostream & operator << (std::ostream & out, const HashPos & pos);

struct HashFunc {
    size_t operator()(const HashPos& pos) const {
        return abs((pos.x * 131.1f + pos.y) * 131.2f + pos.z);
    }
};

class HashVoxeler{

public:

	int m_iFrameCount;

	// size (or resolution) of each voxel
	pcl::PointXYZ m_oVoxelLength;

	typedef std::unordered_map<HashPos, pcl::PointNormal, HashFunc> HashVolume;
	std::unordered_map<HashPos, pcl::PointXYZ, HashFunc> m_vCorner;

private:
	mutable std::shared_mutex m_mVolumeLock;

	/*  data structure
		point.data_c[1] - conflict times
		point.data_c[2] - time stamp
		point.data_c[3] - normal_distribution_distance
		point.data-n[3] - confidence */ 
    HashVoxeler::HashVolume m_vVolume;

	std::shared_ptr<VolumeUpdateStrategy> m_pUpdateStrategy;

public:

	// Constructor and destructor.
	HashVoxeler();

	~HashVoxeler();

	// get volume
	void GetVolume(HashVoxeler::HashVolume & vVolumeCopy) const;
	void GetVolumeCloud(pcl::PointCloud<pcl::PointNormal> & vVolumeCloud) const;
	void GetRecentVolume(HashVoxeler::HashVolume & vVolumeCopy, const int iRecentTime) const;

	// set the resolution of voxel
	void SetResolution(pcl::PointXYZ & oLength);
	void SetStrategy(enum vus eStrategy);

	// transfer the position
	template<class PointType>
	void PointBelongVoxelPos(const PointType & oPoint, HashPos & oPos);


	// voxelize the points and fuse them
	void VoxelizePointsAndFusion(pcl::PointCloud<pcl::PointNormal> & vCloud);

	// decrease the confidence of dynamic point and record conflict
	void UpdateConflictResult(const pcl::PointCloud<pcl::PointNormal> & vVolumeCloud);


	// calcualte corner poses
	static void GetCornerPoses(const HashPos & oVoxelPos, std::vector<HashPos> & vCornerPoses);

	static void HashPosTo3DPos(const HashPos & oCornerPos, const pcl::PointXYZ & oVoxelLength, Eigen::Vector3f & oCorner3DPos);

	// test function
	pcl::PointXYZ HashPosTo3DPos(const HashPos & oPos);
};


#endif