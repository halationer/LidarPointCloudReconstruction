#ifndef HASH_VOLUME_H
#define HASH_VOLUME_H

#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/kdtree/kdtree.h>
#include<unordered_map>
#include<Eigen/Core>
#include<vector>
#include<shared_mutex>

//IndexinAxis
//index value on x, y, z axis
struct IndexinAxis{

	unsigned int ixnum;
	unsigned int iynum;
	unsigned int iznum;

};

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
	typedef std::unordered_map<HashPos, pcl::PointNormal, HashFunc> HashVolume;
	std::unordered_map<HashPos, pcl::PointXYZ, HashFunc> m_vCorner;

private:
	mutable std::shared_mutex m_mVolumeLock;
    HashVoxeler::HashVolume m_vVolume;

public:

	// Constructor and destructor.
	HashVoxeler();

	~HashVoxeler();

	// get volume
	void GetVolume(HashVoxeler::HashVolume & vVolumeCopy) const;

	// set the resolution of voxel
	void GetResolution(pcl::PointXYZ & oLength);

	// transfer the position
	template<class PointType>
	void PointBelongVoxelPos(const PointType & oPoint, HashPos & oPos);


	// voxelize the points and fuse them
	void VoxelizePointsAndFusion(pcl::PointCloud<pcl::PointNormal> & vCloud);

	// calcualte corner poses
	static void GetCornerPoses(const HashPos & oVoxelPos, std::vector<HashPos> & vCornerPoses);

	static void HashPosTo3DPos(const HashPos & oCornerPos, const pcl::PointXYZ & oVoxelSize, Eigen::Vector3f & oCorner3DPos);

	// test function
	pcl::PointXYZ HashPosTo3DPos(const HashPos & oPos);

public: 
	//=======data========
	//the original point expanded by m_oMinCorner
	//it is the origin of the voxel coordinate system
	pcl::PointXYZ m_oOriCorner;

	//the voxel number in one axis
	IndexinAxis m_iNoExpandVoxelNum;

	//the final voxel number in one axis after expand
	IndexinAxis m_iFinalVoxelNum;

	//the length of a voxel
	pcl::PointXYZ m_oVoxelLength;

	//the corner point clouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_pCornerCloud;//location

	//corner point value - signed distance value for marching cube algorithm
	//std::vector<float> m_vCornerValue;

	//1D index to 3D index (m_vCornerValue is a one-dimension vector)
	std::vector<IndexinAxis> m_vCorToGridIdx;

	//the index of point in each voxel
	std::vector <std::vector<int>> m_vVoxelPointIdx;

	//
	pcl::PointCloud<pcl::PointNormal>::Ptr m_pVoxelNormals;

	//a vector indicates whether the corner is adjacent to the surface
	std::vector<bool> m_vNearStatus;

	//grid default
	float m_fDefault;

private:

	//min x value of all input point
	float m_fMinX;
	//min y value of all input point
	float m_fMinY;
	//min z value of all input point
	float m_fMinZ;
	//max x value x of all input point
	float m_fMaxX;
	//max y value of all input point
	float m_fMaxY;
	//max z value of all input point
	float m_fMaxZ;

	//the corner of bounding box with min value
	pcl::PointXYZ m_oMinCorner;

	//the corner of bounding box with max value
	pcl::PointXYZ m_oMaxCorner;

	//indicates the parameter has been set
	bool m_bParamFlag;

	//expand number of bound
	float m_fExpandNum;
};


#endif