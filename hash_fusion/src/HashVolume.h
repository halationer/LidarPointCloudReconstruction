#ifndef HASH_VOLUME_H
#define HASH_VOLUME_H

#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/kdtree/kdtree.h>
#include<unordered_map>

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

struct HashFunc {
    size_t operator()(const HashPos& pos) const {
        return abs((pos.x * 131.1f + pos.y) * 131.2f + pos.z);
    }
};

class HashVolume{

public:

    std::unordered_map<HashPos, pcl::PointNormal> volume;

    

public:

	// Constructor and destructor.
	HashVolume(const pcl::PointCloud<pcl::PointXYZ> & vCloud);

	// Constructor and destructor.
	HashVolume(const pcl::PointCloud<pcl::PointNormal> & vCloud);

	~HashVolume();

	//set the expand voxel number of boundary
	void SetExpandNum(float fExpandNum);

	//initial node value
	void SetInitialNodeValue(float fDefault = 0.0);

	//GetResolution() and GetIntervalNum are coupled to each other,
	//once one updates,the results of the two functions are both updated
	//get the resolution of voxel, or we say, length of a voxel
	void GetResolution(pcl::PointXYZ oLength);

	//transfor 3d index to 1D index
	int Tran3DIdxTo1D(const IndexinAxis & o3DIdx);

	//transfor 3d index to 1D index
	IndexinAxis Tran1DIdxTo3D(const int & i1DIdx);

	//given a 3d index, compute the corner idex if this query voxel
	void CornerIdxs(const IndexinAxis & o3DIdx, std::vector<int> & vNeighborIdxs);

	//get the number of intervals
	void GetIntervalNum(int iVoxelNumX, int iVoxelNumY, int iVoxelNumZ);

	//set the starting point of the voxel, the length of each voxel and the number of voxels in the x, y, z axis, respectively
	void SetVoxelParam(pcl:: PointXYZ oOrignal, int iNumX, int iNumY, int iNumZ, float fLengthX, float fLengthY, float fLengthZ);

	//voxelize the 3d space, generate voxels
	void VoxelizeSpace();

	//compute a given point belong to which voxel
	template<class PointType>
	int PointBelongVoxel(const PointType & oPoint);
	//reload, compute a given point belong to which voxel with a 3d index output
	template<class PointType>
	int PointBelongVoxel(const PointType & oPoint, IndexinAxis & oP3DIndex);
	//out of border
	template<class PointType>
	bool OutOfBorder(const PointType & oPoint);

	//voxelize the sampled point clouds
	void VoxelizePoints(const pcl::PointCloud<pcl::PointXYZ> & vSampledCloud);
	//voxelize the sampled point clouds
	void VoxelizePoints(const pcl::PointCloud<pcl::PointNormal> & vSampledCloud);
	//voxelize the sampled point clouds and also the raw point clouds
	void VoxelizePoints(const pcl::PointCloud<pcl::PointXYZ> & vSampledCloud, const pcl::PointCloud<pcl::PointXYZ> & vCloud);

	void OutputNonEmptyVoxels(std::vector<bool> & vVoxelStatus);

	//get the nodes near the surface
	//void FindNearNodes(const pcl::PointCloud<pcl::PointXYZ> & vCloud, pcl::PointCloud<pcl::PointXYZ> & vNearNodes);

	//clear some data
	void ClearMiddleData();

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