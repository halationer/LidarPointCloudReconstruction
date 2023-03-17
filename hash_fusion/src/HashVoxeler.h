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
        return abs((pos.x * 131.1f + pos.y) * 131.2f + pos.z); // faster
		// return (((std::hash<float>()(pos.x) << 1) ^ std::hash<float>()(pos.y)) << 1) ^ std::hash<float>()(pos.z);
    }
};

class UnionSet {
private:
	std::unordered_map<HashPos, std::pair<HashPos,int>, HashFunc> m_vUnionSet;
public:
	mutable std::unordered_map<HashPos, std::vector<HashPos>, HashFunc> m_vSetBuffer;
	int m_iMaxSetSize = 0;

public:
	void Clear() {
		std::unordered_map<HashPos, std::pair<HashPos,int>, HashFunc> temp;
		m_vUnionSet.swap(temp);
	}
	HashPos Find(const HashPos& oPos) {
		if(m_vUnionSet.count(oPos)) {
			if(oPos == m_vUnionSet[oPos].first) return oPos;
			else return Find(m_vUnionSet[oPos].first);
		}
		m_vUnionSet[oPos] = {oPos, 1};
		return oPos;
	}
	void Union(const HashPos& oPosA, const HashPos& oPosB) {
		HashPos oRootAPos = Find(oPosA), oRootBPos = Find(oPosB);
		if(oRootAPos == oRootBPos) return;
		auto &oRootA = m_vUnionSet[oRootAPos], &oRootB = m_vUnionSet[oRootBPos];
		if(oRootA.second > oRootB.second) {
			oRootB.first = oRootA.first;
			oRootA.second += oRootB.second;
			m_iMaxSetSize = std::max(m_iMaxSetSize, oRootA.second);
		}
		else {
			oRootA.first = oRootB.first;
			oRootB.second += oRootA.second;
			m_iMaxSetSize = std::max(m_iMaxSetSize, oRootB.second);
		}
	}
	int GetSetSize(const HashPos& oPos) {
		return m_vUnionSet[Find(oPos)].second;
	}
	bool InMaxSet(const HashPos& oPos) {
		return GetSetSize(oPos) == m_iMaxSetSize;
	}
	std::unordered_map<HashPos, std::vector<HashPos>, HashFunc>& GetSets() {
		m_vSetBuffer.clear();
		for(auto && [oPos, oPair] : m_vUnionSet) {
			m_vSetBuffer[Find(oPos)].push_back(oPos);
		}
		return m_vSetBuffer;
	}
	std::vector<HashPos>& GetMaxSet(){
		m_vSetBuffer.clear();
		int iMaxSize = 0;
		HashPos oMaxRootPos;
		for(auto && [oPos,_] : m_vUnionSet) {
			std::pair<HashPos,int>& oPair = m_vUnionSet[Find(oPos)];
			if(iMaxSize > oPair.second) continue;
			if(iMaxSize < oPair.second) {
				iMaxSize = oPair.second;
				oMaxRootPos = oPair.first;
			}
			m_vSetBuffer[oPair.first].push_back(oPos);
		}
		return m_vSetBuffer[oMaxRootPos];
	}
};

class HashVoxeler{

public:

	int m_iFrameCount;

	// size (or resolution) of each voxel
	pcl::PointXYZ m_oVoxelLength;

	typedef std::unordered_map<HashPos, pcl::PointNormal, HashFunc> HashVolume;

private:
	mutable std::shared_mutex m_mVolumeLock;

	/*  data structure
		point.data_c[1] - conflict times
		point.data_c[2] - time stamp
		point.data_c[3] - normal_distribution_distance
		point.data-n[3] - confidence */ 
    HashVoxeler::HashVolume m_vVolume;

	std::shared_ptr<VolumeUpdateStrategy> m_pUpdateStrategy;

	// UnionSet m_oUnionSet;

	std::unordered_map<HashPos, pcl::PointNormal, HashFunc> m_vPointFlow;
	bool IsNoneFlow(const HashPos& oPos);

public:

	// Constructor and destructor.
	HashVoxeler();

	~HashVoxeler();

	// get volume
	void GetVolume(HashVoxeler::HashVolume & vVolumeCopy) const;
	void GetStrictStaticVolume(HashVoxeler::HashVolume & vVolumeCopy) const;
	void GetVolumeCloud(pcl::PointCloud<pcl::PointNormal> & vVolumeCloud) const;
	void GetRecentVolume(HashVoxeler::HashVolume & vVolumeCopy, const int iRecentTime) const;

	// get filtered volume
	void GetRecentMaxConnectVolume(HashVoxeler::HashVolume & vVolumeCopy, const int iRecentTime);
	void GetRecentNoneFlowVolume(HashVoxeler::HashVolume & vVolumeCopy, const int iRecentTime);

	// build union set
	void RebuildUnionSet(UnionSet& oUnionSet, const int iRecentTime);
	void UpdateUnionConflict(UnionSet& oUnionSet);

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