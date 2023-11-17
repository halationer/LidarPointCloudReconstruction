#ifndef SIMPLE_VOLUME_H
#define SIMPLE_VOLUME_H

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
#include"volume/VolumeBase.h"

class Voxel {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	pcl::PointNormal point;
    virtual void AddStatus(int intensity) = 0;
    virtual int GetStatus(int intensity) = 0;

    typedef std::shared_ptr<Voxel> Ptr;
};


class StatusVoxel : public Voxel{

    unsigned long long status = 0; // 16 * 4

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void AddStatus(int intensity) override {
        int position = intensity << 2;
        char cur = (status & (0xfull << position)) >> position;
        if(cur != 0xfll) ++cur;
        status = status & ~(0xfull << position) | (cur << position);
        // std::cout << std::hex << status << " " << intensity << " " << (int)cur << std::endl;
    }

    int GetStatus(int intensity) override {
        int position = intensity << 2;
        return (status & (0xfull << position)) >> position;
    }
};


// volume base class (abstract)
class SimpleVolume : public VolumeBase{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::unordered_map<HashPos, Voxel::Ptr, HashFunc> Volume;
    Volume m_vVolume;

public:
    void InitLog() const override { std::cout << "Load SimpleVolume.." << std::endl; }
	Eigen::Vector3f GetVoxelLength() const override {return m_vVoxelSize;}

	// voxelize the points and fuse them
	virtual void VoxelizePointsAndFusion(pcl::PointCloud<pcl::PointNormal> & vCloud) override {};
	// decrease the confidence of dynamic point and record conflict
	virtual void UpdateConflictResult(const pcl::PointCloud<pcl::PointNormal> & vVolumeCloud, const bool bKeepVoxel = false) override {};
	// transfer the position 
	virtual void PointBelongVoxelPos(const pcl::PointNormal & oPoint, HashPos & oPos) const override;

public:
    template<class PointType>
    void PointBelongVoxelPos(const PointType & oPoint, HashPos & oVoxelPos) const;
    void PointBelongVoxelPos(const Eigen::Vector3f & vPoint, HashPos & oVoxelPos) const;
    void SetResolution(pcl::PointXYZ & oLength);
	void VoxelizePointsAndFusion(pcl::PointCloud<pcl::PointXYZI> & vCloud);
    void PublishType(int intensity);
    void PublishType();

private:
	Eigen::Vector3f m_vVoxelSize; 
	Eigen::Vector3f m_vVoxelHalfSize;
	Eigen::Vector3f m_vVoxelSizeInverse;
};


#endif