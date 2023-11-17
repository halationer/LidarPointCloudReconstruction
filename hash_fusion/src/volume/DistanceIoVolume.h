#ifndef DISTANCE_IO_VOLUME
#define DISTANCE_IO_VOLUME

#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/kdtree/kdtree.h>
#include<unordered_map>
#include<Eigen/Core>
#include<vector>
#include<shared_mutex>
#include<memory>
#include<visualization_msgs/MarkerArray.h>
#include<array>

#include"tools/VolumeUpdateStrategy.h"
#include"tools/UnionSet.h"
#include"tools/HashPos.h"
#include"volume/VolumeBase.h"

namespace pcl {

  struct EIGEN_ALIGN16 _PointDistanceIo
  {
    PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
    PCL_ADD_NORMAL4D;
    union
    {
      struct
      {
        float weight;
        float distance;
        float io;
      };
      float data_c[4];
    };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const DistanceIoVoxel& p);

  struct DistanceIoVoxel : public _PointDistanceIo
  {
    inline DistanceIoVoxel (const _PointDistanceIo &p)
    {
      x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
      normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z;
      weight = p.weight; distance = p.distance; io = p.io;
    }
    inline DistanceIoVoxel ()
    {
      x = y = z = 0.0f;
      data[3] = 1.0f;
      normal_x = normal_y = normal_z = data_n[3] = 0.0f;
      weight = distance = io = data_c[4] = 0.0f;
    }
    void Update(float distance, bool in, float weight = 1.0) {
        float sum_weight = this->weight + weight;
        this->distance = (this->distance * this->weight + distance * weight) / sum_weight;
        this->io = (this->io * this->weight + (in ? 1 : 0) * weight) / sum_weight;
        this->weight = sum_weight;
    }
    friend std::ostream& operator << (std::ostream& os, const DistanceIoVoxel& p);
  };
}

// class DistanceIoVoxel {

// public:
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// 	pcl::PointNormal point;
//     float weight = .0f;
//     float distance = .0f;
//     float io = .0f;

//     DistanceIoVoxel(){point.}
//     DistanceIoVoxel(const pcl::PointNormal& point):point(point){}
//     DistanceIoVoxel(const DistanceIoVoxel& voxel):point(voxel.point),weight(voxel.weight),distance(voxel.distance),io(voxel.io){}


//     typedef std::shared_ptr<DistanceIoVoxel> Ptr;
// };


class DistanceIoVolume : public VolumeBase{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::unordered_map<HashPos, pcl::DistanceIoVoxel, HashFunc> Volume;
    Volume m_vVolume;

public:
    void InitLog() const override { std::cout << "Load DistanceIoVolume.." << std::endl; }
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

// main octree
public:
    pcl::PointCloud<DistanceIoVoxel> CreateAndGetCorners(const Eigen::Vector3f& vMin, const Eigen::Vector3f& vMax, size_t iLevel);
    pcl::PointCloud<DistanceIoVoxel> CreateAndGetSubdivideCorners(const pcl::PointCloud<DistanceIoVoxel>& vCorners, size_t iLevel);
    void Fuse(const DistanceIoVolume& oLocal);

private:
    DistanceIoVoxel CreateAndGetVoxel(HashPos& oPos);
    std::array<HashPos, 8> GetCornerPoses(const HashPos& oPos, size_t iLevel);

private:
	Eigen::Vector3f m_vVoxelSize; 
	Eigen::Vector3f m_vVoxelHalfSize;
	Eigen::Vector3f m_vVoxelSizeInverse;
};


#endif