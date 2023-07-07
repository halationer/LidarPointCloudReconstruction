#ifndef HASH_BLOCK_H
#define HASH_BLOCK_H

#include <vector>
#include <unordered_map>

#include "VolumeBase.h"
#include "tools/VolumeUpdateStrategy.h"
#include "tools/OutputUtils.h"


class HashBlock : public VolumeBase {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // types
    typedef std::vector<size_t> Indices;
    typedef std::vector<VoxelBase, Eigen::aligned_allocator<VoxelBase>> Block;
    typedef std::unordered_map<HashPos, Block, HashFunc, std::equal_to<HashPos>,
        Eigen::aligned_allocator<std::pair<const HashPos, Block>>> HashBlockVolume;
    typedef std::unordered_map<HashPos, Indices, HashFunc, std::equal_to<HashPos>,
        Eigen::aligned_allocator<std::pair<const HashPos, Indices>>> HashIndices;


    // block size
    Eigen::Vector3f m_vBlockSize;
    Eigen::Vector3f m_vBlockSizeInverse;
    Eigen::Vector3f m_vVoxelSize;
    Eigen::Vector3f m_vVoxelSizeInverse;
    Eigen::Vector3i m_vVoxelNumsPerBlock;
    size_t m_iVoxelFullNumPerBlock;

private:
    // volume read or write lock
	mutable std::shared_mutex m_mVolumeLock;
    // main volume object
    HashBlockVolume m_vBlockVolume;
    // strategy
	std::shared_ptr<VolumeUpdateStrategy> m_pUpdateStrategy;

    // other params
    int m_iFrameCount;
    int m_iMaxRecentKeep;
    int m_iRecentTimeToGetRadius;
    float m_fLidarSpeed; 
    float m_fRadiusExpandFactor;

    void FusePointToBlock(pcl::PointCloud<pcl::PointNormal> & vCloud, Block& oBlock, Indices& vIndices);
    
public:
    HashBlock();
    ~HashBlock();

    // set the resolution of voxel
	void SetResolution(pcl::PointXYZ & oLength) { 
        constexpr int voxel_num_per_block = 16;
        m_vVoxelNumsPerBlock = Eigen::Vector3i(voxel_num_per_block, voxel_num_per_block, voxel_num_per_block);
        m_iVoxelFullNumPerBlock = m_vVoxelNumsPerBlock.prod();
        std::cout << output::format_red << "full num is: " << m_iVoxelFullNumPerBlock << output::format_white << std::endl;
        m_vVoxelSize = oLength.getVector3fMap(); 
        m_vVoxelSizeInverse = m_vVoxelSize.cwiseInverse();
        m_vBlockSize = m_vVoxelSize.cwiseProduct(m_vVoxelNumsPerBlock.cast<float>());
        m_vBlockSizeInverse = m_vBlockSize.cwiseInverse();
    }
	// Set the strategy of fusion and remove dynamic points
	void SetStrategy(enum vus eStrategy) { m_pUpdateStrategy = std::shared_ptr<VolumeUpdateStrategy>(CreateStrategy(eStrategy)); }
	// Copy strategy from another
	void GetStrategy(HashBlock& oVolume) { m_pUpdateStrategy = oVolume.m_pUpdateStrategy; }

	void GetStaticVolume(HashVolume & vVolumeCopy) const override;
	void GetVolumeCloud(pcl::PointCloud<pcl::PointNormal> & vVolumeCloud) const override;
	void VoxelizePointsAndFusion(pcl::PointCloud<pcl::PointNormal> & vCloud) override;
	void UpdateConflictResult(const pcl::PointCloud<pcl::PointNormal> & vVolumeCloud, const bool bKeepVoxel = false) override;


	template<class PointType>
	void PointBelongBlockPos(const PointType & oPoint, HashPos & oPos) const;
	template<class PointType>
	void PointBelongVoxelPos(const PointType & oPoint, HashPos & oPos) const;
	template<class PointType>
    void PointBelongVoxelIndex(const PointType & oPoint, int & iIndex) const;
};

#endif