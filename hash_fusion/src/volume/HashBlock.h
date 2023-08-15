#ifndef HASH_BLOCK_H
#define HASH_BLOCK_H

#include <vector>
#include <unordered_map>
#include <string>

#include "VolumeBase.h"
#include "tools/VolumeUpdateStrategy.h"
#include "tools/OutputUtils.h"
#include "tools/UnionSet.h"
#include "tools/RosPublishManager.h"

struct Block {

    typedef Block *Ptr;
    typedef std::vector<VoxelBase, Eigen::aligned_allocator<VoxelBase>> DataType;
    DataType data;
    HashPos pos;

    Block(){}
    Block(int iVoxelNum, const HashPos& oPos):data(iVoxelNum),pos(oPos){}
    Block(const Block& oBlock):data(oBlock.data),pos(oBlock.pos) {}
    void copy(const Block& oBlock) { data = oBlock.data; pos = oBlock.pos; }

    size_t size() const { return data.size(); }
    VoxelBase& operator[](size_t iIndex) { return data[iIndex]; }
    VoxelBase& at(size_t iIndex) { return data[iIndex]; }

    typedef DataType::iterator iterator;
    typedef DataType::const_iterator const_iterator;
    iterator begin()                { return data.begin(); }
    iterator end()                  { return data.end(); }
    const_iterator begin() const   { return data.begin(); }
    const_iterator end() const     { return data.end(); }
};

class HashBlock : public VolumeBase {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // types
    typedef std::vector<size_t> Indices;

    typedef std::unordered_map<HashPos, Block::Ptr, HashFunc, std::equal_to<HashPos>,
        Eigen::aligned_allocator<std::pair<const HashPos, Block::Ptr>>> HashBlockVolume;

    typedef std::unordered_map<HashPos, Indices, HashFunc, std::equal_to<HashPos>,
        Eigen::aligned_allocator<std::pair<const HashPos, Indices>>> HashIndices;


    // block size
    Eigen::Vector3f m_vBlockSize;
    Eigen::Vector3f m_vBlockSizeInverse;
    Eigen::Vector3f m_vVoxelSize;
    Eigen::Vector3f m_vVoxelHalfSize;
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
    // union read or write lock
	mutable std::shared_mutex m_mUnionSetLock;
	// connection relation recorder
	UnionSet m_oUnionSet;

    // other params
    int m_iFrameCount;
    int m_iMaxRecentKeep;
    int m_iRecentTimeToGetRadius;
    float m_fLidarSpeed; 
    const float m_fRadiusExpandFactor;

    void FusePointToBlock(pcl::PointCloud<pcl::PointNormal> & vCloud, Block& oBlock, Indices& vIndices);
    
public:
    HashBlock();
    ~HashBlock();

    void InitLog() const override { std::cout << "Load HashBlock.." << std::endl; }
    Eigen::Vector3f GetVoxelLength() const override { return m_vVoxelSize; }
    Eigen::Vector3f HashBlockPosTo3DPos(const HashPos & oPos) const {
        return {oPos.x * m_vBlockSize.x(), oPos.y * m_vBlockSize.y(), oPos.z * m_vBlockSize.z()};
    }    
    Eigen::Vector3f HashVoxelIndexTo3DPos(int iIndex) const {
	    // iIndex = (vPointPos.z() * m_vVoxelNumsPerBlock.y() + vPointPos.y()) * m_vVoxelNumsPerBlock.x() + vPointPos.x();
        Eigen::Vector3f vVoxelPos;
        vVoxelPos[0] = iIndex % m_vVoxelNumsPerBlock[0];
        for(int i = 1; i < 3; ++i){
            iIndex -= vVoxelPos[i - 1];
            iIndex /= m_vVoxelNumsPerBlock[i - 1];
            vVoxelPos[i] = iIndex % m_vVoxelNumsPerBlock[i];
        }
        return vVoxelPos.cwiseProduct(m_vVoxelSize) + m_vVoxelHalfSize;
    }

    // set the resolution of voxel
	void SetResolution(pcl::PointXYZ & oLength);
	// Set the strategy of fusion and remove dynamic points
	void SetStrategy(enum vus eStrategy) { m_pUpdateStrategy = std::shared_ptr<VolumeUpdateStrategy>(CreateStrategy(eStrategy)); }
	// Copy strategy from another
	void GetStrategy(HashBlock& oVolume) { m_pUpdateStrategy = oVolume.m_pUpdateStrategy; }

	void GetStaticVolume(HashVolume & vVolumeCopy) const override;
	void GetVolumeCloud(pcl::PointCloud<pcl::PointNormal> & vVolumeCloud) const override;
	void VoxelizePointsAndFusion(pcl::PointCloud<pcl::PointNormal> & vCloud) override;
	void UpdateConflictResult(const pcl::PointCloud<pcl::PointNormal> & vVolumeCloud, const bool bKeepVoxel = false) override;

	void PointBelongVoxelPos(const pcl::PointNormal & oPoint, HashPos & oVoxelPos) const override;
	void PointBelongVoxelPos(const Eigen::Vector3f & vPoint, HashPos & oVoxelPos) const;

	template<class PointType>
	void PointBelongBlockPos(const PointType & oPoint, HashPos & oPos) const;
	template<class PointType>
    void PointBelongVoxelIndex(const PointType & oPoint, int & iIndex) const;
	template<class PointType>
    void PointBelongVoxelIndex(const PointType & oPoint, HashPos & oBlockPos, int & iVoxelIndex) const;
    
    VoxelBase* GetVoxelPtr(const HashPos & oPos);

    // =========================================================================================================================
public:
    // sdf transfer should use these
	void GetRecentVolume(HashVolume & vVolumeCopy, const int iRecentTime) const override;

    // union set should use these
    void GetRecentConnectVolume(HashVolume & vVolumeCopy, const int iRecentTime, const int iConnectMinSize) override;
	void GetLocalVolume(HashVolume & vVolumeCopy, const Eigen::Vector3f vCenter, const float fRadius) const override;
	void RebuildUnionSetCore(HashVolume & vVolumeCopy, const float fStrictDotRef, const float fSoftDotRef, const float fConfidenceLevelLength);
	void RebuildUnionSet(const float fStrictDotRef, const float fSoftDotRef, const float fConfidenceLevelLength) override;
	void UpdateUnionConflict(const int iRemoveSetSizeRef, const float fRemoveTimeRef) override;
	void DrawUnionSet(visualization_msgs::MarkerArray& oOutputUnionSet) override;

    // final output need
	void GetAllVolume(HashVolume & vVolumeCopy) const override;
	void GetAllConnectVolume(HashVolume & vVolumeCopy, const int iConnectMinSize) override;
	void RebuildUnionSetAll(const float fStrictDotRef, const float fSoftDotRef, const float fConfidenceLevelLength) override;

    // ========================================================================================================================
public:
    size_t GetBlockNum() const { return m_vBlockVolume.size(); }
    std::string PrintVolumeStatus() const;

    void GetBlockCopy(const HashPos& oBlockPos, Block& oBlock);
	void UpdateConflictResult(Block & oBlock, const bool bKeepVoxel = false);
};

#endif