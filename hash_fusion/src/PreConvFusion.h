#ifndef __PRECONV_FUSION__
#define __PRECONV_FUSION__

#include "FramesFusion.h"
#include "HashVoxeler.h"
#include "SignedDistance.h"

class ExpandVoxeler : public HashVoxeler {
    
    typedef HashVoxeler Super;

public:

    int m_iConvDim;
    int m_iConvHalfDim;

    void SetConvDim(int iConvDim) {m_iConvDim = iConvDim; m_iConvHalfDim = iConvDim / 2; }

    // voxelize the points and fuse them
	void VoxelizePointsAndFusion(pcl::PointCloud<pcl::PointNormal> & vCloud);

	// decrease the confidence of dynamic point and record conflict
	void UpdateConflictResult(const pcl::PointCloud<pcl::PointNormal> & vVolumeCloud);
};


class PreConvSD : public SignedDistance {

    typedef SignedDistance Super;

public:

    PreConvSD(int iKeepTime = 50, int iConvDim = 3, int iConvAddPointNumRef = 5, float fConvFusionDistanceRef1 = 0.95f)
        :Super(iKeepTime, iConvDim, iConvAddPointNumRef, fConvFusionDistanceRef1) {};
    
    
	std::unordered_map<HashPos, float, HashFunc> & PreConvGlance(HashVoxeler & oVoxeler, ExpandVoxeler & oPreConvVoxeler);
};


class PreConvFusion : public FramesFusion {

    typedef FramesFusion Super;

public:

    PreConvFusion(ros::NodeHandle & node, ros::NodeHandle & nodeHandle):Super(node, nodeHandle) {
        ReadLaunchParams(nodeHandle);
    }

    ~PreConvFusion() {}
    
    // params 
    int m_iConvHalfDim;

    void LazyLoading() override;
    
protected:

    bool ReadLaunchParams(ros::NodeHandle & nodeHandle);

    void SlideModeling(pcl::PolygonMesh & oResultMesh, const int iFrameId) override;

    void SurfelFusionQuick(pcl::PointNormal oLidarPos, pcl::PointCloud<pcl::PointNormal>& vDepthMeasurementCloud) override;

    void UpdateOneFrame(const pcl::PointNormal& oViewPoint, pcl::PointCloud<pcl::PointNormal>& vFilteredMeasurementCloud) override;

    /** father data
        @param m_vMapPCN    pcl::PointCloud<pcl::PointNormal> - final saved cloud
        @param m_oVoxeler   HashVoxeler - volume manager
    */
    ExpandVoxeler m_oPreConvVoxeler;
};

#endif