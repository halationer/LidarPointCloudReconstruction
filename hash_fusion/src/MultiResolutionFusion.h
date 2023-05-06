#ifndef __MULTI_RESOLUTION_FUSION__
#define __MULTI_RESOLUTION_FUSION__

#include "FramesFusion.h"
#include "HashVoxeler.h"

#include <vector>

class MultiResolutionFusion : public FramesFusion {

    typedef FramesFusion Super;

public:

    MultiResolutionFusion(ros::NodeHandle & node, ros::NodeHandle & nodeHandle):Super(node, nodeHandle) {
        ReadLaunchParams(nodeHandle);
    }

    ~MultiResolutionFusion() {}
    
    // params 
    float m_fExpandDistributionRef;

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
    HashVoxeler m_oDownResolutionVoxeler;
};

#endif