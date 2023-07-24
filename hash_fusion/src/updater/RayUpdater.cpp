#include"RayUpdater.h"

#include <exception>
#include "tools/OutputUtils.h"
#include "volume/HashBlock.h"

RayCaster::RayCaster(const Eigen::Vector3f& vStartPoint, const Eigen::Vector3f& vEndPoint, const Eigen::Vector3f& vBlockSize)
    :m_vStartPoint(vStartPoint), m_vEndPoint(vEndPoint), m_vBlockSize(vBlockSize) {

    InitCasterParams();
}


bool RayCaster::GetNextHashPos(HashPos& oBlockPos) {

    if(m_iCurrentStep++ > m_iStepNum) {
        return false;
    }

    oBlockPos.x = m_vCurrentPos.x();
    oBlockPos.y = m_vCurrentPos.y();
    oBlockPos.z = m_vCurrentPos.z();
    
    int iDimToGo;
    m_vCurrentToNearstCorner.minCoeff(&iDimToGo);
    m_vCurrentPos[iDimToGo] += m_vStepDirection[iDimToGo];
    m_vCurrentToNearstCorner[iDimToGo] += m_vStepDistance[iDimToGo];
    
    return true;
}


void RayCaster::InitCasterParams() {

    Eigen::Vector3i m_vEndPos = m_vEndPoint.cwiseProduct(m_vBlockSize.cwiseInverse()).array().floor().cast<int>();
    m_vCurrentPos = m_vStartPoint.cwiseProduct(m_vBlockSize.cwiseInverse()).array().floor().cast<int>();
    m_iStepNum = (m_vCurrentPos - m_vEndPos).cwiseAbs().sum();
    m_iCurrentStep = 0;

    Eigen::Vector3f vCurrentToEnd = m_vEndPoint - m_vStartPoint;
    m_vStepDirection = vCurrentToEnd.cwiseSign().cast<int>();

    Eigen::Vector3i vNextCornerDirection = m_vStepDirection.cwiseMax(Eigen::Vector3i::Zero());
    m_vCurrentToNearstCorner = m_vBlockSize.cwiseProduct((m_vCurrentPos + vNextCornerDirection).cast<float>()) - m_vStartPoint;
    m_vStepDistance = m_vStepDirection.cast<float>().cwiseProduct(m_vBlockSize);
    
    vCurrentToEnd = vCurrentToEnd.cwiseInverse();
    m_vCurrentToNearstCorner = m_vCurrentToNearstCorner.cwiseProduct(vCurrentToEnd);
    m_vStepDistance = m_vStepDistance.cwiseProduct(vCurrentToEnd);
}


void RayCaster::MakeRayDebugMarker(const std::vector<HashPos>& vBlockList, visualization_msgs::MarkerArray& oOutputVolume, int iOffset) {

    constexpr int index = 2e4;

    // make blocks
	visualization_msgs::Marker oVolumeMarker;
	oVolumeMarker.header.frame_id = "map";
	oVolumeMarker.header.stamp = ros::Time::now();
	oVolumeMarker.type = visualization_msgs::Marker::CUBE_LIST;
	oVolumeMarker.action = visualization_msgs::Marker::MODIFY;
	oVolumeMarker.id = index + 2 * iOffset; 

	oVolumeMarker.scale.x = m_vBlockSize.x();
	oVolumeMarker.scale.y = m_vBlockSize.y();
	oVolumeMarker.scale.z = m_vBlockSize.z();

	oVolumeMarker.pose.position.x = 0.0;
	oVolumeMarker.pose.position.y = 0.0;
	oVolumeMarker.pose.position.z = 0.0;

	oVolumeMarker.pose.orientation.x = 0.0;
	oVolumeMarker.pose.orientation.y = 0.0;
	oVolumeMarker.pose.orientation.z = 0.0;
	oVolumeMarker.pose.orientation.w = 1.0;

	oVolumeMarker.color.a = std::min(0.2 * (iOffset+1), 1.0);
	oVolumeMarker.color.r = 0.8;
	oVolumeMarker.color.g = 0.2;
	oVolumeMarker.color.b = 0.2;

	for(const HashPos& oPos : vBlockList) {

		Eigen::Vector3f oPoint(oPos.x, oPos.y, oPos.z);
        oPoint = oPoint.cwiseProduct(m_vBlockSize);
        oPoint += 0.5 * m_vBlockSize;

		geometry_msgs::Point point;
		point.x = oPoint.x();
		point.y = oPoint.y();
		point.z = oPoint.z();
		oVolumeMarker.points.push_back(point);
	}

	oOutputVolume.markers.push_back(oVolumeMarker);

    // make ray
    oVolumeMarker.id = index + 1 + 2 * iOffset;
    oVolumeMarker.type = visualization_msgs::Marker::LINE_STRIP;
    oVolumeMarker.scale.x = 0.1;
    oVolumeMarker.color.a = 1.0;
    oVolumeMarker.points.clear();
    geometry_msgs::Point point;
    point.x = m_vStartPoint.x();
    point.y = m_vStartPoint.y();
    point.z = m_vStartPoint.z();
    oVolumeMarker.points.push_back(point);
    point.x = m_vEndPoint.x();
    point.y = m_vEndPoint.y();
    point.z = m_vEndPoint.z();
    oVolumeMarker.points.push_back(point);
    
    oOutputVolume.markers.push_back(oVolumeMarker);
}


RayUpdater RayUpdater::instance;

/**
 * @brief ray_cast method to update volume
 * @param oLidarPos lidar center pos as ray start pos
 * @param vDepthMeasurementCloud one frame point cloud, and its center is oLidarPos
 * @param oVolume the volume to be updated
 * @param bKeepVoxel wether to keep the voxel that considered to be dynamic
*/
void RayUpdater::RayFusion(
    pcl::PointNormal oLidarPos, 
    pcl::PointCloud<pcl::PointNormal>& vDepthMeasurementCloud,
    VolumeBase& oVolume,
    bool bKeepVoxel)
{
    // pcl::PointCloud<pcl::PointNormal> vLocalCloud;
    // m_oPcOperation.TranslatePointCloudToLocal(oLidarPos, vDepthMeasurementCloud, vLocalCloud);

    if(vDepthMeasurementCloud.size() <= 0) return;

    HashBlock* pHashBlock = dynamic_cast<HashBlock*>(&oVolume);
    if(pHashBlock == nullptr) {
        std::cout << output::format_red << "[updater/RayUpdater] The volume type is not HashBlock!" << output::format_white << std::endl;
        return;
    }

    HashPos oNextHashPos;
    visualization_msgs::MarkerArray oOutputBlocks;
    std::vector<HashPos> vHashPosList;
    for(int i = 0; i < vDepthMeasurementCloud.size(); ++i) {
        RayCaster oRayCaster(oLidarPos.getVector3fMap(), vDepthMeasurementCloud[i].getVector3fMap(), pHashBlock->m_vVoxelSize);
        while(oRayCaster.GetNextHashPos(oNextHashPos)) {
            vHashPosList.push_back(oNextHashPos);
        }
    }
    std::cout << output::format_purple 
        << "Cloud Size: " << vDepthMeasurementCloud.size() 
        << " | Voxel Num: " << vHashPosList.size() << output::format_white << std::endl;
    // RayCaster oRayCaster(oLidarPos.getVector3fMap(), vDepthMeasurementCloud[0].getVector3fMap(), pHashBlock->m_vBlockSize);
    // while(oRayCaster.GetNextHashPos(oNextHashPos)) {
    //     vHashPosList.push_back(oNextHashPos);
    // }
    // oRayCaster.MakeRayDebugMarker(vHashPosList, oOutputBlocks, 0);
    // RayCaster oRayCaster_(oLidarPos.getVector3fMap(), vDepthMeasurementCloud[0].getVector3fMap(), pHashBlock->m_vVoxelSize);
    // vHashPosList.clear();
    // while(oRayCaster_.GetNextHashPos(oNextHashPos)) {
    //     vHashPosList.push_back(oNextHashPos);
    // }
    // oRayCaster_.MakeRayDebugMarker(vHashPosList, oOutputBlocks, 1);

    m_oRpManager.PublishMarkerArray(oOutputBlocks, "/ray_cast_debug");

}