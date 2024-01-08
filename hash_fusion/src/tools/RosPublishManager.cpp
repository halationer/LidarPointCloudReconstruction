#include "RosPublishManager.h"

#include "tools/OutputUtils.h"
#include "volume/DistanceIoVolume.h"

#include <iostream>
#include <pcl_conversions/pcl_conversions.h>

RosPublishManager RosPublishManager::instance;

template<class T>
void RosPublishManager::PublishPointCloud(
    const pcl::PointCloud<T> & vCloud, 
    const std::vector<float> & vFeatures, 
    const std::string & sTopicName,
    const int iQueueSize) {
    
    // check topic
    if(m_vPublishers.count(sTopicName) == 0)
    {
        m_vPublishers[sTopicName] = m_pNodeHandle->advertise<sensor_msgs::PointCloud2>(sTopicName, iQueueSize, true);
        m_vPublishers[sTopicName].publish(sensor_msgs::PointCloud2());
    }
    if(!PublishCheck(sTopicName)) return;

    //get colors
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pColorClouds(new pcl::PointCloud<pcl::PointXYZRGB>);

    //to each point
    for (int i = 0; i < vCloud.points.size() && i < vFeatures.size(); ++i){

        pcl::PointXYZRGB oColorP;
        oColorP.x = vCloud.points[i].x;
        oColorP.y = vCloud.points[i].y;
        oColorP.z = vCloud.points[i].z;

        float H = abs(vFeatures[i]) * 360.0f;
        float S = vFeatures[i] < 0.0f ? 0.0f : 1.0f;
        float V = vFeatures[i] < 0.0f ? 0.5f : 1.0f;

        HSVToRGB(H, S, V, oColorP.r, oColorP.g, oColorP.b);

        pColorClouds->points.push_back(oColorP);
    }

    pColorClouds->width = 1;

    pColorClouds->height = vCloud.points.size();

    //convert to pc2 message
    sensor_msgs::PointCloud2 vCloudData;

    pcl::toROSMsg(*pColorClouds, vCloudData);

    //other informations
    vCloudData.header.frame_id = m_sFrameId;

    vCloudData.header.stamp = ros::Time::now();

    //publish
    m_vPublishers[sTopicName].publish(vCloudData);
}
template void RosPublishManager::PublishPointCloud(const pcl::PointCloud<pcl::PointNormal> & vCloud, const std::vector<float> & vFeatures, const std::string & sTopicName, const int iQueueSize);
template void RosPublishManager::PublishPointCloud(const pcl::PointCloud<pcl::PointXYZ> & vCloud, const std::vector<float> & vFeatures, const std::string & sTopicName, const int iQueueSize);
template void RosPublishManager::PublishPointCloud(const pcl::PointCloud<pcl::PointXYZI> & vCloud, const std::vector<float> & vFeatures, const std::string & sTopicName, const int iQueueSize);
template void RosPublishManager::PublishPointCloud(const pcl::PointCloud<pcl::DistanceIoVoxel> & vCloud, const std::vector<float> & vFeatures, const std::string & sTopicName, const int iQueueSize);


void RosPublishManager::PublishMarkerArray(
    visualization_msgs::MarkerArray & oMarkerArray,
    const std::string & sTopicName,
    const std::function<void(visualization_msgs::MarkerArray &)>& funcMakeMarker,
    const int iQueueSize) {

    if(m_vPublishers.count(sTopicName) == 0)
    {
        m_vPublishers[sTopicName] = m_pNodeHandle->advertise<visualization_msgs::MarkerArray>(sTopicName, iQueueSize, true);
        m_vPublishers[sTopicName].publish(visualization_msgs::MarkerArray());
    }
    if(!PublishCheck(sTopicName)) return;
    funcMakeMarker(oMarkerArray);
    m_vPublishers[sTopicName].publish(oMarkerArray);
}

void RosPublishManager::PublishBlockSet(const HashPosSet& vBlockSet, const Eigen::Vector3f& vBlockSize, const std::string & sTopicName, const int iQueueSize) {
    
    visualization_msgs::MarkerArray oOutputBlocks;
    PublishMarkerArray(oOutputBlocks, sTopicName, [&](visualization_msgs::MarkerArray& oMarkerArray){
        std::cout << output::format_purple << " Block Num: " << vBlockSet.size()  << output::format_white << std::endl;
        RosPublishManager::BlockSetMaker(vBlockSet, vBlockSize, oMarkerArray);
    }, iQueueSize);
}

void RosPublishManager::PublishVoxelSet(const HashPosDic& vVoxelSet, const Eigen::Vector3f& vVoxelSize, const std::string & sTopicName) {
    
    visualization_msgs::MarkerArray oOutputVoxels;
    PublishMarkerArray(oOutputVoxels, sTopicName, [&](visualization_msgs::MarkerArray& oMarkerArray){
        std::cout << output::format_purple << " Voxel Num: " << vVoxelSet.size()  << output::format_white << std::endl;
        RosPublishManager::VoxelSetMaker(vVoxelSet, vVoxelSize, oMarkerArray);
    });
}

void RosPublishManager::PublishBoundingBoxList(const std::vector<tools::BoundingBox> vBoundingBoxList, const std::string & sTopicName, const int iIdOffset) {

    visualization_msgs::MarkerArray oOutputLines;
    PublishMarkerArray(oOutputLines, sTopicName, [&](visualization_msgs::MarkerArray& oMarkerArray){
        std::cout << output::format_purple << " Cluster Num: " << vBoundingBoxList.size()  << output::format_white << std::endl;
        RosPublishManager::BoundingBoxMaker(vBoundingBoxList, oMarkerArray, iIdOffset);
    });
}

void RosPublishManager::PublishNormalPoints(
    pcl::PointCloud<pcl::PointNormal> & vCloud,
    const std::string & sTopicName,
    const std::function<void(pcl::PointCloud<pcl::PointNormal>&)>& funcMakeCloud,
    const int iQueueSize
) {
    // check topic
    if(m_vPublishers.count(sTopicName) == 0)
    {
        m_vPublishers[sTopicName] = m_pNodeHandle->advertise<visualization_msgs::MarkerArray>(sTopicName, iQueueSize, true);
        m_vPublishers[sTopicName].publish(visualization_msgs::MarkerArray());
    }
    if(!PublishCheck(sTopicName)) return;

    funcMakeCloud(vCloud);

    visualization_msgs::MarkerArray oOutputPoints;
    constexpr int index = 1e5;

    // make points
	visualization_msgs::Marker oPointMarker;
	oPointMarker.header.frame_id = "map";
	oPointMarker.header.stamp = ros::Time::now();
	oPointMarker.type = visualization_msgs::Marker::POINTS;
	oPointMarker.action = visualization_msgs::Marker::MODIFY;
	oPointMarker.id = index;

    oPointMarker.scale.x = 0.1;
    oPointMarker.scale.y = 0.1;

	oPointMarker.pose.position.x = 0.0;
	oPointMarker.pose.position.y = 0.0;
	oPointMarker.pose.position.z = 0.0;

	oPointMarker.pose.orientation.x = 0.0;
	oPointMarker.pose.orientation.y = 0.0;
	oPointMarker.pose.orientation.z = 0.0;
	oPointMarker.pose.orientation.w = 1.0;

	oPointMarker.color.a = 1;
	oPointMarker.color.r = 0.8;
	oPointMarker.color.g = 0.2;
	oPointMarker.color.b = 0.8;

    for(const pcl::PointNormal & oPoint : vCloud) {
        
		geometry_msgs::Point point;
		point.x = oPoint.x;
		point.y = oPoint.y;
		point.z = oPoint.z;
		oPointMarker.points.push_back(point);
    }

	oOutputPoints.markers.push_back(oPointMarker);

    // make normals
	visualization_msgs::Marker oNormalMarker;
	oNormalMarker.header.frame_id = "map";
	oNormalMarker.header.stamp = ros::Time::now();
	oNormalMarker.type = visualization_msgs::Marker::LINE_LIST;
	oNormalMarker.action = visualization_msgs::Marker::MODIFY;
	oNormalMarker.id = index + 1;

    oNormalMarker.scale.x = 0.05;

	oNormalMarker.pose.position.x = 0.0;
	oNormalMarker.pose.position.y = 0.0;
	oNormalMarker.pose.position.z = 0.0;

	oNormalMarker.pose.orientation.x = 0.0;
	oNormalMarker.pose.orientation.y = 0.0;
	oNormalMarker.pose.orientation.z = 0.0;
	oNormalMarker.pose.orientation.w = 1.0;

	oNormalMarker.color.a = 1;
	oNormalMarker.color.r = 0.8;
	oNormalMarker.color.g = 0.2;
	oNormalMarker.color.b = 0.2;

    for(const pcl::PointNormal & oPoint : vCloud) {
        
		geometry_msgs::Point point;
		point.x = oPoint.x;
		point.y = oPoint.y;
		point.z = oPoint.z;
		oNormalMarker.points.push_back(point);

        point.x += oPoint.normal_x;
        point.y += oPoint.normal_y;
        point.z += oPoint.normal_z;
        oNormalMarker.points.push_back(point);
    }

	oOutputPoints.markers.push_back(oNormalMarker);

    m_vPublishers[sTopicName].publish(oOutputPoints);
}

void RosPublishManager::HSVToRGB(float H, float S, float V, float& R, float& G, float& B) {

    float C = V * S;
    int FixFactor = int(H / 60) & 1;
    float X = C * ((FixFactor ? -1 : 1) * (int(H) % 60) / 60.0 + FixFactor); // X 值的变化随H波动，锯齿状0-1之间周期变化
    float m = V - C;
    switch(int(H) / 60)
    {
        case 1:		R = X;	G = C;	B = 0; break;	// 60  <= H < 120
        case 2:		R = 0;	G = C;	B = X; break;	// 120 <= H < 180
        case 3:		R = 0;	G = X;	B = C; break;	// 180 <= H < 240
        case 4:		R = X;	G = 0;	B = C; break;	// 240 <= H < 300
        case 5:
        case 6:		R = C;	G = 0;	B = X; break;	// 300 <= H < 360
        default:	R = C;	G = X;	B = 0; 		// 0   <= H < 60 or outlier
    }
    R += m;
    G += m;
    B += m;
}

void RosPublishManager::HSVToRGB(float H, float S, float V, uint8_t& R, uint8_t& G, uint8_t& B) {

    float R_, G_, B_;
    HSVToRGB(H, S, V, R_, G_, B_);
    R = R_ * 255;
    G = G_ * 255;
    B = B_ * 255;
}

void RosPublishManager::BlockSetMaker(const HashPosSet& vBlockList, const Eigen::Vector3f& vBlockSize, visualization_msgs::MarkerArray& oOutputVolume, int iIdOffset) {

    constexpr int index = 2e4;

    // make blocks
	visualization_msgs::Marker oVolumeMarker;
	oVolumeMarker.header.frame_id = "map";
	oVolumeMarker.header.stamp = ros::Time::now();
	oVolumeMarker.type = visualization_msgs::Marker::CUBE_LIST;
	oVolumeMarker.action = visualization_msgs::Marker::MODIFY;
	oVolumeMarker.id = index + 2 * iIdOffset; 

	oVolumeMarker.scale.x = vBlockSize.x();
	oVolumeMarker.scale.y = vBlockSize.y();
	oVolumeMarker.scale.z = vBlockSize.z();

	oVolumeMarker.pose.position.x = 0.0;
	oVolumeMarker.pose.position.y = 0.0;
	oVolumeMarker.pose.position.z = 0.0;

	oVolumeMarker.pose.orientation.x = 0.0;
	oVolumeMarker.pose.orientation.y = 0.0;
	oVolumeMarker.pose.orientation.z = 0.0;
	oVolumeMarker.pose.orientation.w = 1.0;

	oVolumeMarker.color.a = std::min(0.2 * (iIdOffset+1), 1.0);
	oVolumeMarker.color.r = 0.8;
	oVolumeMarker.color.g = 0.2;
	oVolumeMarker.color.b = 0.2;

	for(const HashPos& oPos : vBlockList) {

		Eigen::Vector3f oPoint(oPos.x, oPos.y, oPos.z);
        oPoint = oPoint.cwiseProduct(vBlockSize);
        oPoint += 0.5 * vBlockSize;

		geometry_msgs::Point point;
		point.x = oPoint.x();
		point.y = oPoint.y();
		point.z = oPoint.z();
		oVolumeMarker.points.push_back(point);
	}

	oOutputVolume.markers.push_back(oVolumeMarker);
}

void RosPublishManager::VoxelSetMaker(const HashPosDic& vVoxelList, const Eigen::Vector3f& vVoxelSize, visualization_msgs::MarkerArray& oOutputVolume) {
 
    constexpr int index = 3e4;

    for(int iIdOffset = 0; iIdOffset < 2; ++iIdOffset) {

        // make blocks
        visualization_msgs::Marker oVolumeMarker;
        oVolumeMarker.header.frame_id = "map";
        oVolumeMarker.header.stamp = ros::Time::now();
        oVolumeMarker.type = visualization_msgs::Marker::CUBE_LIST;
        oVolumeMarker.action = visualization_msgs::Marker::MODIFY;
        oVolumeMarker.id = index + 2 * iIdOffset; 

        oVolumeMarker.scale.x = vVoxelSize.x();
        oVolumeMarker.scale.y = vVoxelSize.y();
        oVolumeMarker.scale.z = vVoxelSize.z();

        oVolumeMarker.pose.position.x = 0.0;
        oVolumeMarker.pose.position.y = 0.0;
        oVolumeMarker.pose.position.z = 0.0;

        oVolumeMarker.pose.orientation.x = 0.0;
        oVolumeMarker.pose.orientation.y = 0.0;
        oVolumeMarker.pose.orientation.z = 0.0;
        oVolumeMarker.pose.orientation.w = 1.0;

        oVolumeMarker.color.a = std::min(0.3 * (iIdOffset + 1), 1.0);
        oVolumeMarker.color.r = 0.8 - 0.6 * iIdOffset;
        oVolumeMarker.color.g = 0.2;
        oVolumeMarker.color.b = 0.2;

        for(auto&& [oPos,influence] : vVoxelList) {

            if(influence != iIdOffset) continue;

            Eigen::Vector3f oPoint(oPos.x, oPos.y, oPos.z);
            oPoint = oPoint.cwiseProduct(vVoxelSize);
            oPoint += 0.5 * vVoxelSize;

            geometry_msgs::Point point;
            point.x = oPoint.x();
            point.y = oPoint.y();
            point.z = oPoint.z();
            oVolumeMarker.points.push_back(point);
        }

        oOutputVolume.markers.push_back(oVolumeMarker);
    }
   
}

static const Eigen::Vector3i line_list[] = {
    {0, 1, 2}, {4, 1, 2}, {4, 1, 2}, {4, 5, 2}, {4, 5, 2}, {0, 5, 2}, {0, 5, 2}, {0, 1, 2},
    {0, 1, 2}, {0, 1, 6}, {4, 1, 2}, {4, 1, 6}, {4, 5, 2}, {4, 5, 6}, {0, 5, 2}, {0, 5, 6},
    {0, 1, 6}, {4, 1, 6}, {4, 1, 6}, {4, 5, 6}, {4, 5, 6}, {0, 5, 6}, {0, 5, 6}, {0, 1, 6},
};

void RosPublishManager::BoundingBoxMaker(const std::vector<tools::BoundingBox>& vBoundingBoxList, visualization_msgs::MarkerArray& oOutputLines, int iIdOffset) {
 
    constexpr int index = 4e4;

    // clear boxes out of time
    visualization_msgs::Marker oLineMarker;
    oLineMarker.header.frame_id = "map";
    oLineMarker.header.stamp = ros::Time::now();
    oLineMarker.type = visualization_msgs::Marker::LINE_LIST;
    oLineMarker.action = visualization_msgs::Marker::DELETEALL;
    oOutputLines.markers.push_back(oLineMarker);

    // make boxes
    for(int i = 0; i < vBoundingBoxList.size(); ++i) {

        visualization_msgs::Marker oLineMarker;
        oLineMarker.header.frame_id = "map";
        oLineMarker.header.stamp = ros::Time::now();
        oLineMarker.type = visualization_msgs::Marker::LINE_LIST;
        oLineMarker.action = visualization_msgs::Marker::MODIFY;
        oLineMarker.id = index + iIdOffset + i; 

        oLineMarker.scale.x = 0.05;
        oLineMarker.pose.orientation.w = 1.0;

        oLineMarker.color.a = 1.0f;
        oLineMarker.color.r = 1.0f;
        oLineMarker.color.g = 0.0f;
        oLineMarker.color.b = 0.0f;

        auto& oBoundingBox = vBoundingBoxList[i];

        for(auto&& oLineIndex : line_list) {
            
            geometry_msgs::Point oPoint;
            oPoint.x = oBoundingBox.data[oLineIndex.x()]; 
            oPoint.y = oBoundingBox.data[oLineIndex.y()]; 
            oPoint.z = oBoundingBox.data[oLineIndex.z()];

            oLineMarker.points.push_back(oPoint);
        }

        oOutputLines.markers.push_back(oLineMarker);
    }
}