#include "RosPublishManager.h"
#include <pcl_conversions/pcl_conversions.h>

RosPublishManager RosPublishManager::instance;

template<class T>
void RosPublishManager::PublishPointCloud(
    const pcl::PointCloud<T> & vCloud, 
    const std::vector<float> & vFeatures, 
    const std::string & sTopicName,
    const int iQueueSize) {
    
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
    if(m_vPublishers.count(sTopicName) == 0)
    {
        m_vPublishers[sTopicName] = m_pNodeHandle->advertise<sensor_msgs::PointCloud2>(sTopicName, iQueueSize, true);
    }
    m_vPublishers[sTopicName].publish(vCloudData);
}
template void RosPublishManager::PublishPointCloud(
    const pcl::PointCloud<pcl::PointNormal> & vCloud, 
    const std::vector<float> & vFeatures, 
    const std::string & sTopicName,
    const int iQueueSize);


void RosPublishManager::PublishMarkerArray(
    const visualization_msgs::MarkerArray & oMarkerArray,
    const std::string & sTopicName,
    const int iQueueSize) {

    if(m_vPublishers.count(sTopicName) == 0)
    {
        m_vPublishers[sTopicName] = m_pNodeHandle->advertise<visualization_msgs::MarkerArray>(sTopicName, iQueueSize, true);
    }
    m_vPublishers[sTopicName].publish(oMarkerArray);
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