#ifndef ROS_PUBLISH_MANAGER
#define ROS_PUBLISH_MANAGER

#include <ros/ros.h>
#include <ros/publisher.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <unordered_set>

#include "HashPos.h"


class RosPublishManager {

//singleton
private:
    static RosPublishManager instance;
    RosPublishManager(){}
    RosPublishManager(RosPublishManager&)=delete;
public:
    static RosPublishManager& GetInstance() {
        return instance;
    }

    typedef std::unordered_set<HashPos, HashFunc> HashPosSet;

private:
    bool PublishCheck(const std::string& sTopicName) {
        return m_vPublishers[sTopicName].getNumSubscribers() > 0;
    }
    static void BlockSetMaker(const HashPosSet& vBlockList, const Eigen::Vector3f& vBlockSize, visualization_msgs::MarkerArray& oOutputVolume, int iIdOffset = 0); 

public:
    template<class T>
    void PublishPointCloud(
        const pcl::PointCloud<T> & vCloud, 
        const std::vector<float> & vFeatures, 
        const std::string & sTopicName,
        const int iQueueSize = 1
    );

    void PublishMarkerArray(
        visualization_msgs::MarkerArray & oMarkerArray,
        const std::string & sTopicName,
        const std::function<void(visualization_msgs::MarkerArray&)>& funcMakeMarker = [](visualization_msgs::MarkerArray&){},
        const int iQueueSize = 1
    );

    void PublishNormalPoints(
        const pcl::PointCloud<pcl::PointNormal> & vCloud,
        const std::string & sTopicName,
        const int iQueueSize = 1
    );

    void PublishBlockSet(const HashPosSet& vBlockSet, const Eigen::Vector3f& vBlockSize, const std::string & sTopicName, const int iQueueSize = 1);

    void HSVToRGB(float H, float S, float V, float& R, float& G, float& B);
    void HSVToRGB(float H, float S, float V, uint8_t& R, uint8_t& G, uint8_t& B);

private:
    std::unordered_map<std::string, ros::Publisher> m_vPublishers;
public:
    std::string m_sFrameId;
    ros::NodeHandle* m_pNodeHandle;
};

#endif