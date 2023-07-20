#ifndef ROS_PUBLISH_MANAGER
#define ROS_PUBLISH_MANAGER

#include <ros/ros.h>
#include <ros/publisher.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


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


public:
    template<class T>
    void PublishPointCloud(
        const pcl::PointCloud<T> & vCloud, 
        const std::vector<float> & vFeatures, 
        const std::string & sTopicName,
        const int iQueueSize = 1
    );

    void PublishMarkerArray(
        const visualization_msgs::MarkerArray & oMarkerArray,
        const std::string & sTopicName,
        const int iQueueSize = 1
    );

    void HSVToRGB(float H, float S, float V, float& R, float& G, float& B);
    void HSVToRGB(float H, float S, float V, uint8_t& R, uint8_t& G, uint8_t& B);

private:
    std::unordered_map<std::string, ros::Publisher> m_vPublishers;
public:
    std::string m_sFrameId;
    ros::NodeHandle* m_pNodeHandle;
};

#endif