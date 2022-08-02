#ifndef __POISSON_H__
#define __POISSON_H__

#include <string>
#include <ctime>
#include <iostream>
#include <cmath>

//ros related
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>

//pcl related
#include "pcl_ros/transforms.h"  

#include <pcl/io/pcd_io.h>         
#include <pcl/io/ply_io.h>         
#include <pcl/point_types.h> 
#include <pcl_conversions/pcl_conversions.h>

class Poisson{

public:

      Poisson(ros::NodeHandle & nodeHandle);

      ~Poisson();

      bool ReadLaunchParams(ros::NodeHandle & nodeHandle);

      void HandlePointClouds(const sensor_msgs::PointCloud2 & fusedMap);

      void PublishMeshes(const pcl::PolygonMesh & oMeshModel);

private:

      /***** launch parameters *****/
      std::string m_sInPointCloudTopic;
      std::string m_sOutputFileHead;
      std::string m_sMeshPublishTopic;
      std::string m_sMeshPublishId;       //frame id of msg: marker

      int m_dPossionDepth;

      /***** ros communication *****/
      ros::Subscriber m_oInPointCloud;
      ros::Publisher  m_oMeshPublisher;

      /***** containers *****/
      pcl::PointCloud<pcl::PointNormal> m_vMapPCN;
};




#endif











