#include "Poisson.h"

#include <pcl/surface/poisson.h>

/*************************************************
Function: Poisson
Description: constrcution function for Poisson class
Calls: all member functions
Called By: main function of project
Input: nodeHandle - a private ros node class
*************************************************/
Poisson::Poisson(ros::NodeHandle & nodeHandle) {

	// read parameters from launch file
	ReadLaunchParams(nodeHandle);

	// receive point cloud with normals
	m_oInPointCloud = nodeHandle.subscribe(m_sInPointCloudTopic, 1, &Poisson::HandlePointClouds, this);

	// publish poisson reconstructed mesh
	m_oMeshPublisher = nodeHandle.advertise<visualization_msgs::Marker>(m_sMeshPublishTopic, 1);
}

/*************************************************
Function: ~Poisson
Description: deconstrcution function for Poisson class (NONE CODE)
*************************************************/
Poisson::~Poisson() { 

}

/*************************************************
Function: ReadLaunchParams
Description: read the parameter value from ros launch file (poisson.launch)
Calls: all member functions
Called By: main function of project
Input: nodeHandle - a private ros node class
Output: the individual parameter value for system
*************************************************/
bool Poisson::ReadLaunchParams(ros::NodeHandle & nodeHandle) {

 	//input point cloud topic
	nodeHandle.param("cloud_in_topic", m_sInPointCloudTopic, std::string("/cloud_points"));

 	//output file name
 	nodeHandle.param("file_outputpath", m_sOutputFileHead, std::string("./"));

 	//publish mesh topic
	nodeHandle.param("mesh_out_topic", m_sMeshPublishTopic, std::string("/poisson_mesh"));

	//publish mesh id
	nodeHandle.param("mesh_out_id", m_sMeshPublishId, std::string("map"));

	//poisson octree depth
	nodeHandle.param("poisson_depth", m_dPossionDepth, 9);

	return true;
}

void Poisson::HandlePointClouds(const sensor_msgs::PointCloud2 & oFusedMap) {
	
	//transform rosmsg to pcl format
	pcl::PointCloud<pcl::PointNormal>::Ptr pPointCloudReceived(new pcl::PointCloud<pcl::PointNormal>);
	pcl::fromROSMsg(oFusedMap, *pPointCloudReceived);

	//output and check point cloud size
	std::cout << "[Poisson out] Received point cloud size: " << pPointCloudReceived->size() << std::endl;
	if(pPointCloudReceived->size() < 10000) {

		ROS_WARN("%s", "Too sparse to reconstruction.");
		return;
	}

	//poission reconstruction
	pcl::Poisson<pcl::PointNormal> oPoisson;
	oPoisson.setDepth(m_dPossionDepth);
	oPoisson.setInputCloud(pPointCloudReceived);
	pcl::PolygonMesh oMeshResult;
	oPoisson.reconstruct(oMeshResult);

	//delet point with low support
	pcl::PointCloud<pcl::PointXYZI> vMeshCloud;
	pcl::fromPCLPointCloud2(oMeshResult.cloud, vMeshCloud);
	for(int i = 0; i < vMeshCloud.size(); ++i) {
		std::cout << vMeshCloud.points[i].data_c[3] << " ";
	}
	std::cout << std::endl;

	//publish mesh
	PublishMeshes(oMeshResult);
}

/*************************************************
Function: PublishMeshes
Description: publish meshes (mainly used for display and test)
Input: vCloud - a polygon mesh to be published
*************************************************/
void Poisson::PublishMeshes(const pcl::PolygonMesh & oMeshModel){
  	
	pcl::PointCloud<pcl::PointXYZ> vPublishCloud;
	pcl::fromPCLPointCloud2(oMeshModel.cloud, vPublishCloud);

  	//new a visual message
	visualization_msgs::Marker oMeshMsgs;
	
	//define header of message
	oMeshMsgs.header.frame_id = m_sMeshPublishId;
	oMeshMsgs.header.stamp = ros::Time::now();
	oMeshMsgs.type = visualization_msgs::Marker::TRIANGLE_LIST;
	oMeshMsgs.action = visualization_msgs::Marker::ADD;

	oMeshMsgs.scale.x = 1.0;
	oMeshMsgs.scale.y = 1.0;
	oMeshMsgs.scale.z = 1.0;

	oMeshMsgs.pose.position.x = 0.0;
	oMeshMsgs.pose.position.y = 0.0;
	oMeshMsgs.pose.position.z = 0.0;

	oMeshMsgs.pose.orientation.x = 0.0;
	oMeshMsgs.pose.orientation.y = 0.0;
	oMeshMsgs.pose.orientation.z = 0.0;
	oMeshMsgs.pose.orientation.w = 1.0;

	std_msgs::ColorRGBA color;
	color.a = 1;
	color.r = 0.2;
	color.g = 1.0;
	color.b = 0.2;
    oMeshMsgs.color = color;
	
	//for each face
	for (int i = 0; i != oMeshModel.polygons.size(); ++i){

		//for each face vertex id
		for (int j = 0; j != oMeshModel.polygons[i].vertices.size(); ++j){

			//vertex id in each sector
			int iVertexIdx =  oMeshModel.polygons[i].vertices[j];

			//temp point
    		geometry_msgs::Point oPTemp;
        	oPTemp.x = vPublishCloud.points[iVertexIdx].x;
        	oPTemp.y = vPublishCloud.points[iVertexIdx].y;
        	oPTemp.z = vPublishCloud.points[iVertexIdx].z;
       		oMeshMsgs.points.push_back(oPTemp);

		}//end k

	}//end j

	m_oMeshPublisher.publish(oMeshMsgs);

}
