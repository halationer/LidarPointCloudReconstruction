#include "FramesFusion.h"

#include <sstream>
#include <random>
#include <algorithm>
#include <cmath>
#include <exception>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

/*************************************************
Function: FramesFusion
Description: constrcution function for FramesFusion class
Calls: all member functions
Called By: main function of project
Table Accessed: none
Table Updated: none
Input: node - a ros node class
     nodeHandle - a private ros node class
*************************************************/
FramesFusion::FramesFusion(ros::NodeHandle & node,
                       ros::NodeHandle & nodeHandle): 
					   m_oGlobalNode(node), m_oNodeHandle(nodeHandle), m_iOdomCount(0), 
					   m_dAverageReconstructTime(0), m_dMaxReconstructTime(0),  m_iReconstructFrameNum(0),
					   m_dAverageFusionTime(0), m_dMaxFusionTime(0), m_iFusionFrameNum(0), m_OdomLoopRate(1) {

	//read parameters
	ReadLaunchParams(nodeHandle);

	//***subscriber related*** 
	//subscribe (hear) the point cloud topic 
	m_oCloudSuber = nodeHandle.subscribe(m_sInCloudTopic, 5, &FramesFusion::HandlePointClouds, this);

	//subscribe (hear) the odometry information (trajectory)
	m_oOdomSuber = nodeHandle.subscribe(
		m_sInOdomTopic, 
		1,
		m_bAsyncReconstruction ? &FramesFusion::HandleTrajectoryThread : &FramesFusion::HandleTrajectory, 
		this
	);

	//***publisher related*** 
	//publish point cloud after processing
	m_oCloudPublisher = nodeHandle.advertise<sensor_msgs::PointCloud2>(m_sOutCloudTopic, 1, true);

  	//publish polygon constructed from one frame point cloud
	m_oMeshPublisher = nodeHandle.advertise<visualization_msgs::MarkerArray>(m_sOutMeshTopic, 1, true);
}

void FramesFusion::LazyLoading() {
	
    std::cout << "Load Fixed Resolution Fusion..." << std::endl;
}

/*************************************************
Function: ~FramesFusion
Description: deconstrcution function for FramesFusion class
Calls: all member functions
Called By: main function of project
Table Accessed: none
Table Updated: none
Input: none
Output: a file storing the point clouds with correct normal for accurate reconstruction
*************************************************/

FramesFusion::~FramesFusion() {

	std::cout << std::format_purple
	<< "Fusion frame numbers: " << m_iFusionFrameNum << std::endl
	<< "Average fusion per frame: " << m_dAverageFusionTime / m_iFusionFrameNum << "ms;\t"
	<< "Max fusion time: " << m_dMaxFusionTime << "ms"
	<< std::format_white << std::endl;

	std::cout << std::format_blue
	<< "Reconstruct frame numbers: " << m_iReconstructFrameNum << std::endl
	<< "Average recontime per frame: " << m_dAverageReconstructTime / m_iReconstructFrameNum << "ms;\t"
	<< "Max frame time: " << m_dMaxReconstructTime << "ms" << std::endl
	<< "Final point nums: " << m_vMapPCN.size() + m_vMapPCNAdded.size() + m_vMapPCNTrueAdded.size()
	<< std::format_white << std::endl;


    //output to the screen
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << "********************************************************************************" << std::endl;

	//output point clouds with computed normals to the files when the node logs out
	if(m_bOutputFiles) {
		
		//define ouput ply file name
		std::stringstream sOutPCNormalFileName;
		sOutPCNormalFileName << m_sFileHead << "Map_PCNormal.ply";

		std::cout << "Please do not force closing the programe, the process is writing output PLY file." << std::endl;
		std::cout << "It may take times (Writing 500M file takes about 20 seconds in usual)." << std::endl;
		std::cout << std::format_purple << "The output file is " << sOutPCNormalFileName.str() << std::format_white << std::endl;

		if(m_bUseAdditionalPoints) {
		
			m_vMapPCN += m_vMapPCNAdded;
			m_vMapPCN += m_vMapPCNTrueAdded;
		}
		
		auto vAllPoints = AllCloud(m_vMapPCN);

		// constexpr int union_set_time = 20;
		// for(int i = 0; i < union_set_time; ++i) {
		// 	if(m_bUseUnionSetConnection) {
		// 		++m_oVoxeler.m_iFrameCount;
		// 		m_oVoxeler.RebuildUnionSet();
		// 		m_oVoxeler.UpdateUnionConflict();
		// 	}
		// }

		HashVoxeler::HashVolume vVolumeCopy;
		// m_oVoxeler.GetRecentMaxConnectVolume(vVolumeCopy, m_iKeepTime);
		m_oVoxeler.GetStaticVolume(vVolumeCopy);


		pcl::PointCloud<pcl::PointXYZINormal> pc;
		for(int i = 0; i < vAllPoints->size(); ++i) {

			std::cout << "All ready process points: " << i+1 << "/" << vAllPoints->size() << "\r";

			pcl::PointNormal& point = (*vAllPoints)[i];
			HashPos pos;
			m_oVoxeler.PointBelongVoxelPos(point, pos);
			if(!vVolumeCopy.count(pos)) continue;

			pcl::PointXYZINormal new_point;
			new_point.x = point.x;
			new_point.y = point.y;
			new_point.z = point.z;
			new_point.normal_x = point.normal_x;
			new_point.normal_y = point.normal_y;
			new_point.normal_z = point.normal_z;
			
			// point confidence (impacted by depth & support & conflict)
			new_point.intensity = point.data_n[3]; 
			
			// maybe use to save gauss distance, but now don't make scence
			// new_point.curvature = point.data_c[3];

			pc.push_back(new_point);
		}
		std::cout << "\nSaving file ..." << std::endl;
		pcl::io::savePLYFileBinary(sOutPCNormalFileName.str(), pc);

		/** 
			if the point cloud has too many points, ros may not wait it to save.
			to solve this problem, change the file: /opt/ros/melodic/lib/python2.7/dist-packages/roslaunch/nodeprocess.py
				DEFAULT_TIMEOUT_SIGINT = 15.0  ->  DEFAULT_TIMEOUT_SIGINT = 60.0 
		**/

		std::cout << "Output is complete! The process will be automatically terminated. Thank you for waiting. " << std::endl;
	}

	system("rm -r /tmp/lidar_recon_temp/");

}



/*************************************************
Function: ReadLaunchParams
Description: read the parameter value from ros launch file (mapping.launch)
Calls: all member functions
Called By: main function of project
Table Accessed: none
Table Updated: none
Input: nodeHandle - a private ros node class
Output: the individual parameter value for system
Return: none
Others: none
*************************************************/

bool FramesFusion::ReadLaunchParams(ros::NodeHandle & nodeHandle) {

 	//output file name
 	nodeHandle.param("file_output_path", m_sFileHead, std::string());
	m_bOutputFiles = !m_sFileHead.empty();

	if(m_bOutputFiles) {

		std::cout << std::format_blue << "mf_output_path:=" << m_sFileHead << std::format_white << std::endl;
		if(m_sFileHead.back() != '/') m_sFileHead += "/";
		std::stringstream sOutputCommand;
		sOutputCommand << "mkdir -p " << m_sFileHead;
		system(sOutputCommand.str().c_str());
		m_sFileHead += "mf_";
	}


 	//input point cloud topic
	nodeHandle.param("cloud_in_topic", m_sInCloudTopic, std::string("/cloud_points"));

	//input odom topic
  	nodeHandle.param("odom_in_topic", m_sInOdomTopic, std::string("/odometry/filtered"));

	//input odom topic
	nodeHandle.param("cloud_out_topic", m_sOutCloudTopic, std::string("/processed_clouds"));

	//input point cloud topic
	nodeHandle.param("outcloud_tf_id", m_sOutCloudTFId, std::string("map"));

	//input odom topic
	nodeHandle.param("polygon_out_topic", m_sOutMeshTopic, std::string("/surrounding_meshes"));

	//input point cloud topic
	nodeHandle.param("polygon_tf_id", m_sOutMeshTFId, std::string("map"));

	//nearbt lengths
	nodeHandle.param("voxel_total_size", m_fNearLengths, 20.0f);
	m_fNearLengths = m_fNearLengths/2.0f;

	//point cloud sampling number
	nodeHandle.param("sample_pcframe_num", m_iFrameSmpNum, 1);

	//point cloud sampling number
	nodeHandle.param("sample_inputpoints_num", m_iSampleInPNum, 1);

	//nearby mesh update period in second
	nodeHandle.param("mesh_update_period", m_fNearMeshPeriod, 2.0f);

	//side length of cube (voxel)
  	float fCubeSize;
  	nodeHandle.param("voxel_cube_size", fCubeSize, 0.5f);
 	m_oVoxelRes.x = fCubeSize;
	m_oVoxelRes.y = fCubeSize;
	m_oVoxelRes.z = fCubeSize;
	m_oVoxeler.SetResolution(m_oVoxelRes);

	int eStrategyType;
  	nodeHandle.param("strategy_type", eStrategyType, static_cast<int>(eEmptyStrategy));
	m_oVoxeler.SetStrategy(static_cast<vus>(eStrategyType));

	//use surfel fusion?
	nodeHandle.param("use_surfel_fusion", m_bSurfelFusion, true);
	// use additional points for reconstruction?
	nodeHandle.param("additional_points", m_bUseAdditionalPoints, true);

	nodeHandle.param("async_reconstruct", m_bAsyncReconstruction, true);
	nodeHandle.param("use_union_set", m_bUseUnionSetConnection, true);
	nodeHandle.param("only_max_union_set", m_bOnlyMaxUnionSet, true);

	//count processed point cloud frame
	m_iPCFrameCount = 0;

	//true indicates the file has not been generated
	m_bOutPCFileFlag = true;

	m_OdomLoopRate = ros::Rate(1 / m_fNearMeshPeriod);

	// sdf meshing params
	nodeHandle.param("keep_time", m_iKeepTime, 30);
	nodeHandle.param("conv_dim", m_iConvDim, 3);
	nodeHandle.param("conv_add_point_ref", m_iConvAddPointNumRef, 5); 
	nodeHandle.param("conv_distance_ref", m_fConvFusionDistanceRef1, 0.95f);
	// m_pSdf = new SignedDistance(m_iKeepTime, m_iConvDim, m_iConvAddPointNumRef, m_fConvFusionDistanceRef1);
	m_oVoxeler.m_iMaxRecentKeep = max(500, m_iKeepTime);

	return true;

}


/*************************************************
Function: PublishPointCloud
Description: publish point clouds (mainly used for display and test)
Calls: none
Called By: ComputeConfidence()
Table Accessed: none
Table Updated: none
Input: vCloud - a point clouds to be published
Output: none
Return: none
Others: none
*************************************************/
template<class T>
void FramesFusion::PublishPointCloud(const pcl::PointCloud<T> & vCloud){
  
	//publish obstacle points
	sensor_msgs::PointCloud2 vCloudData;

	pcl::toROSMsg(vCloud, vCloudData);

	vCloudData.header.frame_id = m_sOutCloudTFId;

	vCloudData.header.stamp = ros::Time::now();

	m_oCloudPublisher.publish(vCloudData);

}
// Make instances of the defined template function
// template void FramesFusion::PublishPointCloud(const pcl::PointCloud<pcl::PointXYZ>&);
template void FramesFusion::PublishPointCloud(const pcl::PointCloud<pcl::PointNormal>&);

/*************************************************
Function: PublishPointCloud
Description: publish point clouds (mainly used for display and test)
Calls: none
Called By: ComputeConfidence()
Table Accessed: none
Table Updated: none
Input: vCloud - a point clouds to be published
Output: none
Return: none
Others: none
*************************************************/
void FramesFusion::PublishPointCloud(const pcl::PointCloud<pcl::PointXYZ> & vCloud, const std::vector<float> & vFeatures){
  
	//get colors
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr  pColorClouds (new pcl::PointCloud<pcl::PointXYZRGB>);

	//to each point
	for (int i = 0; i <  vCloud.points.size(); ++i){

		pcl::PointXYZRGB oColorP;
		oColorP.x = vCloud.points[i].x;
		oColorP.y = vCloud.points[i].y;
		oColorP.z = vCloud.points[i].z;

		oColorP.r =  (1.0-vFeatures[i])*255.0f;
		oColorP.g =  vFeatures[i]*255.0f;
		oColorP.b =  (1.0-vFeatures[i])*255.0f; 
		pColorClouds->points.push_back(oColorP);
	}

    pColorClouds->width = 1;

    pColorClouds->height = vCloud.points.size();

    //convert to pc2 message
	sensor_msgs::PointCloud2 vCloudData;

	pcl::toROSMsg(*pColorClouds, vCloudData);

	//other informations
	vCloudData.header.frame_id = m_sOutCloudTFId;

	vCloudData.header.stamp = ros::Time::now();

	//publish
	m_oCloudPublisher.publish(vCloudData);

}

void HSVToRGB(float H, float S, float V, float& R, float& G, float& B) {

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

void HSVToRGB(float H, float S, float V, uint8_t& R, uint8_t& G, uint8_t& B) {

	float R_, G_, B_;
	HSVToRGB(H, S, V, R_, G_, B_);
	R = R_ * 255;
	G = G_ * 255;
	B = B_ * 255;
}

/*************************************************
Function: PublishPointCloud
Description: publish point clouds (mainly used for display and test)
Calls: none
Called By: ComputeConfidence()
Table Accessed: none
Table Updated: none
Input: vCloud - a point clouds to be published
       vFeatures - use different color to show the features, should in [0.0, 1.0]
	   sTopicName - topic to output
*************************************************/
template<class T>
void FramesFusion::PublishPointCloud(const pcl::PointCloud<T> & vCloud, const std::vector<float> & vFeatures, const std::string sTopicName) {
	
	//get colors
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr  pColorClouds(new pcl::PointCloud<pcl::PointXYZRGB>);

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
	vCloudData.header.frame_id = m_sOutCloudTFId;

	vCloudData.header.stamp = ros::Time::now();

	//publish
	if(m_vDebugPublishers.count(sTopicName) == 0)
	{
		m_vDebugPublishers[sTopicName] = m_oNodeHandle.advertise<sensor_msgs::PointCloud2>(sTopicName, 1, true);
	}
	m_vDebugPublishers[sTopicName].publish(vCloudData);
}
template void FramesFusion::PublishPointCloud(const pcl::PointCloud<pcl::PointNormal>&, const std::vector<float>&, const std::string);
template void FramesFusion::PublishPointCloud(const pcl::PointCloud<pcl::PointXYZ>&, const std::vector<float>&, const std::string);

void InitMeshMsg(visualization_msgs::Marker& oMeshMsgs, string frame_id, int id, float r, float g, float b) {
	
	oMeshMsgs.header.frame_id = frame_id;
	oMeshMsgs.header.stamp = ros::Time::now();
	oMeshMsgs.type = visualization_msgs::Marker::TRIANGLE_LIST;
	oMeshMsgs.action = visualization_msgs::Marker::ADD;
	oMeshMsgs.id = id; 

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

	oMeshMsgs.color.a = 1.0;
	oMeshMsgs.color.r = r;
	oMeshMsgs.color.g = g;
	oMeshMsgs.color.b = b;
}

/*************************************************
Function: PublishPointCloud
Description: publish point clouds (mainly used for display and test)
Calls: none
Called By: ComputeConfidence()
Table Accessed: none
Table Updated: none
Input: vCloud - a point clouds to be published
Output: none
Return: none
Others: none
*************************************************/
void FramesFusion::PublishMeshs(const pcl::PolygonMesh & oMeshModel){
  	
	pcl::PointCloud<pcl::PointXYZ> vPublishCloud;
	pcl::fromPCLPointCloud2(oMeshModel.cloud, vPublishCloud);

  	//new a visual message
	visualization_msgs::MarkerArray oMeshMsgList;
	visualization_msgs::Marker oMeshMsgs, oMeshDynamic, oMeshAdded, oMeshFused;
	
	//define header of message
	InitMeshMsg(oMeshMsgs, 		m_sOutMeshTFId, 0, 1. , 0.95, 0.2);
	InitMeshMsg(oMeshDynamic, 	m_sOutMeshTFId, 1, 1. , 0.1, 0.1);
	// InitMeshMsg(oMeshAdded, 	m_sOutMeshTFId, 2, 1. , 0.9, 0.2);
	// InitMeshMsg(oMeshFused,		m_sOutMeshTFId, 3, 0.9, 1. , 0.2);

	// dynamic
	// InitMeshMsg(oMeshAdded, 	m_sOutMeshTFId, 2, 1. , 0.95, 0.2);
	// InitMeshMsg(oMeshFused,		m_sOutMeshTFId, 3, 1. , 0.95, 0.2);

	// add
	InitMeshMsg(oMeshAdded, 	m_sOutMeshTFId, 2, 0.4, 0.6, 1. );
	InitMeshMsg(oMeshFused,		m_sOutMeshTFId, 3, 1. , 0.95, 0.2);

	//for each face
	for (int i = 0; i != oMeshModel.polygons.size(); ++i){

		//for each face vertex id
		for (int j = 0; j != 3; ++j){

			//vertex id in each sector
			int iVertexIdx =  oMeshModel.polygons[i].vertices[j];

			//temp point
    		geometry_msgs::Point oPTemp;
        	oPTemp.x = vPublishCloud.points[iVertexIdx].x;
        	oPTemp.y = vPublishCloud.points[iVertexIdx].y;
        	oPTemp.z = vPublishCloud.points[iVertexIdx].z;

			uint32_t mesh_type = oMeshModel.polygons[i].vertices.back();
			if(mesh_type >= __INT_MAX__) {
				// oMeshAdded.points.push_back(oPTemp);
				oMeshMsgs.points.push_back(oPTemp);
			}
			else if(mesh_type >= 0x3fffffff) {
				oMeshFused.points.push_back(oPTemp);
			}
			else if(mesh_type < 1) {
			// else if(mesh_type >= 1) {
       			oMeshMsgs.points.push_back(oPTemp);
			}
			else {
				oMeshDynamic.points.push_back(oPTemp);
			}

		}//end k

	}//end j

	oMeshMsgList.markers.push_back(oMeshMsgs);
	oMeshMsgList.markers.push_back(oMeshDynamic);
	oMeshMsgList.markers.push_back(oMeshAdded);
	oMeshMsgList.markers.push_back(oMeshFused);

	m_oMeshPublisher.publish(oMeshMsgList);
}

 

/*************************************************
Function: EuclideanDistance
Description: calculate the Euclidean distance between two points
Calls: none
Called By: NearbyClouds
Table Accessed: none
Table Updated: none
Input: oBasedP - point a
           oTargetP - point b
Output: distance between two points
Return: sqrt(fDis)
Others: none
*************************************************/
 float FramesFusion::EuclideanDistance(const pcl::PointXYZ & oBasedP, const pcl::PointNormal & oTargetP){

 	 float fDis = (oBasedP.x - oTargetP.x)*(oBasedP.x - oTargetP.x)
              	+ (oBasedP.y - oTargetP.y)*(oBasedP.y - oTargetP.y)
              	+ (oBasedP.z - oTargetP.z)*(oBasedP.z - oTargetP.z);

 	 return sqrt(fDis);

}


/*************************************************
Function: NearbyClouds
Description: Get the neighboring points based on the given distance
Calls: none
Called By: NearbyClouds
Table Accessed: none
Table Updated: none
Input: pRawCloud - input point clouds
          oBasedP - a query point
          fLength - the neighboring distance
Output:  pNearCloud - the neighboring points near the query point
Return: none
Others: none
*************************************************/
void FramesFusion::NearbyClouds(const pcl::PointCloud<pcl::PointNormal> & pRawCloud, const pcl::PointXYZ & oBasedP, pcl::PointCloud<pcl::PointNormal> & pNearCloud, float fLength){

	// pNearCloud.clear();
			
	for (int i = 0; i != pRawCloud.size(); ++i){
		
		if (EuclideanDistance(oBasedP, pRawCloud.at(i)) <= fLength) {
			pNearCloud.push_back(pRawCloud.at(i));
			pNearCloud.points.rbegin()->data_c[3] = i; //record the rawcloud index in data_c[3]
		}

	}

}

void FramesFusion::NearbyClouds(CloudVector & pRawCloud, const pcl::PointXYZ & oBasedP, pcl::PointCloud<pcl::PointNormal> & pNearCloud, float fLength) {
	
	for (int i = 0; i != pRawCloud.size(); ++i){
		
		if (EuclideanDistance(oBasedP, pRawCloud.at(i)) <= fLength) {
			pNearCloud.push_back(pRawCloud.at(i));
			pNearCloud.points.rbegin()->data_c[3] = i; //record the rawcloud index in data_c[3]
		}

	}
}


/*************************************************
Function: ExtractNearbyClouds
Description: Get the nearby point clouds and delete the geted point in rawCloud
Calls: none
Called By: NearbyClouds
Table Accessed: none
Table Updated: none
Input: pRawCloud - input point clouds
          oBasedP - a query point
          fLength - the neighboring distance
Output:  pNearCloud - the neighboring points near the query point
Return: none
Others: none
*************************************************/
void FramesFusion::ExtractNearbyClouds(pcl::PointCloud<pcl::PointNormal> & pRawCloud, const pcl::PointXYZ & oBasedP, pcl::PointCloud<pcl::PointNormal> & pNearCloud, float fLength){

	// pNearCloud.clear();
	
	for (int i = 0; i < pRawCloud.points.size();){
		
		if (EuclideanDistance(oBasedP, pRawCloud.points[i]) <= fLength) {
			pNearCloud.push_back(pRawCloud.points[i]);
			pNearCloud.points.rbegin()->data_c[3] = i; //record the rawcloud index in data_c[3]

			//删除已经识别的顶点
			swap(pRawCloud.points[i], pRawCloud.points.back());
			pRawCloud.points.pop_back();
		}
		else {
			++i;
		}
	}

	pRawCloud.width = static_cast<uint32_t> (pRawCloud.points.size ()); // 模仿pointcloud中的erase函数

};

void FramesFusion::ExtractNearbyClouds(CloudVector & pRawCloud, const pcl::PointXYZ & oBasedP, pcl::PointCloud<pcl::PointNormal> & pNearCloud, float fLength){

	for (int i = 0; i < pRawCloud.size();){
		
		if (EuclideanDistance(oBasedP, pRawCloud.at(i)) <= fLength) {
			pNearCloud.push_back(pRawCloud.at(i));
			pNearCloud.points.rbegin()->data_c[3] = i; //record the rawcloud index in data_c[3]

			//删除已经识别的顶点
			pRawCloud.erase(i);
		}
		else {
			++i;
		}
	}
};

void FramesFusion::FusionNormalBackToPoint(const pcl::PointCloud<pcl::PointNormal>& pNearCloud, pcl::PointCloud<pcl::PointNormal> & pRawCloud, int offset, int point_num) {

	MeshOperation m;
	for(int i = offset; i != offset + point_num; ++i) {
		pcl::PointNormal& related_point = pRawCloud.at(pNearCloud.at(i).data_c[3]);
		related_point.normal_x = related_point.normal_x + 0.3 * pNearCloud.at(i).normal_x;
		related_point.normal_y = related_point.normal_y + 0.3 * pNearCloud.at(i).normal_y;
		related_point.normal_z = related_point.normal_z + 0.3 * pNearCloud.at(i).normal_z;
		m.VectorNormalization(related_point.normal_x, related_point.normal_y, related_point.normal_z);
	}
}

void FramesFusion::FusionNormalBackToPoint(const pcl::PointCloud<pcl::PointNormal>& pNearCloud, CloudVector & pRawCloud, int offset, int point_num) {

	MeshOperation m;
	for(int i = offset; i != offset + point_num; ++i) {
		pcl::PointNormal& related_point = pRawCloud.at(pNearCloud.at(i).data_c[3]);
		related_point.normal_x = related_point.normal_x + 0.3 * pNearCloud.at(i).normal_x;
		related_point.normal_y = related_point.normal_y + 0.3 * pNearCloud.at(i).normal_y;
		related_point.normal_z = related_point.normal_z + 0.3 * pNearCloud.at(i).normal_z;
		m.VectorNormalization(related_point.normal_x, related_point.normal_y, related_point.normal_z);
	}
}

/*************************************************
Function: SurroundModeling
Description: Build surface models based on surrounding points with computed normals
Calls: none
Called By: 
Table Accessed: none
Table Updated: none
Input: pRawCloud - input point clouds
          oBasedP - a query point
          fLength - the neighboring distance
Output:  pNearCloud - the neighboring points near the query point
Return: none
Others: none
*************************************************/
void FramesFusion::SurroundModeling(const pcl::PointXYZ & oBasedP, pcl::PolygonMesh & oCBModel, const int iFrameId){

	//output mesh
	//oCBModel.cloud.clear();
	oCBModel.polygons.clear();

	//get the target points for construction
	pcl::PointCloud<pcl::PointNormal>::Ptr pProcessedCloud(new pcl::PointCloud<pcl::PointNormal>);

	//based on received point cloud and normal vector
	//get the target point cloud to be modeled
	// m_mNewPointMutex.lock();
	// *pProcessedCloud += m_vNewPoints;
	// m_vNewPoints.clear();
	// m_mNewPointMutex.unlock();

	// 点云抽稀
	int new_end = 0, point_num = pProcessedCloud->size();
	if(m_iSampleInPNum > 1 && point_num > 1e5) {
		int past_point_num = point_num;
		std::cout << "sample: " << m_iSampleInPNum << " | " << pProcessedCloud->size();
		for(int i = 0; i < pProcessedCloud->size(); i+=m_iSampleInPNum) {
			swap(pProcessedCloud->at(i), pProcessedCloud->at(new_end++));
			if(i < past_point_num) point_num = new_end;
		}
		pProcessedCloud->erase(pProcessedCloud->begin() + new_end, pProcessedCloud->end());
		std::cout << " | " << pProcessedCloud->size() << std::endl;
	}

	std::vector<float> temp_feature(pProcessedCloud->size());
	PublishPointCloud(*pProcessedCloud, temp_feature, "/temp_near_cloud");


	///* output nearby cloud
	if(m_bOutputFiles) {

		pcl::PointCloud<pcl::PointXYZINormal> pc;
		for(auto point : *pProcessedCloud) {
			pcl::PointXYZINormal new_point;
			new_point.x = point.x;
			new_point.y = point.y;
			new_point.z = point.z;
			new_point.normal_x = point.normal_x;
			new_point.normal_y = point.normal_y;
			new_point.normal_z = point.normal_z;
			
			// point confidence (impacted by depth & support & conflict)
			new_point.intensity = point.data_n[3];
			
			// maybe use to save gauss distance, but now don't make scence
			// new_point.curvature = point.data_c[3];

			pc.push_back(new_point);
		}
		std::stringstream filename;
		filename << m_sFileHead << std::setw(4) << std::setfill('0') << iFrameId << "_pc.ply";
		pcl::io::savePLYFileASCII(filename.str(), pc);
	}
	//*/

	// update voxel point-normal-sdf
	m_oVoxeler.VoxelizePointsAndFusion(*pProcessedCloud);

	//******voxelization********
	//using signed distance
	SignedDistance oSDer;

	//compute signed distance based on centroids and its normals within voxels
	std::unordered_map<HashPos, float, HashFunc> vSignedDis = oSDer.ConvedGlance(m_oVoxeler);

	// debug
	// pcl::PointCloud<pcl::PointNormal> vVolumeCloud;
	// std::vector<float> vVoxelValue;
	// for(auto && [oPos, oPoint] : m_oVoxeler.m_vVolume) {
	// 	vVolumeCloud.push_back(oPoint);
	// 	vVoxelValue.push_back(oPoint.data_n[3] > 1 ? 0.3f : 0.0f);
	// }
	// PublishPointCloud(vVolumeCloud, vVoxelValue, "/temp_voxel_cloud");

	// pcl::PointCloud<pcl::PointXYZ> vSignedDisCloud;
	// std::vector<float> vSignedDisValue;
	// for(auto && [oPos, fSDF] : vSignedDis) {
	// 	vSignedDisCloud.push_back(m_oVoxeler.HashPosTo3DPos(oPos));
	// 	vSignedDisValue.push_back(fSDF > 0 ? 0.5f : 0.0f);
	// } 
	// PublishPointCloud(vSignedDisCloud, vSignedDisValue, "/temp_sdf_cloud");

	//传回去
	// m_mPCNMutex.lock();
	// FusionNormalBackToPoint(*pProcessedCloud, m_vMapPCN, 0, point_num);
	// FusionNormalBackToPoint(*pProcessedCloud, m_vMapPCNAdded, point_num + 1, pProcessedCloud->size() - point_num);
	// m_vMapPCN += *pProcessedCloud;
	// m_mPCNMutex.unlock();

	//record non-empty voxels
	// std::vector<bool> vVoxelStatus;
	// oVoxeler.OutputNonEmptyVoxels(vVoxelStatus);

	//clear accelerated data
	// oVoxeler.ClearMiddleData();

	//******construction********
	//marching cuber
	CIsoSurface<float> oMarchingCuber;
	//get surface mesh based on voxel related algorithm
	oMarchingCuber.GenerateSurface(vSignedDis, oSDer.m_vVolumeCopy, 0,
			                       m_oVoxeler.m_oVoxelLength.x, m_oVoxeler.m_oVoxelLength.y, m_oVoxeler.m_oVoxelLength.z);

	//move the mesh position to the actual starting point
	pcl::PointCloud<pcl::PointXYZ>::Ptr pMCResultCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZ oOffset(0, 0, 0);
	oMarchingCuber.OutputMesh(oOffset, oCBModel, pMCResultCloud);
	
	///* output result mesh
	if(m_bOutputFiles) {

		std::stringstream sOutputPath;
		sOutputPath << m_sFileHead << std::setw(4) << std::setfill('0') << iFrameId << "_mesh.ply";
		pcl::io::savePLYFileBinary(sOutputPath.str(), oCBModel);
	}
	//*/
}


/*************************************************
Function: SlideModeling
Description: Build surface models based on new points that received from ros
Called By: 
Input:	m_vNewPoints - new points that received from ros
Output: oResultMesh - result mesh of the surface model
		m_vMapPCN - the fused points is added to m_vMapPCN
*************************************************/
void FramesFusion::SlideModeling(pcl::PolygonMesh & oResultMesh, const int iFrameId) {
	
	//clear mesh
	//oResultMesh.cloud.clear();
	oResultMesh.polygons.clear();

	//******make mesh********
	//check connection
	if(m_bUseUnionSetConnection) {
		m_oVoxeler.RebuildUnionSet();
		m_oVoxeler.UpdateUnionConflict();

		// output unionset result
		visualization_msgs::MarkerArray union_set_marker;
		m_oVoxeler.DrawUnionSet(union_set_marker);
		std::string sTopicName = "/union_set";
		if(m_vDebugPublishers.count(sTopicName) == 0)
		{
			m_vDebugPublishers[sTopicName] = m_oNodeHandle.advertise<visualization_msgs::MarkerArray>(sTopicName, 1, true);
		}
		m_vDebugPublishers[sTopicName].publish(union_set_marker);
	}

	//using signed distance
	SignedDistance oSDer(m_iKeepTime, m_iConvDim, m_iConvAddPointNumRef, m_fConvFusionDistanceRef1);
	//compute signed distance based on centroids and its normals within voxels
	std::unordered_map<HashPos, float, HashFunc> vSignedDis;
	vSignedDis = m_bUseUnionSetConnection && m_bOnlyMaxUnionSet ? oSDer.ConvedGlanceOnlyMaxUnion(m_oVoxeler) : oSDer.ConvedGlance(m_oVoxeler);


	//marching cuber
	CIsoSurface<float> oMarchingCuber;
	//get surface mesh based on voxel related algorithm
	oMarchingCuber.GenerateSurface(vSignedDis, oSDer.m_vVolumeCopy, 0, m_oVoxeler.m_oVoxelLength.x, m_oVoxeler.m_oVoxelLength.y, m_oVoxeler.m_oVoxelLength.z);

	//move the mesh position to the actual starting point
	pcl::PointCloud<pcl::PointXYZ>::Ptr pMCResultCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZ oOffset(0, 0, 0);
	oMarchingCuber.OutputMesh(oOffset, oResultMesh, pMCResultCloud);
	
	///* output result mesh
	if(m_bOutputFiles) {

		std::stringstream sOutputPath;
		sOutputPath << m_sFileHead << std::setw(4) << std::setfill('0') << iFrameId << "_mesh.ply";
		auto oCopyMesh = oResultMesh;
		for(auto & polygon : oCopyMesh.polygons)
			polygon.vertices.pop_back();
		pcl::io::savePLYFileBinary(sOutputPath.str(), oCopyMesh);
	}
	//*/
}

/*************************************************
Function: HandlePointClouds
Description: a callback function in below:
node.subscribe(m_sLaserTopic, 5, &GroundExtraction::HandlePointClouds, this);
Calls: CheckTruthPoint
Called By: TransformLaserInOdom, which is the construction function
Table Accessed: none
Table Updated: none
Input: rawpoint, a 3d point with pcl point type
Output: a point clouds are almost the same with raw point clouds but only their timestamp values are modified
Return: none
Others: none
*************************************************/
void FramesFusion::HandlePointClouds(const sensor_msgs::PointCloud2 & vLaserData)
{

	//a point clouds in PCL type
	pcl::PointCloud<pcl::PointNormal>::Ptr pFramePN(new pcl::PointCloud<pcl::PointNormal>);
	//message from ROS type to PCL type
	pcl::fromROSMsg(vLaserData, *pFramePN);

	// Surfel Cloud Fusion. Done.
	// 提取中心点
	pcl::PointNormal oViewPoint;
	oViewPoint.x = pFramePN->back().x;
	oViewPoint.y = pFramePN->back().y;
	oViewPoint.z = pFramePN->back().z;

	if(pFramePN->back().curvature == -1) { //标识码
		pFramePN->erase(pFramePN->end()-1);
	}

	// 标识码，单帧输出两种点云：真实点云(true) | 猜测填充点云(false), 目前不对猜测点云有任何处理
	if(pFramePN->is_dense == false) return;

	// start update frame point cloud
	++m_iFusionFrameNum;

	if(m_bSurfelFusion) {

		std::cout << "fusion start \t";

		struct timeval start;
		gettimeofday(&start, NULL);

		SurfelFusionQuick(oViewPoint, *pFramePN);

		struct timeval end;
		gettimeofday(&end,NULL);
		double frames_fusion_time = (end.tv_sec - start.tv_sec) * 1000.0 +(end.tv_usec - start.tv_usec) * 0.001;
		
		std::cout << std::format_purple 
			<< "The No. " << m_iFusionFrameNum 
			<< ";\tframes_fusion_time: " << frames_fusion_time << "ms" 
			<< std::format_white;
		m_dAverageFusionTime += frames_fusion_time;
		m_dMaxFusionTime = frames_fusion_time > m_dMaxFusionTime ? frames_fusion_time : m_dMaxFusionTime;
	}

	// 点云抽稀
	// int new_end = 0, point_num = pProcessedCloud->size();
	// if(m_iSampleInPNum > 1 && point_num > 1e5) {
	// 	int past_point_num = point_num;
	// 	std::cout << "sample: " << m_iSampleInPNum << " | " << pProcessedCloud->size();
	// 	for(int i = 0; i < pProcessedCloud->size(); i+=m_iSampleInPNum) {
	// 		swap(pProcessedCloud->at(i), pProcessedCloud->at(new_end++));
	// 		if(i < past_point_num) point_num = new_end;
	// 	}
	// 	pProcessedCloud->erase(pProcessedCloud->begin() + new_end, pProcessedCloud->end());
	// 	std::cout << " | " << pProcessedCloud->size() << std::endl;
	// }

	// std::vector<float> temp_feature(pProcessedCloud->size());
	// PublishPointCloud(*pProcessedCloud, temp_feature, "/temp_near_cloud");

	//merge one frame data
	struct timeval start;
	gettimeofday(&start, NULL);

	UpdateOneFrame(oViewPoint, *pFramePN);

	struct timeval end;
	gettimeofday(&end,NULL);
	double voxelize_time = (end.tv_sec - start.tv_sec) * 1000.0 +(end.tv_usec - start.tv_usec) * 0.001;
	std::cout << std::format_blue << ";\tvoxelize_time: " << voxelize_time << "ms" << std::format_white << std::endl;


	// output point cloud with (depth & view) confidence
	// TODO: 或许可以使用多线程发布数据？
	if(m_bSurfelFusion && false) {
		
		pcl::PointCloud<pcl::PointNormal> temp;
		temp += m_vMapPCN;
		temp += m_vMapPCNAdded;
		vector<float> confidence(temp.size());
		constexpr float confidence_scalar = 0.3f;
		for(int i = 0; i < m_vMapPCN.size(); ++i) {
			confidence[i] = m_vMapPCN.at(i).data_n[3] * confidence_scalar;
			if(confidence[i] > confidence_scalar) confidence[i] = confidence_scalar;
		}
		for(int i = 0; i < m_vMapPCNAdded.size(); ++i) {
			confidence[i + m_vMapPCN.size()] = m_vMapPCNAdded.at(i).data_n[3] * confidence_scalar;
			if(confidence[i + m_vMapPCN.size()] > confidence_scalar) confidence[i + m_vMapPCN.size()] = confidence_scalar;
		}
		PublishPointCloud(temp, confidence, "/all_cloud_confidence");
	}

	return;
}


void FramesFusion::UpdateOneFrame(const pcl::PointNormal& oViewPoint, pcl::PointCloud<pcl::PointNormal>& vFilteredMeasurementCloud) {

	m_oVoxeler.VoxelizePointsAndFusion(vFilteredMeasurementCloud);
	m_vMapPCN += vFilteredMeasurementCloud;
}
  

/*=======================================
SamplePoints
Input: vCloud - the raw point clouds
vNewCloud - the sampled point clouds
iSampleNum - interval number or the maximum number
bIntervalSamp - interval number model (true) or the maximum number model (false)
Output: vNewCloud - the sampled point clouds
Function: Sample a point clouds. Note that tthe given maximum total number guarantes the number of sampled point is smaller than it
However, it is not guaranteed that the number of sampled point could be equal to the given maximum total number
========================================*/
void FramesFusion::SamplePoints(const pcl::PointCloud<pcl::PointXYZ> & vCloud, pcl::PointCloud<pcl::PointXYZ> & vNewCloud, int iSampleNum, bool bIntervalSamp){

	vNewCloud.clear();

	//sample by interval number
	if (bIntervalSamp){

		for (int i = 0; i < vCloud.points.size(); i = i + iSampleNum)
			vNewCloud.push_back(vCloud.points[i]);

		//over the function and output	
		return;

	}//end if

	//Sampling according to the given maximum total number
	//get the interval point number - how muny points should be skipped
	int iInterval = ceil(float(vCloud.points.size()) / float(iSampleNum));
	//sample
	for (int i = 0; i < vCloud.points.size(); i = i + iInterval)
		vNewCloud.push_back(vCloud.points[i]);

	//output
	return;

}


/*************************************************
Function: HandleTrajectory
Description: a callback function in below:
m_oOdomSuber = node.subscribe(m_sOdomTopic, 5, &GroundExtraction::HandleTrajectory, this);
Calls: none
Called By: TransformLaserInOdom, which is the construction function
Table Accessed: none
Table Updated: none
Input: rawpoint, a 3d point with pcl point type
Output: a point clouds are almost the same with raw point clouds but only their timestamp values are modified
Return: none
Others: none
*************************************************/
void FramesFusion::HandleTrajectory(const nav_msgs::Odometry & oTrajectory)
{

	//a flag indicates whether to calculate this odom
	bool bComputeFlag = false;

	//count input frames
	if(!m_iOdomCount){

		//initialize the time of last calculation as beginning
		m_oLastModelingTime = oTrajectory.header.stamp;
		//need to compute
		bComputeFlag = true;

	}else{

		//compute the time difference
		ros::Duration oModelduration = oTrajectory.header.stamp - m_oLastModelingTime;

		//
		if(oModelduration.toSec() > m_fNearMeshPeriod)
			//
			bComputeFlag = true;

	}

	m_iOdomCount++;

	//if it is in the calculation period
	//it would end this function and wait for the next input	
	if(!bComputeFlag)
		return;
	
	//if need to be updated
	//get the newest information
	m_oLastModelingTime = oTrajectory.header.stamp;

	//save the position of trajectory
	Eigen::Vector3f oLidarPos;
	oLidarPos.x() = oTrajectory.pose.pose.position.x;
	oLidarPos.y() = oTrajectory.pose.pose.position.y;
	oLidarPos.z() = oTrajectory.pose.pose.position.z;
	m_oVoxeler.UpdateLidarCenter(oLidarPos);

	//get the reconstructed surfaces
	pcl::PolygonMesh oNearbyMeshes;

	// Mesh Generate
	clock_t start_time = clock();

	// if(!m_bSurfelFusion && m_bUseAdditionalPoints) 
	// 	SurroundModelingWithPointProcessing(oOdomPoint.oLocation, oNearbyMeshes, m_iReconstructFrameNum);
	// else 
	SlideModeling(oNearbyMeshes, m_iReconstructFrameNum);
	clock_t frames_fusion_time = 1000.0 * (clock() - start_time) / CLOCKS_PER_SEC;

	++m_iReconstructFrameNum;
	std::cout << std::format_blue 
		<< "The No. " << m_iReconstructFrameNum 
		<< ";\tframes_fusion_time: " << frames_fusion_time << "ms" 
		<< std::format_white << std::endl;
	m_dAverageReconstructTime += frames_fusion_time;
	m_dMaxReconstructTime = frames_fusion_time > m_dMaxReconstructTime ? frames_fusion_time : m_dMaxReconstructTime;

	//output the nearby surfaces
	PublishMeshs(oNearbyMeshes);

	// auto vAllCloud = AllCloud(m_vMapPCN);
	// std::cout << "### cloud_size: " << vAllCloud->size() << " ###" << std::endl; 
	// PublishPointCloud(*vAllCloud);
}

void FramesFusion::HandleTrajectoryThread(const nav_msgs::Odometry & oTrajectory) {

	// get odom
	bool bComputeFlag = false;
	if(!m_iOdomCount){
		m_oLastModelingTime = oTrajectory.header.stamp;
		bComputeFlag = true;
	}else{
		//compute the time difference
		ros::Duration oModelduration = oTrajectory.header.stamp - m_oLastModelingTime;
		if(oModelduration.toSec() > m_fNearMeshPeriod)
			bComputeFlag = true;
	}
	m_iOdomCount++;
	if(!bComputeFlag) return;
	m_oLastModelingTime = oTrajectory.header.stamp;
	
	Eigen::Vector3f oLidarPos;
	oLidarPos.x() = oTrajectory.pose.pose.position.x;
	oLidarPos.y() = oTrajectory.pose.pose.position.y;
	oLidarPos.z() = oTrajectory.pose.pose.position.z;
	m_oVoxeler.UpdateLidarCenter(oLidarPos);

	//if need to be updated
	//get the newest information
	int now_frame_num = m_iReconstructFrameNum++;

	auto ModelingFunction = [&, now_frame_num]() {

		std::cout << std::format_purple << "No. " << now_frame_num << " reconstruct start" << std::format_white << std::endl;

		//get the reconstructed surfaces
		pcl::PolygonMesh oResultMeshes;

		struct timeval start;
		gettimeofday(&start, NULL);

		// if(!m_bSurfelFusion && m_bUseAdditionalPoints) 
		// 	SurroundModelingWithPointProcessing(oOdomPoint.oLocation, oResultMeshes, now_frame_num);
		// else 
		SlideModeling(oResultMeshes, now_frame_num);

		struct timeval end;
		gettimeofday(&end,NULL);
		double frame_reconstruct_time = (end.tv_sec - start.tv_sec) * 1000.0 +(end.tv_usec - start.tv_usec) * 0.001;
		std::cout << std::format_blue 
			<< "The No. " << now_frame_num
			<< ";\tframes_reconstruct_time: " << frame_reconstruct_time << "ms" 
			<< ";\tgen_face_num: " << oResultMeshes.polygons.size()
			<< std::format_white << std::endl;
		m_dAverageReconstructTime += frame_reconstruct_time;
		m_dMaxReconstructTime = frame_reconstruct_time > m_dMaxReconstructTime ? frame_reconstruct_time : m_dMaxReconstructTime;

		//output the nearby surfaces
		PublishMeshs(oResultMeshes);
	};

	std::thread Modeling(ModelingFunction);

	Modeling.detach();
}


/*************************************************
Function: OutputScannedPCFile
Description: output scanned point clouds in a txt file
Calls: none
Called By: HandleTrajectory
Table Accessed: none
Table Updated: none
Input: vCloud - one frame scanning point cloud data
Output: a point cloud txt file
Return: none
Others: none
*************************************************/
void FramesFusion::OutputPCFile(const pcl::PointCloud<pcl::PointXYZ> & vCloud, bool bAllRecord){
  
    //generate a output file if possible
	if( m_bOutPCFileFlag || bAllRecord){

		m_sOutPCFileName.clear();
	    //set the current time stamp as a file name
		//m_sOutPCFileName << m_sFileHead << "PC_" << ros::Time::now() << ".txt"; 

		//set the count as a file name
		m_sOutPCFileName << m_sFileHead << "PC_" << m_iPCFrameCount << ".txt"; 

		m_bOutPCFileFlag = false;

        //print output file generation message
		std::cout << "[*] Attention, a point cloud recording file is created in " << m_sOutPCFileName.str() << std::endl;
	}

    //output
	m_oOutPCFile.open(m_sOutPCFileName.str(), std::ios::out | std::ios::app);

	//output in a txt file
	//the storage type of output file is x y z time frames 
    //record the point clouds
    for(int i = 0; i != vCloud.size(); ++i ){

        //output in a txt file
        //the storage type of output file is x y z time frames right/left_sensor
        m_oOutPCFile << vCloud.points[i].x << " "
                  << vCloud.points[i].y << " "
                  << vCloud.points[i].z << " " 
                  << m_iPCFrameCount << " " 
                  << std::endl;
    }//end for         

    m_oOutPCFile.close();

    //count new point cloud input (plus frame) 

}


/*************************************************
Function: OutputScannedPCFile
Description: output scanned point clouds in a txt file
Calls: none
Called By: HandleTrajectory
Table Accessed: none
Table Updated: none
Input: vCloud - one frame scanning point cloud data
Output: a point cloud txt file
Return: none
Others: none
*************************************************/
void FramesFusion::OutputPCFile(const pcl::PointCloud<pcl::PointXYZ> & vCloud, const std::vector<float> & vFeatures, bool bAllRecord){
  
    //generate a output file if possible
	if( m_bOutPCFileFlag || bAllRecord){

		m_sOutPCFileName.clear();
	    //set the current time stamp as a file name
		//m_sOutPCFileName << m_sFileHead << "PC_" << ros::Time::now() << ".txt"; 

		//set the count as a file name
		m_sOutPCFileName << m_sFileHead << "PC_" << m_iPCFrameCount << ".txt"; 

		m_bOutPCFileFlag = false;

        //print output file generation message
		std::cout << "[*] Attention, a point cloud recording file is created in " << m_sOutPCFileName.str() << std::endl;
	}

    //output
	m_oOutPCFile.open(m_sOutPCFileName.str(), std::ios::out | std::ios::app);

	//output in a txt file
	//the storage type of output file is x y z time frames 
    //record the point clouds
    for(int i = 0; i != vCloud.size(); ++i ){

        //output in a txt file
        //the storage type of output file is x y z time frames right/left_sensor
        m_oOutPCFile << vCloud.points[i].x << " "
                  << vCloud.points[i].y << " "
                  << vCloud.points[i].z << " " 
                  << vFeatures[i] << " "
                  << m_iPCFrameCount << " " 
                  << std::endl;
    }//end for         

    m_oOutPCFile.close();

    //count new point cloud input (plus frame) 

}


void FramesFusion::SurfelFusionCore(pcl::PointNormal oLidarPos, pcl::PointCloud<pcl::PointNormal>& vDepthMeasurementCloud, pcl::PointCloud<pcl::PointNormal>& vPointCloudBuffer) {

	constexpr int pitch_dim_expand = 4;
	constexpr int yaw_dim_expand = 4;
	constexpr int pitch_dim = 180 * pitch_dim_expand;
	constexpr int yaw_dim = 360 * yaw_dim_expand;
	
	constexpr double support_factor = 0.3;
	constexpr double tight_support_factor = 0.1;
	constexpr double normal_support_factor = 0.8;

	//计算点云每个点的角度，将其置于相机像素中 4ms - 8ms
	std::vector<std::vector<double>> depth_image(pitch_dim, std::vector<double>(yaw_dim));
	std::vector<std::vector<int>> depth_index(pitch_dim, std::vector<int>(yaw_dim));
	std::vector<bool> vCurrentFuseIndex(vDepthMeasurementCloud.size(), false); //因为只记录一个点，哪些点被覆盖需要记录

	// conf - 0.01 - 0.5
	// double min_depth = __INT_MAX__; 
	// int min_id = 0, max_id = 0;
	double max_depth = 0;
	for(int i = 0; i < vDepthMeasurementCloud.size(); ++i) {

		const pcl::PointNormal& oCurrentPoint = vDepthMeasurementCloud.at(i);
		Eigen::Vector3f oRefPoint(oCurrentPoint.x - oLidarPos.x, oCurrentPoint.y - oLidarPos.y, oCurrentPoint.z - oLidarPos.z);

		// 计算投影位置和深度
		double depth = oRefPoint.norm();
		if(depth < 1e-5) continue;
		max_depth = max_depth < depth ? depth : max_depth;

		// min_depth = min_depth > depth ? depth : min_depth;
		// if(min_depth == depth) min_id = i;
		// if(max_depth == depth) max_id = i;

		oRefPoint.normalize();
		double yaw = std::atan2((double)oRefPoint.y(), (double)oRefPoint.x()) / M_PI * 180.0;
		double pitch = std::asin((double)oRefPoint.z()) / M_PI * 180.0;

		// 将投影结果存储
		double row = (pitch + 90) * pitch_dim_expand;
		double col = (yaw   + 180) * yaw_dim_expand;
		if(int(row) < 0 || int(row) >= pitch_dim || int(col) < 0 || int(col) >= yaw_dim) continue;
		if(depth_image[int(row)][int(col)] > 0)
			vCurrentFuseIndex[depth_index[int(row)][int(col)]] = true;
		depth_image[int(row)][int(col)] = depth;
		depth_index[int(row)][int(col)] = i;

		// 根据surfel在subpixel的位置，添加到邻近的像素中去
		double local_row = row - int(row);
		double local_col = col - int(col);

		// 如果只扩展左右，不扩展上下如何？好像效果还可以，但是仍然不能完全解决单向素拉长导致的错误complict判断问题
		// /* 双向扩展
		if(local_col > 0.6) {
			// 右
			int temp_col = col + 1;
			if(temp_col >= yaw_dim) temp_col = 0;

			if(depth_image[int(row)][temp_col] > 0)
				vCurrentFuseIndex[depth_index[int(row)][temp_col]] = true;
			depth_image[int(row)][temp_col] = depth;
			depth_index[int(row)][temp_col] = i;
		}
		else if(local_col < 0.4) {
			// 左
			int temp_col = col - 1;
			if(temp_col < 0) temp_col = yaw_dim - 1;

			if(depth_image[int(row)][temp_col] > 0)
				vCurrentFuseIndex[depth_index[int(row)][temp_col]] = true;
			depth_image[int(row)][temp_col] = depth;
			depth_index[int(row)][temp_col] = i;
		}
		// */
	}	
	
	// std::cout << std::format_red << "min depth: " << min_depth << " | conf: " << vDepthMeasurementCloud[min_id].data_n[3] << "\t";
	// std::cout << std::format_red << "max depth: " << max_depth << " | conf: " << vDepthMeasurementCloud[max_id].data_n[3] << std::format_white << std::endl;

	// 将多帧重建结果点云拷贝到Buffer中，并筛选范围内的点
	pcl::PointCloud<pcl::PointNormal> vVolumeCloud;
	m_oVoxeler.GetVolumeCloud(vVolumeCloud);
	pcl::PointXYZ oBase(oLidarPos.x, oLidarPos.y, oLidarPos.z);
	NearbyClouds(vVolumeCloud, oBase, vPointCloudBuffer, max_depth);


	// /* 对于之前帧的所有点，对应位置建立匹配关系 60 - 150 ms
	std::vector<float> associated_feature(vPointCloudBuffer.size(), 0);

	for(int i = 0; i < vPointCloudBuffer.size(); ++i) {

		const pcl::PointNormal& oCurrentPoint = vPointCloudBuffer.at(i);
		Eigen::Vector3f oRefPoint(oCurrentPoint.x - oLidarPos.x, oCurrentPoint.y - oLidarPos.y, oCurrentPoint.z - oLidarPos.z);

		double depth = oRefPoint.norm();
		if(depth < 1e-5) continue;
		oRefPoint.normalize();
		double yaw = std::atan2((double)oRefPoint.y(), (double)oRefPoint.x()) / M_PI * 180.0;
		double pitch = std::asin((double)oRefPoint.z()) / M_PI * 180.0;
		
		int row = (pitch + 90) * pitch_dim_expand;
		int col = (yaw   + 180) * yaw_dim_expand;

		if(row < 0 || row >= pitch_dim || col < 0 || col >= yaw_dim) continue;

		int complict_count = 0;

		pcl::PointNormal* pSupportPoint = nullptr;
		pcl::PointNormal* pConflictPoint = nullptr;

		if(row < pitch_dim && col < yaw_dim && depth_image[row][col]) {

			pcl::PointNormal& oAssociatedPoint = vDepthMeasurementCloud.points[depth_index[row][col]];
			double dAssociatedDepth = depth_image[row][col];
			// 判断complict关系
			if(abs(depth - dAssociatedDepth) > support_factor && depth < dAssociatedDepth) { // 旧点的depth < 新点的depth
				
				++complict_count;
				pConflictPoint = &oAssociatedPoint;
			}
		}

		if(complict_count > 0) {

			//进一步精细的遮挡检查
			
			pcl::PointNormal& oOldPoint = vPointCloudBuffer.at(i);
			// pcl::PointNormal& oOldPoint = i < lidar_size ? m_vMapPCN[vPointCloudBuffer[i].data_c[3]] : m_vMapPCNAdded[vPointCloudBuffer[i].data_c[3]];
			float& fOldConfidence = oOldPoint.data_n[3];
			float& fConfidenceRecord = oOldPoint.data_c[0];
			pcl::PointNormal& oNewPoint = *pConflictPoint;
			float& fNewConfidence = oNewPoint.data_n[3];

			Eigen::Vector3f p1(oOldPoint.x - oLidarPos.x, oOldPoint.y - oLidarPos.y, oOldPoint.z - oLidarPos.z);
			Eigen::Vector3f p2(oNewPoint.x - oLidarPos.x, oNewPoint.y - oLidarPos.y, oNewPoint.z - oLidarPos.z);
			Eigen::Vector3f n2(oNewPoint.normal_x, oNewPoint.normal_y, oNewPoint.normal_z);
			// 给测量点加一个bais，减少错误判断
			p2 += n2 * tight_support_factor;

			Eigen::Vector3f vp2 = p2 / p2.norm();
			Eigen::Vector3f v12 = p2 - p1;

			// dynamic surfel radius, according to confidence
			float r = abs(n2.dot(vp2)) * 2 * fNewConfidence;
			// float r = abs(n2.dot(vp2)) * support_factor;
			Eigen::Vector3f vr = v12.dot(vp2) * vp2 - v12;
			float a = p2.norm() - v12.dot(vp2), b = p2.norm();

			if(vr.norm() < r * a / b && v12.dot(n2) < 0) {

				associated_feature[i] = 0.0f;

				// 记录减少的置信度
				fConfidenceRecord = fOldConfidence;
				fOldConfidence = - 0.8 * fNewConfidence;
				// if(fOldConfidence < 0.f) fOldConfidence = 0.f;
			}
			else associated_feature[i] = -1.f;
		}
		else {
			associated_feature[i] = -1.0f;
		}
	}	
	
	// output dynamic
	PublishPointCloud(vPointCloudBuffer, associated_feature, "/debug_associated_point");
}

/** 基于反投影的点云融合
   @param 	pcl::PointNormal 					oLidarPos - m_oCurrentViewPoint 雷达视点的位置
   @param 	pcl::PointCloud<pcl::PointNormal> 	vDepthMeasurementCloud - vDepthMeasurementCloud 新一帧的点云
   @param_	pcl::PointCloud<pcl::PointNormal> 	m_vMapPCN - 旧点云（源自于lidar测量）
   @param_	pcl::PointCloud<pcl::PointNormal> 	m_vMapPCNAdded - 新点云（源自于单帧重建补点）
*/
void FramesFusion::SurfelFusionQuick(pcl::PointNormal oLidarPos, pcl::PointCloud<pcl::PointNormal>& vDepthMeasurementCloud) {

	pcl::PointCloud<pcl::PointNormal> vPointCloudBuffer;

	SurfelFusionCore(oLidarPos, vDepthMeasurementCloud, vPointCloudBuffer);

	// 更新volume
	m_oVoxeler.UpdateConflictResult(vPointCloudBuffer);

	// output connect
	// pcl::PointCloud<pcl::PointNormal> vMaxConnected;
	// m_oVoxeler.GetMaxConnectCloud(vMaxConnected);
	// std::vector<float> feature(vMaxConnected.size(), 0.8f);
	// PublishPointCloud(vMaxConnected, feature, "/debug_max_connect");
}

pcl::PointCloud<pcl::PointNormal>::Ptr AllCloud(pcl::PointCloud<pcl::PointNormal>& cloud_vector) {

    return cloud_vector.makeShared();
}
