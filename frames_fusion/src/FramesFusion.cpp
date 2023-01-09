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
	m_oMeshPublisher = nodeHandle.advertise<visualization_msgs::Marker>(m_sOutMeshTopic, 1, true);
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

		pcl::PointCloud<pcl::PointXYZINormal> pc;
		for(auto point : *vAllPoints) {
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
		pcl::io::savePLYFileASCII(sOutPCNormalFileName.str(), pc);

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
	m_oGlobalNode.param("mf_output_path", m_sFileHead, std::string());
	if(m_sFileHead.empty())
 		nodeHandle.param("file_output_path", m_sFileHead, std::string("./"));
	m_bOutputFiles = !m_sFileHead.empty();

	if(m_bOutputFiles) {

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

	//use surfel fusion?
	nodeHandle.param("use_surfel_fusion", m_bSurfelFusion, true);
	// use additional points for reconstruction?
	nodeHandle.param("additional_points", m_bUseAdditionalPoints, true);

	nodeHandle.param("async_reconstruct", m_bAsyncReconstruction, true);

	nodeHandle.param("quick_surfel_fusion", m_bQuickSurfelFusion, false);

	//count processed point cloud frame
	m_iPCFrameCount = 0;

	//true indicates the file has not been generated
	m_bOutPCFileFlag = true;

	m_OdomLoopRate = ros::Rate(1 / m_fNearMeshPeriod);

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

void HSVToRGB(float H, float S, float V, uint8_t& R, uint8_t& G, uint8_t& B) {

	float C = V * S;
	int FixFactor = int(H / 60) & 1;
	float X = C * ((FixFactor ? -1 : 1) * (int(H) % 60) / 60.0 + FixFactor); // X 值的变化随H波动，锯齿状0-1之间周期变化
	float m = V - C;
	float R_, G_, B_;
	switch(int(H) / 60)
	{
		case 1:		R_ = X;	G_ = C;	B_ = 0; break;	// 60  <= H < 120
		case 2:		R_ = 0;	G_ = C;	B_ = X; break;	// 120 <= H < 180
		case 3:		R_ = 0;	G_ = X;	B_ = C; break;	// 180 <= H < 240
		case 4:		R_ = X;	G_ = 0;	B_ = C; break;	// 240 <= H < 300
		case 5: 
		case 6:		R_ = C;	G_ = 0;	B_ = X; break;	// 300 <= H < 360

		default:	R_ = C;	G_ = X;	B_ = 0; 		// 0   <= H < 60 or outlier
	}
	R = (R_ + m) * 255;
	G = (G_ + m) * 255;
	B = (B_ + m) * 255;
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
	visualization_msgs::Marker oMeshMsgs;
	
	//define header of message
	oMeshMsgs.header.frame_id = m_sOutMeshTFId;
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
	color.a = 1.0;
	color.r = 1.0;
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

        	//color
       		oMeshMsgs.points.push_back(oPTemp);
		}//end k

	}//end j

	m_oMeshPublisher.publish(oMeshMsgs);
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

// XXX
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
	pcl::PointCloud<pcl::PointNormal>::Ptr pNearCloud(new pcl::PointCloud<pcl::PointNormal>);

	//based on received point cloud and normal vector
	//get the target point cloud to be modeled
	m_mPCNMutex.lock();
	NearbyClouds(m_vMapPCN, oBasedP, *pNearCloud, m_fNearLengths);
	int point_num = pNearCloud->size();
	NearbyClouds(m_vMapPCNAdded, oBasedP, *pNearCloud, m_fNearLengths);
	m_mPCNMutex.unlock();

	// 点云抽稀
	int new_end = 0;
	if(m_iSampleInPNum > 1 && pNearCloud->size() > 1e5) {
		int past_point_num = point_num;
		std::cout << "sample: " << m_iSampleInPNum << " | " << pNearCloud->size();
		for(int i = 0; i < pNearCloud->size(); i+=m_iSampleInPNum) {
			swap(pNearCloud->at(i), pNearCloud->at(new_end++));
			if(i < past_point_num) point_num = new_end;
		}
		pNearCloud->erase(pNearCloud->begin() + new_end, pNearCloud->end());
		std::cout << " | " << pNearCloud->size() << std::endl;
	}

	std::vector<float> temp_feature(pNearCloud->size());
	PublishPointCloud(*pNearCloud, temp_feature, "/temp_near_cloud");

	///* output nearby cloud
	if(m_bOutputFiles) {

		pcl::PointCloud<pcl::PointXYZINormal> pc;
		for(auto point : *pNearCloud) {
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

	//******voxelization********
	//set voxelization parameters based on point cloud extent
	Voxelization oVoxeler(*pNearCloud);

	//set the number of voxels
	//set voxel resolution or voxel size
	//oVoxeler.GetIntervalNum(100,100,100);
	oVoxeler.GetResolution(m_oVoxelRes);

	//voxelize the space
	oVoxeler.VoxelizeSpace();

	//using signed distance
	SignedDistance oSDer;

	//compute signed distance based on centroids and its normals within voxels
	std::vector<float> vSignedDis = oSDer.NormalBasedGlance(pNearCloud, oVoxeler);

	//传回去
	m_mPCNMutex.lock();
	FusionNormalBackToPoint(*pNearCloud, m_vMapPCN, 0, point_num);
	FusionNormalBackToPoint(*pNearCloud, m_vMapPCNAdded, point_num + 1, pNearCloud->size() - point_num);
	m_mPCNMutex.unlock();

	//record non-empty voxels
	std::vector<bool> vVoxelStatus;
	oVoxeler.OutputNonEmptyVoxels(vVoxelStatus);

	//clear accelerated data
	oVoxeler.ClearMiddleData();

	//******construction********
	//marching cuber
	CIsoSurface<float> oMarchingCuber;
	//get surface mesh based on voxel related algorithm
	oMarchingCuber.GenerateSurface(vSignedDis, vVoxelStatus, 0,
			                       oVoxeler.m_iFinalVoxelNum.ixnum - 1, oVoxeler.m_iFinalVoxelNum.iynum - 1, oVoxeler.m_iFinalVoxelNum.iznum - 1, 
			                       oVoxeler.m_oVoxelLength.x, oVoxeler.m_oVoxelLength.y, oVoxeler.m_oVoxelLength.z);

	//move the mesh position to the actual starting point
	pcl::PointCloud<pcl::PointXYZ>::Ptr pMCResultCloud(new pcl::PointCloud<pcl::PointXYZ>);
	oMarchingCuber.OutputMesh(oVoxeler.m_oOriCorner, oCBModel, pMCResultCloud);
	
	///* output result mesh
	if(m_bOutputFiles) {

		std::stringstream sOutputPath;
		sOutputPath << m_sFileHead << std::setw(4) << std::setfill('0') << iFrameId << "_mesh.ply";
		pcl::io::savePLYFileBinary(sOutputPath.str(), oCBModel);
	}
	//*/

	// //new a mesh operation object for re-order the triangle vertex
	// MeshOperation oReOrder;

	// //compute the normal vector of each face for its centerpoint
	// //the normal vector will be facing away from the viewpoint
	// Eigen::MatrixXf oMCMatNormal;
	// Eigen::VectorXf vMCfDParam;

	// //note a bug releases
	// //sometimes the triangular generated by the CB algorithm has two same vertices but does not affect the result
	// oReOrder.ComputeAllFaceParams(oBasedP, *pMCResultCloud, oCBModel.polygons, oMCMatNormal, vMCfDParam);

}

// this is the main fusion function
/*	input 
	  @param oBasedP 视点 
	output
	  @param oCBModel 输出网格
*/
void FramesFusion::SurroundModelingWithPointProcessing(const pcl::PointXYZ & oBasedP, pcl::PolygonMesh & oCBModel, const int iFrameId) {

	//output mesh
	//oCBModel.cloud.clear();
	oCBModel.polygons.clear();

	//get the target points for construction
	pcl::PointCloud<pcl::PointNormal>::Ptr pNearCloud(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr pNearCloudAdded(new pcl::PointCloud<pcl::PointNormal>);

	//based on received point cloud and normal vector
	//get the target point cloud to be modeled
	NearbyClouds(m_vMapPCN, oBasedP, *pNearCloud, m_fNearLengths);
	int raw_point_num = pNearCloud->size();
	NearbyClouds(m_vMapPCNTrueAdded, oBasedP, *pNearCloud, m_fNearLengths);
	ExtractNearbyClouds(m_vMapPCNAdded, oBasedP, *pNearCloudAdded, m_fNearLengths);
	// NearbyClouds(m_vMapPCNAdded, oBasedP, *pNearCloudAdded, m_fNearLengths);

	///* output nearby cloud
	if(m_bOutputFiles) {

		std::stringstream filename;
		filename << m_sFileHead << std::setw(4) << std::setfill('0') << iFrameId << "_pc.ply";
		pcl::io::savePLYFileASCII(filename.str(), *pNearCloud);
	}
	//*/

	//******voxelization********
	//set voxelization parameters based on point cloud extent
	Voxelization oVoxeler(*pNearCloud);
	
	//set the number of voxels
	//set voxel resolution or voxel size
	//oVoxeler.GetIntervalNum(100,100,100);
	oVoxeler.GetResolution(m_oVoxelRes);

	//voxelize the space
	oVoxeler.VoxelizeSpace();

	//using signed distance
	SignedDistance oSDer;

	//compute signed distance based on centroids and its normals within voxels
	std::vector<float> vSignedDis = oSDer.NormalBasedGlance(pNearCloud, oVoxeler);
	//record non-empty voxels
	std::vector<bool> vVoxelStatus;
	oVoxeler.OutputNonEmptyVoxels(vVoxelStatus);

	// 注意，步骤到此，oVoxeler中已经记录了voxel中包含原始点云的信息 m_vVoxelPointIdx，以及融合后的点的信息 m_pVoxelNormals
	// 除此之外，还有 vSignedDis 以及 vVoxel Status(可以用 m_vVoxelPointIdx[i].size()代替) 等信息； 
	// pNearCloud 的法向量也已经被更新（此时只是用于重建的副本点云被更新，未涉及到真正的回传）
	// 对补全点的正确性进行判断
	std::vector<int> vTruePointCloudIndices;
	CheckAddedPointWithOcclusion(*pNearCloudAdded, oVoxeler, oBasedP, vTruePointCloudIndices);
	std::cout << std::format_blue << "fusion true added point size: " << vTruePointCloudIndices.size()
			  << "\t and the all added point size is: " << pNearCloudAdded->size() << std::format_white << std::endl;
	for(int i = 0; i < vTruePointCloudIndices.size(); ++i) {
		m_vMapPCNTrueAdded.push_back(pNearCloudAdded->points[vTruePointCloudIndices[i]]);
	}

	/**TODO: 存在的问题
	 * 1. 射线相交判断不准确，导致遗漏大量点
	 * 2. 上一步经过确认的点与原来的点不能保持坐标对应查询
	 * 3. 补全步骤延迟过大
	 */
	

	//传回去
	FusionNormalBackToPoint(*pNearCloud, m_vMapPCN, 0, raw_point_num);
	// FusionNormalBackToPoint(*pNearCloud, m_vMapPCNAdded, raw_point_num + 1, pNearCloud->size() - raw_point_num);

	//clear accelerated data
	oVoxeler.ClearMiddleData();

	//******construction********
	//marching cuber
	CIsoSurface<float> oMarchingCuber;
	//get surface mesh based on voxel related algorithm
	oMarchingCuber.GenerateSurface(vSignedDis, vVoxelStatus, 0,
			                       oVoxeler.m_iFinalVoxelNum.ixnum - 1, oVoxeler.m_iFinalVoxelNum.iynum - 1, oVoxeler.m_iFinalVoxelNum.iznum - 1, 
			                       oVoxeler.m_oVoxelLength.x, oVoxeler.m_oVoxelLength.y, oVoxeler.m_oVoxelLength.z);

	//move the mesh position to the actual starting point
	pcl::PointCloud<pcl::PointXYZ>::Ptr pMCResultCloud(new pcl::PointCloud<pcl::PointXYZ>);
	oMarchingCuber.OutputMesh(oVoxeler.m_oOriCorner, oCBModel, pMCResultCloud);

	///* output result mesh
	if(m_bOutputFiles) {

		std::stringstream sOutputPath;
		sOutputPath << m_sFileHead << std::setw(4) << std::setfill('0') << iFrameId << "_mesh.ply";
		pcl::io::savePLYFileBinary(sOutputPath.str(), oCBModel);
	}
	//*/
	
	// //new a mesh operation object for re-order the triangle vertex，in order to make the adjacency triangles face to the same direction
	// MeshOperation oReOrder;

	// //compute the normal vector of each face for its centerpoint
	// //the normal vector will be facing away from the viewpoint
	// Eigen::MatrixXf oMCMatNormal;
	// Eigen::VectorXf vMCfDParam;

	// //note a bug releases
	// //sometimes the triangular generated by the CB algorithm has two same vertices but does not affect the result
	// oReOrder.ComputeAllFaceParams(oBasedP, *pMCResultCloud, oCBModel.polygons, oMCMatNormal, vMCfDParam);
}

// New Fusion Function
void FramesFusion::SurroundModelingOnlyCheckOcclusion(const pcl::PointXYZ & oBasedP, pcl::PolygonMesh & oCBModel, const int iFrameId) {

	//oCBModel.cloud.clear();
	oCBModel.polygons.clear();

	//get the target points for construction
	pcl::PointCloud<pcl::PointNormal>::Ptr pNearCloud(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr pNearCloudAdded(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr pNearTrueAdded(new pcl::PointCloud<pcl::PointNormal>);

	//based on received point cloud and normal vector
	//get the target point cloud to be modeled
	NearbyClouds(m_vMapPCN, oBasedP, *pNearCloud, m_fNearLengths);
	NearbyClouds(m_vMapPCNAdded, oBasedP, *pNearCloudAdded, m_fNearLengths);

	//******voxelization********
	//set voxelization parameters based on point cloud extent
	Voxelization oVoxeler(*pNearCloud);
	
	//set the number of voxels
	//set voxel resolution or voxel size
	//oVoxeler.GetIntervalNum(100,100,100);
	oVoxeler.GetResolution(m_oVoxelRes);

	//voxelize the space
	oVoxeler.VoxelizeSpace();

	//using signed distance
	SignedDistance oSDer;

	//compute signed distance based on centroids and its normals within voxels
	std::vector<float> vSignedDis = oSDer.NormalBasedGlance(pNearCloud, oVoxeler);
	// 注意，步骤到此，oVoxeler中已经记录了voxel中包含原始点云的信息 m_vVoxelPointIdx，以及融合后的点的信息 m_pVoxelNormals
	// 除此之外，还有 vSignedDis 以及 vVoxel Status(可以用 m_vVoxelPointIdx[i].size()代替) 等信息； 
	// pNearCloud 的法向量也已经被更新（此时只是用于重建的副本点云被更新，未涉及到真正的回传）
	// 对补全点的正确性进行判断
	std::vector<int> vTruePointCloudIndices;
	CheckAddedPointWithOcclusion(*pNearCloudAdded, oVoxeler, oBasedP, vTruePointCloudIndices);
	std::cout << std::format_blue << "fusion true added point size: " << vTruePointCloudIndices.size()
			  << "\t and the all added point size is: " << pNearCloudAdded->size() << std::format_white << std::endl;
	for(int i = 0; i < vTruePointCloudIndices.size(); ++i) {
		pNearTrueAdded->push_back(pNearCloudAdded->points[vTruePointCloudIndices[i]]);
	}

	// add new points to the pointset and record
	int raw_point_num = pNearCloud->size();
	*pNearCloud += *pNearTrueAdded;

	// 第二次体素化
	Voxelization oVoxelerFixed(*pNearCloud);
	oVoxelerFixed.GetResolution(m_oVoxelRes);
	oVoxelerFixed.VoxelizeSpace();
	std::vector<float> vSignedDisFixed = oSDer.NormalBasedGlance(pNearCloud, oVoxelerFixed);
	//record non-empty voxels
	std::vector<bool> vVoxelStatus;
	oVoxelerFixed.OutputNonEmptyVoxels(vVoxelStatus);

	//传回去
	// FusionNormalBackToPoint(*pNearCloud, m_vMapPCN, 0, raw_point_num);
	// FusionNormalBackToPoint(*pNearCloud, m_vMapPCNAdded, raw_point_num + 1, pNearCloud->size() - raw_point_num);

	//clear accelerated data
	oVoxeler.ClearMiddleData();
	oVoxelerFixed.ClearMiddleData();

	//******construction********
	//marching cuber
	CIsoSurface<float> oMarchingCuber;
	//get surface mesh based on voxel related algorithm
	oMarchingCuber.GenerateSurface(vSignedDisFixed, vVoxelStatus, 0,
			                       oVoxelerFixed.m_iFinalVoxelNum.ixnum - 1, oVoxelerFixed.m_iFinalVoxelNum.iynum - 1, oVoxelerFixed.m_iFinalVoxelNum.iznum - 1, 
			                       oVoxelerFixed.m_oVoxelLength.x, oVoxelerFixed.m_oVoxelLength.y, oVoxelerFixed.m_oVoxelLength.z);

	//move the mesh position to the actual starting point
	pcl::PointCloud<pcl::PointXYZ>::Ptr pMCResultCloud(new pcl::PointCloud<pcl::PointXYZ>);
	oMarchingCuber.OutputMesh(oVoxelerFixed.m_oOriCorner, oCBModel, pMCResultCloud);
}

/*************************************************
Function: HandleRightLaser
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

	if(pFramePN->is_dense == false) {//标识码

		if(m_bUseAdditionalPoints) {
			
			++m_iFusionFrameNum;

			if(m_bSurfelFusion) {

				clock_t start_time = clock();

				AddedSurfelFusion(oViewPoint, *pFramePN);

				clock_t frames_fusion_time = 1000.0 * (clock() - start_time) / CLOCKS_PER_SEC;
				std::cout << std::format_purple 
					<< "The No. " << m_iFusionFrameNum 
					<< ";\tAdded_frames_fusion_time: " << frames_fusion_time << "ms" 
					<< std::format_white << std::endl;
				m_dAverageFusionTime += frames_fusion_time;
				m_dMaxFusionTime = frames_fusion_time > m_dMaxFusionTime ? frames_fusion_time : m_dMaxFusionTime;
			}

			m_vMapPCNAdded += *pFramePN;
		}

	}
	else {
		
		std::lock_guard<std::mutex> temp_lock(m_mPCNMutex);
		
		++m_iFusionFrameNum;

		if(m_bSurfelFusion) {

			clock_t start_time = clock();

			// the function will change m_vCurrentFramePoints, m_vMapPCNAdded
			if(m_bQuickSurfelFusion)
				SurfelFusionQuick(oViewPoint, *pFramePN);
			else SurfelFusion(oViewPoint, *pFramePN);

			clock_t frames_fusion_time = 1000.0 * (clock() - start_time) / CLOCKS_PER_SEC;
			std::cout << std::format_purple 
				<< "The No. " << m_iFusionFrameNum 
				<< ";\tframes_fusion_time: " << frames_fusion_time << "ms" 
				<< std::format_white << std::endl;
			m_dAverageFusionTime += frames_fusion_time;
			m_dMaxFusionTime = frames_fusion_time > m_dMaxFusionTime ? frames_fusion_time : m_dMaxFusionTime;
		}

		//merge one frame data
		m_vMapPCN += *pFramePN;
	}

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
	RosTimePoint oOdomPoint;
	oOdomPoint.oLocation.x = oTrajectory.pose.pose.position.x;
	oOdomPoint.oLocation.y = oTrajectory.pose.pose.position.y;
	oOdomPoint.oLocation.z = oTrajectory.pose.pose.position.z;

	//get the reconstructed surfaces
	pcl::PolygonMesh oNearbyMeshes;

	// Mesh Generate
	clock_t start_time = clock();

	if(!m_bSurfelFusion && m_bUseAdditionalPoints) 
		SurroundModelingWithPointProcessing(oOdomPoint.oLocation, oNearbyMeshes, m_iReconstructFrameNum);
	else SurroundModeling(oOdomPoint.oLocation, oNearbyMeshes, m_iReconstructFrameNum);
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

	auto vAllCloud = AllCloud(m_vMapPCN);
	std::cout << "### cloud_size: " << vAllCloud->size() << " ###" << std::endl; 
	PublishPointCloud(*vAllCloud);
}

void FramesFusion::HandleTrajectoryThread(const nav_msgs::Odometry & oTrajectory) {

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
	if(!bComputeFlag) {
		// m_OdomLoopRate.sleep();
		return;
	}
	
	//if need to be updated
	//get the newest information
	m_oLastModelingTime = oTrajectory.header.stamp;
	int now_frame_num = m_iReconstructFrameNum++;

	auto ModelingFunction = [&, now_frame_num]() {

		std::cout << std::format_purple << "No. " << now_frame_num << " reconstruct start" << std::format_white << std::endl;

		//save the position of trajectory
		RosTimePoint oOdomPoint;
		oOdomPoint.oLocation.x = oTrajectory.pose.pose.position.x;
		oOdomPoint.oLocation.y = oTrajectory.pose.pose.position.y;
		oOdomPoint.oLocation.z = oTrajectory.pose.pose.position.z;

		//get the reconstructed surfaces
		pcl::PolygonMesh oNearbyMeshes;

		struct timeval start;
		gettimeofday(&start, NULL);

		if(!m_bSurfelFusion && m_bUseAdditionalPoints) 
			SurroundModelingWithPointProcessing(oOdomPoint.oLocation, oNearbyMeshes, now_frame_num);
		else SurroundModeling(oOdomPoint.oLocation, oNearbyMeshes, now_frame_num);

		struct timeval end;
		gettimeofday(&end,NULL);
		double frame_reconstruct_time = (end.tv_sec - start.tv_sec) * 1000.0 +(end.tv_usec - start.tv_usec) * 0.001;
		std::cout << std::format_blue 
			<< "The No. " << now_frame_num
			<< ";\tframes_reconstruct_time: " << frame_reconstruct_time << "ms" 
			<< ";\tgen_face_num: " << oNearbyMeshes.polygons.size()
			<< std::format_white << std::endl;
		m_dAverageReconstructTime += frame_reconstruct_time;
		m_dMaxReconstructTime = frame_reconstruct_time > m_dMaxReconstructTime ? frame_reconstruct_time : m_dMaxReconstructTime;

		//output the nearby surfaces
		PublishMeshs(oNearbyMeshes);
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

void FramesFusion::CheckAddedPointWithOcclusion(const pcl::PointCloud<pcl::PointNormal> & vAddedCloud, Voxelization & oVoxeler, 
												const pcl::PointXYZ & oViewPoint, std::vector<int> & vTruePointIndices) {
	
	auto MoveToNextVoxel = [&oVoxeler](Eigen::Vector3f& point, const Eigen::Vector3f& rayDirection) {		
		/** 射线与AABB 相交 **/

		IndexinAxis oVoxelPositon;
		int iVoxelIndex = oVoxeler.PointBelongVoxel(point, oVoxelPositon);

		std::vector<int> vCornerIdxs;
		oVoxeler.CornerIdxs(oVoxelPositon, vCornerIdxs);
		pcl::PointXYZ oMinCorner = oVoxeler.m_pCornerCloud->points[vCornerIdxs.front()];
		pcl::PointXYZ oMaxCorner = oVoxeler.m_pCornerCloud->points[vCornerIdxs.back()];
		
		Eigen::Vector3f oEigenMin(oMinCorner.x, oMinCorner.y, oMinCorner.z);
		Eigen::Vector3f oEigenMax(oMaxCorner.x, oMaxCorner.y, oMaxCorner.z);

		// border judge
		bool bPointInBox = (Eigen::Vector3f(oEigenMin.array().min(point.array())) == oEigenMin);
		bPointInBox = bPointInBox && (Eigen::Vector3f(oEigenMax.array().max(point.array())) == oEigenMax);
		// if(bPointInBox) std::cout << "true" << std::endl;
		// else std::cout << std::format_red << "false" << std::format_white << std::endl;
		if(!bPointInBox) return -1;

		Eigen::Vector3f oLenMin = (oEigenMin.array() - point.array()) / rayDirection.array();
		Eigen::Vector3f oLenMax = (oEigenMax.array() - point.array()) / rayDirection.array();

		float fIntersectionLen = 1e10;
		if(oLenMin.x() > 0 && oLenMin.x() < fIntersectionLen) fIntersectionLen = oLenMin.x();
		if(oLenMin.y() > 0 && oLenMin.y() < fIntersectionLen) fIntersectionLen = oLenMin.y();
		if(oLenMin.z() > 0 && oLenMin.z() < fIntersectionLen) fIntersectionLen = oLenMin.z();
		if(oLenMax.x() > 0 && oLenMax.x() < fIntersectionLen) fIntersectionLen = oLenMax.x();
		if(oLenMax.y() > 0 && oLenMax.y() < fIntersectionLen) fIntersectionLen = oLenMax.y();
		if(oLenMax.z() > 0 && oLenMax.z() < fIntersectionLen) fIntersectionLen = oLenMax.z();

		if(fIntersectionLen == 1e10) {
			
			std::cout << "find false: \n"
					  << "     len:       " << oLenMin.transpose() << "\t" << oLenMax.transpose() << "\n"
					  << "     corner:    " << oEigenMin.transpose() << "\t" << oEigenMax.transpose() << "\n"
					  << "     point,ray: " << point.transpose() << "\t" << rayDirection.transpose() << std::endl;
			point += rayDirection;	/** 固定长度采样 **/
		}
		else {
			
			point += rayDirection * (fIntersectionLen + 0.001);
		}
		

		IndexinAxis oVoxelPositionNext;
		int iVoxelIndexNext = oVoxeler.PointBelongVoxel(point, oVoxelPositionNext);
		if(oVoxeler.OutOfBorder(oVoxelPositionNext)) return -1;
		return iVoxelIndexNext;
	};

	Eigen::Vector3f oStartPoint(oViewPoint.x, oViewPoint.y, oViewPoint.z);

	//对点云进行随机采样(弃用)
	// std::default_random_engine oRandomEngine(time(0));
	// uniform_int_distribution<int> oRandomIntGenerator(1, 5);

	//跳跃采样法
	constexpr int iJumpStep = 5;
	for(int i = 0; i < vAddedCloud.size(); i += iJumpStep) {

		Eigen::Vector3f oRayDirection;
		oRayDirection.x() = vAddedCloud[i].x - oViewPoint.x;
		oRayDirection.y() = vAddedCloud[i].y - oViewPoint.y;
		oRayDirection.z() = vAddedCloud[i].z - oViewPoint.z;
		// float fAddedPointStep = oRayDirection.norm() / oVoxeler.m_oVoxelLength.x;

		oRayDirection.normalize();
		// oRayDirection *= oVoxeler.m_oVoxelLength.x;

		Eigen::Vector3f oNowPoint(oStartPoint);
		int iVoxelIndex = MoveToNextVoxel(oNowPoint, oRayDirection);
		// int iNowStep = 1;
		bool bIsTruePoint = true;

		while(iVoxelIndex != -1) {

			bool bVoxelHasPoint = oVoxeler.m_vVoxelPointIdx[iVoxelIndex].size();

			if(bVoxelHasPoint) {

				pcl::PointNormal oPatch = oVoxeler.m_pVoxelNormals->points[iVoxelIndex];

				//计算射线是否与平面相交
				Eigen::Vector3f oPatchPoint(oPatch.x, oPatch.y, oPatch.z);
				Eigen::Vector3f oPatchNormal(oPatch.normal_x, oPatch.normal_y, oPatch.normal_z);

				Eigen::Vector3f oIntersectPoint = ((oPatchPoint - oNowPoint).dot(oPatchNormal) / oRayDirection.dot(oPatchNormal)) * oRayDirection + oNowPoint;

				bIsTruePoint = !((oIntersectPoint - oPatchPoint).norm() * 2 < oVoxeler.m_oVoxelLength.x);

				if(!bIsTruePoint) break;

				// if(fabs(fAddedPointStep - iNowStep) < 1.5) { //added_point close to voxel
				// 	bIsTruePoint = true;
				// 	break;
				// }

				// if(fAddedPointStep < iNowStep) { // added_point occlude the voxel
					
				// 	bIsTruePoint = false;
				// 	break;
				// }
				// else { // voxel occlude the added_point

				// 	bIsTruePoint = false;
				// 	break;
				// }
			}

			iVoxelIndex = MoveToNextVoxel(oNowPoint, oRayDirection);
			// ++iNowStep;
		}

		//只要跳跃采样点通过计算测试成功，则被跳跃的点也一并加入集合之中，视为正确的点（局部性原理）
		if(bIsTruePoint) {

			for(int j = 0; j < iJumpStep && (i + j) < vAddedCloud.size(); ++j) {
				vTruePointIndices.push_back(i + j);
			}

			// vTruePointIndices.push_back(i);
		}
	}
}

/** 基于反投影的点云融合
   @param 	pcl::PointNormal 					oLidarPos - m_oCurrentViewPoint 雷达视点的位置
   @param 	pcl::PointCloud<pcl::PointNormal> 	vDepthMeasurementCloud - vDepthMeasurementCloud 新一帧的点云
   @param_	pcl::PointCloud<pcl::PointNormal> 	m_vMapPCN - 旧点云（源自于lidar测量）
   @param_	pcl::PointCloud<pcl::PointNormal> 	m_vMapPCNAdded - 新点云（源自于单帧重建补点）
*/
void FramesFusion::SurfelFusion(pcl::PointNormal oLidarPos, pcl::PointCloud<pcl::PointNormal>& vDepthMeasurementCloud) {

	std::vector<Eigen::Vector3f> vRefPoints;
	double min_pitch = 200, max_pitch = -200, min_depth = INFINITY, max_depth = 0;

	constexpr int yaw_dim_expand = 4;
	constexpr int pitch_dim_expand = 2;
	constexpr int yaw_dim = 360 * yaw_dim_expand;
	constexpr int pitch_dim = 360 * pitch_dim_expand;

	std::vector<std::vector<std::vector<double>>> depth_image(pitch_dim, std::vector<std::vector<double>>(yaw_dim));
	std::vector<std::vector<std::vector<int>>> depth_index(pitch_dim, std::vector<std::vector<int>>(yaw_dim));
	// std::vector<float> yaw_record(vDepthMeasurementCloud.size());
	std::vector<float> point_feature(vDepthMeasurementCloud.size(), 0.5f);

	//计算点云每个点的角度，将其置于相机像素中
	for(int i = 0; i < vDepthMeasurementCloud.size(); ++i) {

		const pcl::PointNormal& oCurrentPoint = vDepthMeasurementCloud.at(i);
		Eigen::Vector3f oRefPoint(oCurrentPoint.x - oLidarPos.x, oCurrentPoint.y - oLidarPos.y, oCurrentPoint.z - oLidarPos.z);
		vRefPoints.push_back(oRefPoint);

		// 计算投影位置和深度
		double depth = oRefPoint.norm();
		oRefPoint.normalize();
		double yaw = std::atan2((double)oRefPoint.y(), (double)oRefPoint.x()) / M_PI * 180.0;
		double pitch = std::asin((double)oRefPoint.z()) / M_PI * 180.0;
		// std::cout << "Point: " << oCurrentPoint.curvature << "\t";
		// std::cout << std::fixed << std::setprecision(3)
		// 		  << oRefPoint.x() << "," << oRefPoint.y() << "," << oRefPoint.z() << "\t" << yaw << "," << pitch << std::endl;
		min_pitch = pitch < min_pitch ? pitch : min_pitch;
		max_pitch = pitch > max_pitch ? pitch : max_pitch;
		min_depth = depth < min_depth ? depth : min_depth;
		max_depth = depth > max_depth ? depth : max_depth;
		// yaw_record[i] = yaw / 360.0 + 0.5;

		// 将投影结果存储
		double row = (pitch + 180) * pitch_dim_expand;
		double col = (yaw   + 180) * yaw_dim_expand;
		depth_image[int(row)][int(col)].push_back(depth);
		depth_index[int(row)][int(col)].push_back(i);

		// 根据surfel在subpixel的位置，添加到邻近的像素中去
		double local_row = row - int(row);
		double local_col = col - int(col);

		// 如果只扩展左右，不扩展上下如何？好像效果还可以，但是仍然不能完全解决单向素拉长导致的错误complict判断问题
		// /* 双向扩展
		if(local_col > 0.6) {
			// 右
			int temp_col = col + 1;
			if(temp_col >= yaw_dim) temp_col = 0;
			depth_image[int(row)][temp_col].push_back(depth);
			depth_index[int(row)][temp_col].push_back(i);
		}
		else if(local_col < 0.4) {
			// 左
			int temp_col = col - 1;
			if(temp_col < 0) temp_col = yaw_dim - 1;
			depth_image[int(row)][temp_col].push_back(depth);
			depth_index[int(row)][temp_col].push_back(i);
		}
		// */

		/*上下
		// 上
		if(local_row < 0.2) {
			int temp_row = row - 1;
			if(temp_row >= 0) {
				depth_image[temp_row][int(col)].push_back(depth);
				depth_index[temp_row][int(col)].push_back(i);
			}
		}

		// 下
		if(local_row > 0.8) {
			int temp_row = row + 1;
			if(temp_row < pitch_dim) {
				depth_image[temp_row][int(col)].push_back(depth);
				depth_index[temp_row][int(col)].push_back(i);
			}
		}
		//*/
		
		/* 四向扩展
		if(local_row < local_col) {

			if(local_row + local_col >= 1) {
				// 右
				int temp_col = col + 1;
				if(temp_col >= yaw_dim) temp_col = 0;
				depth_image[int(row)][temp_col].push_back(depth);
				depth_index[int(row)][temp_col].push_back(i);
			}
			else {
				// 上
				int temp_row = row - 1;
				if(temp_row >= 0) {
					depth_image[temp_row][int(col)].push_back(depth);
					depth_index[temp_row][int(col)].push_back(i);
				}
			}
		}
		else {

			if(local_row + local_col < 1) {
				// 左
				int temp_col = col - 1;
				if(temp_col < 0) temp_col = yaw_dim - 1;
				depth_image[int(row)][temp_col].push_back(depth);
				depth_index[int(row)][temp_col].push_back(i);
			}
			else {
				// 下
				int temp_row = row + 1;
				if(temp_row < pitch_dim) {
					depth_image[temp_row][int(col)].push_back(depth);
					depth_index[temp_row][int(col)].push_back(i);
				}
			}
		}
		//*/
	}

	// std::cout << std::format_blue << std::fixed << std::setprecision(3) 
	// 		  << "Point pitch range: [" << min_pitch << ", " << max_pitch << "]\t"
	// 		  << "Depth range: [" << min_depth << ", " << max_depth << "]" 
	// 		  << std::format_white << std::endl;

	/* 计算深度平均数，并将结果添加到数组末尾(或许可以尝试最小值?)
	for(int row = 0; row < depth_image.size(); ++row) {
		for(int col = 0; col < depth_image[row].size(); ++col) {

			double average_depth = 0;

			if(depth_image[row][col].size()) {

				for(auto& depth_sample : depth_image[row][col])
					average_depth += depth_sample;

				//在数组末尾添加平均值方便利用
				average_depth /= depth_image.size();
				depth_image[row][col].push_back(average_depth);
			}
		}
	}
	//*/

	/* 深度数组输出为图片
	{
		cv::Mat oDepthImage = cv::Mat::zeros(cv::Size(yaw_dim, pitch_dim), CV_8UC1);

		for(int row = 0; row < depth_image.size(); ++row) {
			for(int col = 0; col < depth_image[row].size(); ++col) {

				if(depth_image[row][col].size()) {

					double average_depth = depth_image[row][col].back();
					
					//将平均值输出到图片
					uint8_t depth_color = (average_depth - 90) / (12) * 255;
					oDepthImage.data[row * yaw_dim + col] = depth_color;
				}
			}
		}
		
		std::stringstream sFileName;
		sFileName << m_sFileHead << "img_" << vDepthMeasurementCloud.header.seq << ".png";
		std::stringstream sMakeFileDirCommand;
		sMakeFileDirCommand << "mkdir -p " << m_sFileHead;
		system(sMakeFileDirCommand.str().c_str());

		cv::imwrite(sFileName.str(), oDepthImage, {cv::ImwriteFlags::IMWRITE_PNG_STRATEGY});
	}
	//*/

	// 把added点也考虑进来，并加入置信度系统
	pcl::PointCloud<pcl::PointNormal> vPointCloudBuffer;
	std::vector<int> vPointConfidence(m_vMapPCN.size(), 1);
	std::vector<int> vAddedConfidence(m_vMapPCNAdded.size(), 0);
	std::vector<bool> vCurrentFuseIndex(vDepthMeasurementCloud.size(), false);
	vPointCloudBuffer += m_vMapPCN;
	vPointCloudBuffer += m_vMapPCNAdded;

	// /* 对于之前帧的所有点，对应位置建立匹配关系
	constexpr double support_factor = 0.3;
	constexpr double tight_support_factor = 0.1;
	constexpr double normal_support_factor = 0.8;
	std::vector<int> vSupportPointIndex;
	std::vector<int> vOcclusionPointIndex;
	std::vector<int> vComplictPointIndex;

	std::vector<float> associated_feature;

	for(int i = 0; i < vPointCloudBuffer.size(); ++i) {

		const pcl::PointNormal& oCurrentPoint = vPointCloudBuffer.at(i);
		Eigen::Vector3f oRefPoint(oCurrentPoint.x - oLidarPos.x, oCurrentPoint.y - oLidarPos.y, oCurrentPoint.z - oLidarPos.z);

		double depth = oRefPoint.norm();
		oRefPoint.normalize();
		double yaw = std::atan2((double)oRefPoint.y(), (double)oRefPoint.x()) / M_PI * 180.0;
		double pitch = std::asin((double)oRefPoint.z()) / M_PI * 180.0;
		
		int row = (pitch + 180) * pitch_dim_expand;
		int col = (yaw   + 180) * yaw_dim_expand;

		int support_count = 0;
		int tight_support_count = 0;
		int occlusion_count = 0;
		int complict_count = 0;

		pcl::PointNormal* pSupportPoint = nullptr;

		if(row < pitch_dim && col < yaw_dim && depth_index[row][col].size()) {

			for(int pixel_index = 0; pixel_index < depth_index[row][col].size(); ++pixel_index) {
				
				pcl::PointNormal& oAssociatedPoint = vDepthMeasurementCloud.points[depth_index[row][col][pixel_index]];
				double dAssociatedDepth = depth_image[row][col][pixel_index];

				// 判断是否与对应点是support关系
				if(abs(depth - dAssociatedDepth) < support_factor) {
					
					++support_count;
					pSupportPoint = &oAssociatedPoint;

					// 判断是否严格support，应该还判断法向，但似乎不太必要
					if(abs(depth - dAssociatedDepth) < tight_support_factor) {

						// Eigen::Vector3f oAssociatedNormal(oAssociatedPoint.normal_x, oAssociatedPoint.normal_y, oAssociatedPoint.normal_z);
						// Eigen::Vector3f oOldNormal(oCurrentPoint.normal_x, oCurrentPoint.normal_y, oCurrentPoint.normal_z);
						// if(oAssociatedNormal.dot(oOldNormal) > normal_support_factor) {
							++ tight_support_count;
							vCurrentFuseIndex[depth_index[row][col][pixel_index]] = true; // 新帧的点会被合并到旧帧点，并不添加新的点，记录下来之后剔除这些新点
						// }
					}
				}
				// 判断complict关系
				else if(depth < dAssociatedDepth) { // 旧点的depth < 新点的depth
					
					++complict_count;
				}
				// occlusion关系
				else {
					++occlusion_count;
				}
			}
		}

		if(support_count > 0) {
			
			if(tight_support_count > 0) {

				vSupportPointIndex.push_back(i);
				associated_feature.push_back(0.4f);
				if(i < vPointConfidence.size()) ++vPointConfidence[i];
			}
			else {

				vSupportPointIndex.push_back(i);
				associated_feature.push_back(0.15f);				
				
				// 对于非tight_support点，将其投影到新测量的平面上(弃用，非置信度融合方式会破坏远处物体结构)
				// if(pSupportPoint != nullptr) {
				// 	pcl::PointNormal& oCurrentPoint = vPointCloudBuffer[i];	//旧点
				// 	Eigen::Vector3f oDiffVector(oCurrentPoint.x - pSupportPoint->x, oCurrentPoint.y - pSupportPoint->y, oCurrentPoint.z - pSupportPoint->z);
				// 	Eigen::Vector3f oSupportNormal(pSupportPoint->normal_x, pSupportPoint->normal_y, pSupportPoint->normal_z);
				// 	auto oProjectDiff = - oDiffVector.dot(oSupportNormal) * oSupportNormal;
				// 	oCurrentPoint.x += oProjectDiff.x();
				// 	oCurrentPoint.y += oProjectDiff.y();
				// 	oCurrentPoint.z += oProjectDiff.z();
				// }
			}

			// 根据（深度）置信度融合
			if(pSupportPoint != nullptr) {
				
				pcl::PointNormal& oOldPoint = i < m_vMapPCN.size() ? m_vMapPCN[i] : m_vMapPCNAdded[i - m_vMapPCN.size()];
				// pcl::PointNormal& oOldPoint = vPointCloudBuffer[i];
				float& fOldConfidence = oOldPoint.data_n[3];
				pcl::PointNormal& oNewPoint = *pSupportPoint;
				float& fNewConfidence = oNewPoint.data_n[3];
				Eigen::Vector3f oDiffVector(oOldPoint.x - oNewPoint.x, oOldPoint.y - oNewPoint.y, oOldPoint.z - oNewPoint.z);
				Eigen::Vector3f oNewNormal(oNewPoint.normal_x, oNewPoint.normal_y, oNewPoint.normal_z);
				Eigen::Vector3f oProjectDiff = - oDiffVector.dot(oNewNormal) * oNewNormal;

				// oOldPoint.x = fOldConfidence * oOldPoint.x + fNewConfidence * oNewPoint.x;
				// oOldPoint.y = fOldConfidence * oOldPoint.y + fNewConfidence * oNewPoint.y;
				// oOldPoint.z = fOldConfidence * oOldPoint.z + fNewConfidence * oNewPoint.z;
				fOldConfidence += fNewConfidence;
				oOldPoint.x += oProjectDiff.x() * fNewConfidence / fOldConfidence;
				oOldPoint.y += oProjectDiff.y() * fNewConfidence / fOldConfidence;
				oOldPoint.z += oProjectDiff.z() * fNewConfidence / fOldConfidence;

				// std::cout << std::format_blue << "confidence: " << fOldConfidence << "," << fNewConfidence << std::format_white << std::endl;
				// oOldPoint.x /= fOldConfidence;
				// oOldPoint.y /= fOldConfidence;
				// oOldPoint.z /= fOldConfidence;
			}

		}
		else if(complict_count > 0) {

			vComplictPointIndex.push_back(i);
			associated_feature.push_back(0.0f);
			if(i < vPointConfidence.size()) --vPointConfidence[i];
			else --vAddedConfidence[i - vPointConfidence.size()];
		}
		else if(occlusion_count > 0) {

			vOcclusionPointIndex.push_back(i);
			associated_feature.push_back(0.6f);
		}
		else {
			associated_feature.push_back(-1.0f);
		}
	}

	// std::cout << std::format_blue 
	// 		  << "associated percent: " << ((vSupportPointIndex.size() + vComplictPointIndex.size() + vOcclusionPointIndex.size()) * 100.0 / vPointCloudBuffer.size()) << "% "
	// 		  << "[" << (vSupportPointIndex.size() + vComplictPointIndex.size() + vOcclusionPointIndex.size()) << "/" << vPointCloudBuffer.size() << "]\t"
	// 		  << "supported point num: " << vSupportPointIndex.size() << ",\t"
	// 		  << "complict point num: " << vComplictPointIndex.size() << ",\t"
	// 		  << "occlusion point num: " << vOcclusionPointIndex.size()
	// 		  << std::format_white << std::endl;


	// 删除被新帧中的被融合点
	int new_end = vCurrentFuseIndex.size();
	for(int i = new_end - 1; i >= 0; --i) {

		if(vCurrentFuseIndex[i])
		{
			swap(vDepthMeasurementCloud.points[i],vDepthMeasurementCloud.points[--new_end]);
			point_feature[new_end] = 0.9f;
		}
	}
	// PublishPointCloud(vDepthMeasurementCloud, yaw_record, "/current_frame_yaw");
	PublishPointCloud(vDepthMeasurementCloud, point_feature, "/current_frame_const");
	vDepthMeasurementCloud.erase(vDepthMeasurementCloud.begin() + new_end, vDepthMeasurementCloud.end());
	std::cout << std::format_blue << "new point num: " << vDepthMeasurementCloud.size() << std::format_white << "\t";


	// 删除 Added 帧中置信度不足的点，效果不太好，会错误删除很多点
	// constexpr int added_erase_conf = -1;
	// int added_end = vAddedConfidence.size(), original_end = added_end;
	// for(int i = added_end - 1; i >= 0; --i) {
	// 	if(vAddedConfidence[i] <= added_erase_conf)
	// 	{
	// 		swap(m_vMapPCNAdded.points[i], m_vMapPCNAdded.points[--added_end]);
	// 		associated_feature[vPointConfidence.size() + i] = 0.8f;
	// 	}
	// }
	// m_vMapPCNAdded.erase(m_vMapPCNAdded.begin() + added_end, m_vMapPCNAdded.end());
	// std::cout << std::format_blue << "erase point num: " << original_end - m_vMapPCNAdded.size() << std::format_white << std::endl;

	PublishPointCloud(vPointCloudBuffer, associated_feature, "/debug_associated_point");
	//*/
}


// 对于补充点的反投影融合方式，筛选过滤更加严格
/** 基于反投影的点云融合
   @param 	pcl::PointNormal 					oLidarPos - m_oCurrentViewPoint 雷达视点的位置
   @param 	pcl::PointCloud<pcl::PointNormal> 	vDepthMeasurementCloud - vDepthMeasurementCloud 新一帧的点云
   @param_	pcl::PointCloud<pcl::PointNormal> 	m_vMapPCN - 旧点云（源自于lidar测量）
   @param_	pcl::PointCloud<pcl::PointNormal> 	m_vMapPCNAdded - 新点云（源自于单帧重建补点）
*/
void FramesFusion::AddedSurfelFusion(pcl::PointNormal oLidarPos, pcl::PointCloud<pcl::PointNormal>& vDepthMeasurementCloud) {

	std::vector<Eigen::Vector3f> vRefPoints;
	double min_pitch = 200, max_pitch = -200, min_depth = INFINITY, max_depth = 0;

	constexpr int yaw_dim_expand = 2;
	constexpr int pitch_dim_expand = 2;
	constexpr int yaw_dim = 360 * yaw_dim_expand;
	constexpr int pitch_dim = 360 * pitch_dim_expand;

	std::vector<std::vector<std::vector<double>>> depth_image(pitch_dim, std::vector<std::vector<double>>(yaw_dim));
	std::vector<std::vector<std::vector<int>>> depth_index(pitch_dim, std::vector<std::vector<int>>(yaw_dim));
	std::vector<float> point_feature(vDepthMeasurementCloud.size(), 0.5f);

	//计算点云每个点的角度，将其置于相机像素中
	for(int i = 0; i < vDepthMeasurementCloud.size(); ++i) {

		const pcl::PointNormal& oCurrentPoint = vDepthMeasurementCloud.at(i);
		Eigen::Vector3f oRefPoint(oCurrentPoint.x - oLidarPos.x, oCurrentPoint.y - oLidarPos.y, oCurrentPoint.z - oLidarPos.z);
		vRefPoints.push_back(oRefPoint);

		// 计算投影位置和深度
		double depth = oRefPoint.norm();
		oRefPoint.normalize();
		double yaw = std::atan2((double)oRefPoint.y(), (double)oRefPoint.x()) / M_PI * 180.0;
		double pitch = std::asin((double)oRefPoint.z()) / M_PI * 180.0;
		min_pitch = pitch < min_pitch ? pitch : min_pitch;
		max_pitch = pitch > max_pitch ? pitch : max_pitch;
		min_depth = depth < min_depth ? depth : min_depth;
		max_depth = depth > max_depth ? depth : max_depth;
		// yaw_record[i] = yaw / 360.0 + 0.5;

		// 将投影结果存储
		double row = (pitch + 180) * pitch_dim_expand;
		double col = (yaw   + 180) * yaw_dim_expand;
		depth_image[int(row)][int(col)].push_back(depth);
		depth_index[int(row)][int(col)].push_back(i);

		// 根据surfel在subpixel的位置，添加到邻近的像素中去
		double local_row = row - int(row);
		double local_col = col - int(col);

		// 如果只扩展左右，不扩展上下如何？好像效果还可以，但是仍然不能完全解决单向素拉长导致的错误complict判断问题
		// /* 双向扩展
		if(local_col > 0.6) {
			// 右
			int temp_col = col + 1;
			if(temp_col >= yaw_dim) temp_col = 0;
			depth_image[int(row)][temp_col].push_back(depth);
			depth_index[int(row)][temp_col].push_back(i);
		}
		else if(local_col < 0.4) {
			// 左
			int temp_col = col - 1;
			if(temp_col < 0) temp_col = yaw_dim - 1;
			depth_image[int(row)][temp_col].push_back(depth);
			depth_index[int(row)][temp_col].push_back(i);
		}
		// */
	}

	// 把added点也考虑进来，并加入置信度系统
	pcl::PointCloud<pcl::PointNormal> vPointCloudBuffer;
	std::vector<bool> vCurrentFuseIndex(vDepthMeasurementCloud.size(), false);
	vPointCloudBuffer += m_vMapPCN;
	vPointCloudBuffer += m_vMapPCNAdded;

	// /* 对于之前帧的所有点，对应位置建立匹配关系
	constexpr double support_factor = 0.3;
	constexpr double tight_support_factor = 0.1;
	std::vector<int> vSupportPointIndex;

	std::vector<float> associated_feature;

	for(int i = 0; i < vPointCloudBuffer.size(); ++i) {

		const pcl::PointNormal& oCurrentPoint = vPointCloudBuffer.at(i);
		Eigen::Vector3f oRefPoint(oCurrentPoint.x - oLidarPos.x, oCurrentPoint.y - oLidarPos.y, oCurrentPoint.z - oLidarPos.z);

		double depth = oRefPoint.norm();
		oRefPoint.normalize();
		double yaw = std::atan2((double)oRefPoint.y(), (double)oRefPoint.x()) / M_PI * 180.0;
		double pitch = std::asin((double)oRefPoint.z()) / M_PI * 180.0;
		
		int row = (pitch + 180) * pitch_dim_expand;
		int col = (yaw   + 180) * yaw_dim_expand;

		int support_count = 0;
		int tight_support_count = 0;
		int occlusion_count = 0;
		int complict_count = 0;

		pcl::PointNormal* pSupportPoint = nullptr;

		if(row < pitch_dim && col < yaw_dim && depth_index[row][col].size()) {

			for(int pixel_index = 0; pixel_index < depth_index[row][col].size(); ++pixel_index) {
				
				pcl::PointNormal& oAssociatedPoint = vDepthMeasurementCloud.points[depth_index[row][col][pixel_index]];
				double dAssociatedDepth = depth_image[row][col][pixel_index];

				// 无论是什么关系都代表这个点不行
				vCurrentFuseIndex[depth_index[row][col][pixel_index]] = true; // 新帧的点会被合并到旧帧点，并不添加新的点，记录下来之后剔除这些新点

				// 判断是否与对应点是support关系
				if(abs(depth - dAssociatedDepth) < support_factor) {
					
					++support_count;
					pSupportPoint = &oAssociatedPoint;

					if(abs(depth - dAssociatedDepth) < tight_support_factor) {

						++ tight_support_count;
					}
				}
				// 判断complict关系
				else if(depth < dAssociatedDepth) { // 旧点的depth < 新点的depth
					
					++complict_count;
				}
				// occlusion关系
				else {
					++occlusion_count;
				}
			}
		}

		if(support_count > 0) {
			
			if(tight_support_count > 0) {

				vSupportPointIndex.push_back(i);
				associated_feature.push_back(0.4f);
			}
			else {

				vSupportPointIndex.push_back(i);
				associated_feature.push_back(0.15f);				
			}
			
			// 根据（深度）置信度融合
			if(pSupportPoint != nullptr) {
				
				pcl::PointNormal& oOldPoint = i < m_vMapPCN.size() ? m_vMapPCN[i] : m_vMapPCNAdded[i - m_vMapPCN.size()];
				// pcl::PointNormal& oOldPoint = vPointCloudBuffer[i];
				float& fOldConfidence = oOldPoint.data_n[3];
				pcl::PointNormal& oNewPoint = *pSupportPoint;
				float& fNewConfidence = oNewPoint.data_n[3];
				Eigen::Vector3f oDiffVector(oOldPoint.x - oNewPoint.x, oOldPoint.y - oNewPoint.y, oOldPoint.z - oNewPoint.z);
				Eigen::Vector3f oNewNormal(oNewPoint.normal_x, oNewPoint.normal_y, oNewPoint.normal_z);
				Eigen::Vector3f oProjectDiff = - oDiffVector.dot(oNewNormal) * oNewNormal;

				// oOldPoint.x = fOldConfidence * oOldPoint.x + fNewConfidence * oNewPoint.x;
				// oOldPoint.y = fOldConfidence * oOldPoint.y + fNewConfidence * oNewPoint.y;
				// oOldPoint.z = fOldConfidence * oOldPoint.z + fNewConfidence * oNewPoint.z;
				fOldConfidence += fNewConfidence;
				oOldPoint.x += oProjectDiff.x() * fNewConfidence / fOldConfidence;
				oOldPoint.y += oProjectDiff.y() * fNewConfidence / fOldConfidence;
				oOldPoint.z += oProjectDiff.z() * fNewConfidence / fOldConfidence;

				// std::cout << std::format_blue << "confidence: " << fOldConfidence << "," << fNewConfidence << std::format_white << std::endl;
				// oOldPoint.x /= fOldConfidence;
				// oOldPoint.y /= fOldConfidence;
				// oOldPoint.z /= fOldConfidence;
			}
		}
		else if(complict_count > 0) {

			associated_feature.push_back(0.0f);
		}
		else if(occlusion_count > 0) {

			associated_feature.push_back(0.6f);
		}
		else {
			associated_feature.push_back(-1.0f);
		}
	}

	// 删除被新帧中的被融合点
	int new_end = vCurrentFuseIndex.size();
	for(int i = new_end - 1; i >= 0; --i) {

		if(vCurrentFuseIndex[i])
		{
			swap(vDepthMeasurementCloud.points[i],vDepthMeasurementCloud.points[--new_end]);
			point_feature[new_end] = 0.9f;
		}
	}
	// PublishPointCloud(vDepthMeasurementCloud, yaw_record, "/current_frame_yaw");
	PublishPointCloud(vDepthMeasurementCloud, point_feature, "/added_current_frame_const");
	vDepthMeasurementCloud.erase(vDepthMeasurementCloud.begin() + new_end, vDepthMeasurementCloud.end());
	std::cout << std::format_blue << "new point num: " << vDepthMeasurementCloud.size() << std::format_white << "\t";

	PublishPointCloud(vPointCloudBuffer, associated_feature, "/added_debug_associated_point");
	//*/
}

// 算法简化加速
/** 基于反投影的点云融合
   @param 	pcl::PointNormal 					oLidarPos - m_oCurrentViewPoint 雷达视点的位置
   @param 	pcl::PointCloud<pcl::PointNormal> 	vDepthMeasurementCloud - vDepthMeasurementCloud 新一帧的点云
   @param_	pcl::PointCloud<pcl::PointNormal> 	m_vMapPCN - 旧点云（源自于lidar测量）
   @param_	pcl::PointCloud<pcl::PointNormal> 	m_vMapPCNAdded - 新点云（源自于单帧重建补点）
*/
void FramesFusion::SurfelFusionQuick(pcl::PointNormal oLidarPos, pcl::PointCloud<pcl::PointNormal>& vDepthMeasurementCloud) {

	std::cout << "fusion start \t";

	constexpr int yaw_dim_expand = 4;
	constexpr int pitch_dim_expand = 2;
	constexpr int yaw_dim = 360 * yaw_dim_expand;
	constexpr int pitch_dim = 360 * pitch_dim_expand;
	
	constexpr double support_factor = 0.3;
	constexpr double tight_support_factor = 0.1;
	constexpr double normal_support_factor = 0.8;

	//计算点云每个点的角度，将其置于相机像素中 4ms - 8ms
	// clock_t start_time1 = clock();

	//验证性调试，假设视点不变
	// oLidarPos.x = 0;
	// oLidarPos.y = 0;
	// oLidarPos.z = 0;

	std::vector<std::vector<double>> depth_image(pitch_dim, std::vector<double>(yaw_dim));
	std::vector<std::vector<int>> depth_index(pitch_dim, std::vector<int>(yaw_dim));
	std::vector<bool> vCurrentFuseIndex(vDepthMeasurementCloud.size(), false);

	double max_depth = 0;
	for(int i = 0; i < vDepthMeasurementCloud.size(); ++i) {

		const pcl::PointNormal& oCurrentPoint = vDepthMeasurementCloud.at(i);
		Eigen::Vector3f oRefPoint(oCurrentPoint.x - oLidarPos.x, oCurrentPoint.y - oLidarPos.y, oCurrentPoint.z - oLidarPos.z);

		// 计算投影位置和深度
		double depth = oRefPoint.norm();
		max_depth = max_depth < depth ? depth : max_depth;

		oRefPoint.normalize();
		double yaw = std::atan2((double)oRefPoint.y(), (double)oRefPoint.x()) / M_PI * 180.0;
		double pitch = std::asin((double)oRefPoint.z()) / M_PI * 180.0;

		// 将投影结果存储
		double row = (pitch + 180) * pitch_dim_expand;
		double col = (yaw   + 180) * yaw_dim_expand;
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

	// clock_t frames_fusion_time1 = 1000.0 * (clock() - start_time1) / CLOCKS_PER_SEC;
	// std::cout << std::format_purple 
	// 	<< "project_image_time: " << frames_fusion_time1 << "ms" 
	// 	<< std::format_white << std::endl;
	// input - lidar pos, measurement cloud | output - depth_image and depth_index
	

	// 把added点也考虑进来，并加入置信度系统
	// clock_t start_time2 = clock();

	pcl::PointCloud<pcl::PointNormal> vPointCloudBuffer;
	// vPointCloudBuffer += m_vMapPCN;
	// vPointCloudBuffer += m_vMapPCNAdded;
	pcl::PointXYZ oBase(oLidarPos.x, oLidarPos.y, oLidarPos.z);
	NearbyClouds(m_vMapPCN, oBase, vPointCloudBuffer, max_depth);
	int lidar_size = vPointCloudBuffer.size();
	NearbyClouds(m_vMapPCNAdded, oBase, vPointCloudBuffer, max_depth);

	// clock_t frames_fusion_time2 = 1000.0 * (clock() - start_time2) / CLOCKS_PER_SEC;
	// std::cout << std::format_purple 
	// 	<< "copy_cloud_time: " << frames_fusion_time2 << "ms" 
	// 	<< std::format_white << std::endl;
	// input - point clouds | output - vPointCloudBuffer

	// /* 对于之前帧的所有点，对应位置建立匹配关系 60 - 150 ms
	// clock_t start_time3 = clock();

	std::vector<float> associated_feature(vPointCloudBuffer.size(), 0);

	for(int i = 0; i < vPointCloudBuffer.size(); ++i) {

		const pcl::PointNormal& oCurrentPoint = vPointCloudBuffer.at(i);
		Eigen::Vector3f oRefPoint(oCurrentPoint.x - oLidarPos.x, oCurrentPoint.y - oLidarPos.y, oCurrentPoint.z - oLidarPos.z);

		double depth = oRefPoint.norm();
		oRefPoint.normalize();
		double yaw = std::atan2((double)oRefPoint.y(), (double)oRefPoint.x()) / M_PI * 180.0;
		double pitch = std::asin((double)oRefPoint.z()) / M_PI * 180.0;
		
		int row = (pitch + 180) * pitch_dim_expand;
		int col = (yaw   + 180) * yaw_dim_expand;

		int support_count = 0;
		int tight_support_count = 0;
		int occlusion_count = 0;
		int complict_count = 0;

		pcl::PointNormal* pSupportPoint = nullptr;
		pcl::PointNormal* pConflictPoint = nullptr;

		if(row < pitch_dim && col < yaw_dim && depth_image[row][col]) {

			// for(int pixel_index = 0; pixel_index < depth_index[row][col].size(); ++pixel_index) {
				
				pcl::PointNormal& oAssociatedPoint = vDepthMeasurementCloud.points[depth_index[row][col]];
				double dAssociatedDepth = depth_image[row][col];

				// 判断是否与对应点是support关系
				if(abs(depth - dAssociatedDepth) < support_factor) {
					
					++support_count;
					pSupportPoint = &oAssociatedPoint;

					if(abs(depth - dAssociatedDepth) < tight_support_factor) {

						++ tight_support_count;
						vCurrentFuseIndex[depth_index[row][col]] = true; // 新帧的点会被合并到旧帧点，并不添加新的点，记录下来之后剔除这些新点
					}
				}
				// 判断complict关系
				else if(depth < dAssociatedDepth) { // 旧点的depth < 新点的depth
					
					++complict_count;
					pConflictPoint = &oAssociatedPoint;
				}
				// occlusion关系
				else {
					++occlusion_count;
				}
			// }
		}

		if(support_count > 0) {
			
			if(tight_support_count > 0) {

				// associated_feature[i] = 0.4f;
				associated_feature[i] = -1.f;
			}
			else {

				// associated_feature[i] = 0.15f;	
				associated_feature[i] = -1.f;			
			}

			// 根据（深度）置信度融合
			if(pSupportPoint != nullptr) {
				
				pcl::PointNormal& oOldPoint = i < lidar_size ? m_vMapPCN[vPointCloudBuffer[i].data_c[3]] : m_vMapPCNAdded[vPointCloudBuffer[i].data_c[3]];
				// pcl::PointNormal& oOldPoint = m_vMapPCN[vPointCloudBuffer[i].data_c[3]];
				float& fOldConfidence = oOldPoint.data_n[3];
				pcl::PointNormal& oNewPoint = *pSupportPoint;
				float& fNewConfidence = oNewPoint.data_n[3];

				Eigen::Vector3f oDiffVector(oOldPoint.x - oNewPoint.x, oOldPoint.y - oNewPoint.y, oOldPoint.z - oNewPoint.z);
				Eigen::Vector3f oNewNormal(oNewPoint.normal_x, oNewPoint.normal_y, oNewPoint.normal_z);
				Eigen::Vector3f oProjectDiff = - oDiffVector.dot(oNewNormal) * oNewNormal;

				fOldConfidence += fNewConfidence;
				oOldPoint.x += oProjectDiff.x() * fNewConfidence / fOldConfidence;
				oOldPoint.y += oProjectDiff.y() * fNewConfidence / fOldConfidence;
				oOldPoint.z += oProjectDiff.z() * fNewConfidence / fOldConfidence;
			}

		}
		else if(complict_count > 0) {

			//进一步精细的遮挡检查
			
			pcl::PointNormal& oOldPoint = i < lidar_size ? m_vMapPCN[vPointCloudBuffer[i].data_c[3]] : m_vMapPCNAdded[vPointCloudBuffer[i].data_c[3]];
			float& fOldConfidence = oOldPoint.data_n[3];
			pcl::PointNormal& oNewPoint = *pConflictPoint;
			float& fNewConfidence = oNewPoint.data_n[3];

			Eigen::Vector3f p1(oOldPoint.x - oLidarPos.x, oOldPoint.y - oLidarPos.y, oOldPoint.z - oLidarPos.z);
			Eigen::Vector3f p2(oNewPoint.x - oLidarPos.x, oNewPoint.y - oLidarPos.y, oNewPoint.z - oLidarPos.z);
			Eigen::Vector3f vp2 = p2 / p2.norm();
			Eigen::Vector3f n2(oNewPoint.normal_x, oNewPoint.normal_y, oNewPoint.normal_z);
			Eigen::Vector3f v12 = p2 - p1;

			float r = abs( n2.dot(vp2) * support_factor );
			Eigen::Vector3f vr = v12.dot(vp2) * vp2 - v12;
			float a = p2.norm() - v12.dot(vp2), b = p2.norm();

			if(vr.norm() < r * a / b && v12.dot(n2) < 0) {

				associated_feature[i] = 0.0f;

				// 减小点的置信度
				fOldConfidence -= 0.8 * fNewConfidence;
				if(fOldConfidence < 0.f) fOldConfidence = 0.f;
			}
			else associated_feature[i] = -1.f;

			
			// associated_feature[i] = 0.0f;

		}
		else if(occlusion_count > 0) {

			// associated_feature[i] = 0.6f;
			associated_feature[i] = -1.0f;
		}
		else {
			associated_feature[i] = -1.0f;
		}
	}	

	// clock_t frames_fusion_time3 = 1000.0 * (clock() - start_time3) / CLOCKS_PER_SEC;
	// std::cout << std::format_purple 
	// 	<< "data_associate_time: " << frames_fusion_time3 << "ms" 
	// 	<< std::format_white << std::endl;
	//  input - vPointCloudBuffer, lidar pos, measurement cloud, depth_image/index | output - vCurrentFuseIndex, associated_feature

	// the first to fix - hold about 1/3 of the time (because of the large point set) !!!
	// clock_t start_time5 = clock();

	PublishPointCloud(vPointCloudBuffer, associated_feature, "/debug_associated_point");

	// clock_t frames_fusion_time5 = 1000.0 * (clock() - start_time5) / CLOCKS_PER_SEC;
	// std::cout << std::format_purple
	// 	<< "publish_point_time: " << frames_fusion_time5 << "ms"  
	// 	<< std::format_white << std::endl;

	// 删除被新帧中的被融合点
	// clock_t start_time4 = clock();
	std::vector<float> point_feature(vDepthMeasurementCloud.size(), 0.5f);

	int new_end = vCurrentFuseIndex.size();
	for(int i = new_end - 1; i >= 0; --i) {

		if(vCurrentFuseIndex[i])
		{
			swap(vDepthMeasurementCloud.points[i],vDepthMeasurementCloud.points[--new_end]);
			point_feature[new_end] = 0.9f; 
		}
	}


	// PublishPointCloud(vDepthMeasurementCloud, point_feature, "/current_frame_const");
	vDepthMeasurementCloud.erase(vDepthMeasurementCloud.begin() + new_end, vDepthMeasurementCloud.end());
	std::cout << std::format_blue << "new point num: " << vDepthMeasurementCloud.size() << std::format_white << "\t";

	// clock_t frames_fusion_time4 = 1000.0 * (clock() - start_time4) / CLOCKS_PER_SEC;
	// std::cout << std::format_purple << std::endl
	// 	<< "erase_point_time: " << frames_fusion_time4 << "ms" 
	// 	<< std::format_white << std::endl;

	//*/
}


// ##############################################################################################
// #################################### CloudVector #############################################
// ##############################################################################################

pcl::PointCloud<pcl::PointNormal>& operator+=(pcl::PointCloud<pcl::PointNormal>& vCloudA, const CloudVector& vCloudB) {

    
    for(auto [_, pc] : vCloudB.data) {

        vCloudA += *pc;
    }

    return vCloudA;
}

CloudVector& operator+=(CloudVector &vCloudA, const CloudVector &vCloudB) {

	vCloudA.dirty_flag = true;

	for(auto [seq, pc] : vCloudB.data) {

		if(vCloudA.data.count(seq)) {
		
			*vCloudA.data[seq] += *pc;
		}
		else {

			vCloudA.data[seq] = pc;
		}
	}

	// release frames to keep storage
	if(vCloudA.auto_release_frames) {

		vCloudA.ReleaseFrames();
	}

	return vCloudA;
}

CloudVector& operator+=(CloudVector &vCloudA, const pcl::PointCloud<pcl::PointNormal> vCloudB) {

	// std::cout << "Add frame: " << vCloudB.header.seq << ";\t"
	// 			<< "Past size: " << (vCloudA.data.count(vCloudB.header.seq) ? vCloudA.data[vCloudB.header.seq]->size() : 0) << ";\t"
	// 			<< "After size: " << vCloudB.size() + (vCloudA.data.count(vCloudB.header.seq) ? vCloudA.data[vCloudB.header.seq]->size() : 0) << std::endl;

	vCloudA.dirty_flag = true;

	if(vCloudA.data.count(vCloudB.header.seq)) {
	
		*vCloudA.data[vCloudB.header.seq] += vCloudB;
	}
	else {

		vCloudA.data[vCloudB.header.seq] = vCloudB.makeShared();
	}

	// release frames to keep storage
	if(vCloudA.auto_release_frames) {

		vCloudA.ReleaseFrames();
	}

	// std::cout << "Add seq " << vCloudB.header.seq << " Finish, final size: " << vCloudA.size() << std::endl;

	return vCloudA;
}  

void CloudVector::ComputeSize() {

	if(dirty_flag) {

		dirty_flag = false;
		size_pre_sum.clear();
		seq_record.clear();
		size_pre_sum.reserve(data.size());
		seq_record.reserve(data.size()); 

		for(auto [_,pc] : data) {
			
			unsigned int last_size = size_pre_sum.empty() ? 0 : size_pre_sum.back();
			size_pre_sum.push_back(pc->size() + last_size);
			seq_record.push_back(pc->header.seq);
		}
	}
}

pcl::PointNormal& CloudVector::at(const int index) {

	ComputeSize();
	
	auto iter = upper_bound(size_pre_sum.begin(), size_pre_sum.end(), index);

	// std::cout << "iter: " << (iter != size_pre_sum.end() ? data[seq_record[iter - size_pre_sum.begin()]]->size() - *iter + index : -1) << std::endl;
	
	if(iter == size_pre_sum.end()) {
		
		std::cout << std::format_red << "CloudVector out of range!!!" << std::format_white << std::endl;
		return data[seq_record.back()]->back();
	}
	else return data[seq_record[iter - size_pre_sum.begin()]]->at(data[seq_record[iter - size_pre_sum.begin()]]->size() - *iter + index);
}

void CloudVector::erase(int index) {

	ComputeSize();
	
	dirty_flag = true;
	auto iter = upper_bound(size_pre_sum.begin(), size_pre_sum.end(), index);
	
	if(iter == size_pre_sum.end()) {
		
		std::cout << std::format_red << "CloudVector out of range!!!" << std::format_white << std::endl;
		return;
	}
	swap(
		data[seq_record[iter - size_pre_sum.begin()]]->at(data[seq_record[iter - size_pre_sum.begin()]]->size() - *iter + index), 
		data[seq_record[iter - size_pre_sum.begin()]]->back()
	);
	data[seq_record[iter - size_pre_sum.begin()]]->points.pop_back();
	--data[seq_record[iter - size_pre_sum.begin()]]->width;
}

void CloudVector::ReleaseFrames() {

	int release_num = data.size() - max_window_size;
	std::vector<int> release_seq;

	dirty_flag = true;

	for(auto [seq,pc] : data) {

		if(release_num-- <= 0) break;

		release_seq.push_back(seq);
	}

	for(auto seq : release_seq) {
		
		if(auto_save_frames) {

			SaveFrame(seq);
		}
		data.erase(seq);
	}
}

void CloudVector::SaveFrame(int seq) {

	// std::cout << "release seq: " << seq << std::endl;
	pcl::PointCloud<pcl::PointNormal>::Ptr save_cloud = data[seq];
	std::thread save_thread([save_cloud, seq]() {
		
		system("mkdir -p /tmp/lidar_recon_temp");
		std::stringstream save_path;
		save_path << "/tmp/lidar_recon_temp/frame" << std::setw(6) << std::setfill('0') << seq << ".ply";
		pcl::io::savePLYFileASCII(save_path.str(), *save_cloud);
	});
	save_thread.detach();
}

pcl::PointCloud<pcl::PointNormal>::Ptr AllCloud(CloudVector& cloud_vector) {

    pcl::PointCloud<pcl::PointNormal>::Ptr vOutputCloud(new pcl::PointCloud<pcl::PointNormal>());

    for(auto [_,pc] : cloud_vector.data) {

        *vOutputCloud += *pc;
    }

    return vOutputCloud;
}

pcl::PointCloud<pcl::PointNormal>::Ptr AllCloud(pcl::PointCloud<pcl::PointNormal>& cloud_vector) {

    pcl::PointCloud<pcl::PointNormal>::Ptr vOutputCloud = cloud_vector.makeShared();
    return vOutputCloud;
}
