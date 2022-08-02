#include "FramesFusion.h"
#include <random>
#include <algorithm>

namespace std {
	const char* format_white = "\033[0m";
	const char* format_yellow =  "\033[33m";
	const char* format_red = "\033[31m";
	const char* format_blue = "\033[34m";
}

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
                       ros::NodeHandle & nodeHandle):m_iOdomCount(0), m_dAverageReconstructTime(0), m_dMaxReconstructTime(0),  m_iReconstructFrameNum(0){

	//read parameters
	ReadLaunchParams(nodeHandle);

	//***subscriber related*** 
	//subscribe (hear) the point cloud topic 
	m_oCloudSuber = nodeHandle.subscribe(m_sInCloudTopic, 2, &FramesFusion::HandlePointClouds, this);

	//subscribe (hear) the odometry information (trajectory)
	m_oOdomSuber = nodeHandle.subscribe(m_sInOdomTopic, 1, &FramesFusion::HandleTrajectory, this);

	//***publisher related*** 
	//publish point cloud after processing
	m_oCloudPublisher = nodeHandle.advertise<sensor_msgs::PointCloud2>(m_sOutCloudTopic, 1, true);

  	//publish polygon constructed from one frame point cloud
	m_oMeshPublisher = nodeHandle.advertise<visualization_msgs::Marker>(m_sOutMeshTopic, 1);


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

	std::cout << std::format_blue
	<< "Fused frame numbers: " << m_iReconstructFrameNum << std::endl
	<< "Average recontime per frame: " << m_dAverageReconstructTime / m_iReconstructFrameNum << "ms"
	<< ";\t Max frame time: " << m_dMaxReconstructTime << "ms"
	<< std::format_white << std::endl;

	//define ouput ply file name
	m_sOutPCNormalFileName << m_sFileHead << "Map_PCNormal.ply"; 

    //output to the screen
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << "********************************************************************************" << std::endl;
	std::cout << "Please do not force closing the programe, the process is writing output PLY file." << std::endl;
	std::cout << "It may take times (Writing 500M file takes about 20 seconds in usual)." << std::endl;
	std::cout << "The output file is " << m_sOutPCNormalFileName.str() << std::endl;

	//output point clouds with computed normals to the files when the node logs out
	// m_vMapPCN += m_vMapPCNAdded;
	// pcl::io::savePLYFileASCII(m_sOutPCNormalFileName.str(), m_vMapPCN);

	/** 
		if the point cloud has too many points, ros may not wait it to save.
		to solve this problem, change the file: /opt/ros/kinetic/lib/python2.7/dist-packages/roslaunch/nodeprocess.py
			_TIMEOUT_SIGINT = 15.0  ->  _TIME_OUT_SIGINT = 60.0 
	**/

	std::cout << "Output is complete! The process will be automatically terminated. Thank you for waiting. " << std::endl;

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
 	nodeHandle.param("file_outputpath", m_sFileHead, std::string("./"));

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

	//count processed point cloud frame
	m_iPCFrameCount = 0;

	//true indicates the file has not been generated
	m_bOutPCFileFlag = true;

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
	pcl::fromPCLPointCloud2 (oMeshModel.cloud, vPublishCloud);

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
	color.a = 1;
	color.r = 1.0;
	color.g = 1.0;
	color.b = 0.2;
	
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
        	oMeshMsgs.color = color;

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
			
	for (int i = 0; i != pRawCloud.points.size(); ++i){
		
		if (EuclideanDistance(oBasedP, pRawCloud.points[i]) <= fLength) {
			pNearCloud.push_back(pRawCloud.points[i]);
			pNearCloud.points.rbegin()->data_c[3] = i; //record the rawcloud index in data_c[3]
		}

	}

};

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

void FramesFusion::FusionNormalBackToPoint(const pcl::PointCloud<pcl::PointNormal>& pNearCloud, pcl::PointCloud<pcl::PointNormal> & pRawCloud, int offset, int point_num) {

	MeshOperation m;
	for(int i = offset; i != offset + point_num; ++i) {
		pcl::PointNormal& related_point = pRawCloud.points[pNearCloud.points[i].data_c[3]];
		related_point.normal_x = related_point.normal_x + 0.3 * pNearCloud.points[i].normal_x;
		related_point.normal_y = related_point.normal_y + 0.3 * pNearCloud.points[i].normal_y;
		related_point.normal_z = related_point.normal_z + 0.3 * pNearCloud.points[i].normal_z;
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
void FramesFusion::SurroundModeling(const pcl::PointXYZ & oBasedP, pcl::PolygonMesh & oCBModel){

	//output mesh
	//oCBModel.cloud.clear();
	oCBModel.polygons.clear();

	//get the target points for construction
	pcl::PointCloud<pcl::PointNormal>::Ptr pNearCloud(new pcl::PointCloud<pcl::PointNormal>);

	//based on received point cloud and normal vector
	//get the target point cloud to be modeled
	NearbyClouds(m_vMapPCN, oBasedP, *pNearCloud, m_fNearLengths);
	int point_num = pNearCloud->size();
	NearbyClouds(m_vMapPCNAdded, oBasedP, *pNearCloud, m_fNearLengths);

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
	FusionNormalBackToPoint(*pNearCloud, m_vMapPCN, 0, point_num);
	FusionNormalBackToPoint(*pNearCloud, m_vMapPCNAdded, point_num + 1, pNearCloud->size() - point_num);

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


	//new a mesh operation object for re-order the triangle vertex
	MeshOperation oReOrder;

	//compute the normal vector of each face for its centerpoint
	//the normal vector will be facing away from the viewpoint
	Eigen::MatrixXf oMCMatNormal;
	Eigen::VectorXf vMCfDParam;

	//note a bug releases
	//sometimes the triangular generated by the CB algorithm has two same vertices but does not affect the result
	oReOrder.ComputeAllFaceParams(oBasedP, *pMCResultCloud, oCBModel.polygons, oMCMatNormal, vMCfDParam);

}

void FramesFusion::SurroundModelingWithPointProcessing(const pcl::PointXYZ & oBasedP, pcl::PolygonMesh & oCBModel) {

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


	//new a mesh operation object for re-order the triangle vertex
	MeshOperation oReOrder;

	//compute the normal vector of each face for its centerpoint
	//the normal vector will be facing away from the viewpoint
	Eigen::MatrixXf oMCMatNormal;
	Eigen::VectorXf vMCfDParam;

	//note a bug releases
	//sometimes the triangular generated by the CB algorithm has two same vertices but does not affect the result
	oReOrder.ComputeAllFaceParams(oBasedP, *pMCResultCloud, oCBModel.polygons, oMCMatNormal, vMCfDParam);
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

	if(pFramePN->is_dense == false) {
		m_vMapPCNAdded += *pFramePN;
	}
	else {
		//merge one frame data
		m_vMapPCN += *pFramePN;
		// for(int i = 0; i != pFramePN->points.size(); ++i)
		// {
		// 	m_vMapPCN.push_back(pFramePN->points[i]);
			
		// 	// if(m_vMapPCN.points[i].data_n[3] < 0.1)
		// 	// 	std::cout << m_vMapPCN.points[i].data_n[3] << " ";
		// }
		// std::cout << "handle point finish" << std::endl;

		//count
		m_iPCFrameCount++;
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

	clock_t start_time = clock();
	++m_iReconstructFrameNum;
	SurroundModelingWithPointProcessing(oOdomPoint.oLocation, oNearbyMeshes);
	clock_t frames_fusion_time = 1000.0 * (clock() - start_time) / CLOCKS_PER_SEC;
	std::cout << std::format_blue 
		<< "The No. " << m_iReconstructFrameNum 
		<< ";\tframes_fusion_time: " << frames_fusion_time << "ms" 
		<< std::format_white << std::endl;
	m_dAverageReconstructTime += frames_fusion_time;
	m_dMaxReconstructTime = frames_fusion_time > m_dMaxReconstructTime ? frames_fusion_time : m_dMaxReconstructTime;

	//output the nearby surfaces
	PublishMeshs(oNearbyMeshes);

	PublishPointCloud(m_vMapPCN);
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
	if(!bComputeFlag)
		return;
	
	//if need to be updated
	//get the newest information
	m_oLastModelingTime = oTrajectory.header.stamp;
	int now_frame_num = ++m_iReconstructFrameNum;

	std::thread Modeling([&, now_frame_num]() {

		//save the position of trajectory
		RosTimePoint oOdomPoint;
		oOdomPoint.oLocation.x = oTrajectory.pose.pose.position.x;
		oOdomPoint.oLocation.y = oTrajectory.pose.pose.position.y;
		oOdomPoint.oLocation.z = oTrajectory.pose.pose.position.z;

		//get the reconstructed surfaces
		pcl::PolygonMesh oNearbyMeshes;

		clock_t start_time = clock();
		SurroundModelingWithPointProcessing(oOdomPoint.oLocation, oNearbyMeshes);
		clock_t frames_fusion_time = 1000.0 * (clock() - start_time) / CLOCKS_PER_SEC;
		std::cout << std::format_blue 
			<< "The No. " << now_frame_num
			<< ";\tframes_fusion_time: " << frames_fusion_time << "ms" 
			<< std::format_white << std::endl;
		m_dAverageReconstructTime += frames_fusion_time;
		m_dMaxReconstructTime = frames_fusion_time > m_dMaxReconstructTime ? frames_fusion_time : m_dMaxReconstructTime;

		//output the nearby surfaces
		PublishMeshs(oNearbyMeshes);

		// PublishPointCloud(m_vMapPCN);
	});

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


