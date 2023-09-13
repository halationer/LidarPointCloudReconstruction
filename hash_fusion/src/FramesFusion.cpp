#include "FramesFusion.h"

#include <sstream>
#include <random>
#include <algorithm>
#include <cmath>
#include <exception>
#include <unordered_set>

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
					   m_dAverageFusionTime(0), m_dMaxFusionTime(0), m_iFusionFrameNum(0), m_OdomLoopRate(1),
					   m_oProjectUpdater(ProjectUpdater::GetInstance()),
					   m_oRayUpdater(RayUpdater::GetInstance()),
					   m_oMeshUpdater(MeshUpdater::GetInstance()),
					   m_oRpManager(RosPublishManager::GetInstance()) {

	//read parameters
	ReadLaunchParams(nodeHandle);

	//***subscriber related*** 
	//subscribe (hear) the point cloud topic 
	m_oMeshSuber = nodeHandle.subscribe(m_sInMeshTopic, 5, &FramesFusion::HandleMesh, this);

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

	std::cout << output::format_purple
	<< "Fusion frame numbers: " << m_iFusionFrameNum << std::endl
	<< "Average fusion per frame: " << m_dAverageFusionTime / m_iFusionFrameNum << "ms;\t"
	<< "Max fusion time: " << m_dMaxFusionTime << "ms"
	<< output::format_white << std::endl;

	std::cout << output::format_blue
	<< "Reconstruct frame numbers: " << m_iReconstructFrameNum << std::endl
	<< "Average recontime per frame: " << m_dAverageReconstructTime / m_iReconstructFrameNum << "ms;\t"
	<< "Max frame time: " << m_dMaxReconstructTime << "ms" << std::endl
	<< "Final point nums: " << m_vMapPCN.size() + m_vMapPCNAdded.size() + m_vMapPCNTrueAdded.size()
	<< output::format_white << std::endl;


    //output to the screen
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << "********************************************************************************" << std::endl;

	//output point clouds with computed normals to the files when the node logs out
	if(m_bOutputFiles) {
		
		//output times
		std::stringstream sFuseEvalFileName;
		sFuseEvalFileName << m_sFileHead << "FuseTime.csv";
		std::cout << "The output eval file is " << sFuseEvalFileName.str() << std::endl;
		fuse_timer.OutputDebug(sFuseEvalFileName.str());

		std::stringstream sReconEvalFileName;
		sReconEvalFileName << m_sFileHead << "ReconstructTime.csv";
		std::cout << "The output eval file is " << sReconEvalFileName.str() << std::endl;
		reconstruct_timer.OutputDebug(sReconEvalFileName.str());

		SaveFinalMeshAndPointCloud();
	}

	system("rm -r /tmp/lidar_recon_temp/");
}

/*************************************************
Function: SaveFinalMeshAndPointCloud
Description: save mesh and point cloud of whole scene to files
Called By: ~FramesFusion
*************************************************/
void FramesFusion::SaveFinalMeshAndPointCloud() {

	//define ouput ply file name
	std::stringstream sOutPCNormalFileName;
	sOutPCNormalFileName << m_sFileHead << "Map_PCNormal.ply";

	std::cout << "Please do not force closing the programe, the process is writing output PLY file." << std::endl;
	std::cout << "It may take times (Writing 500M file takes about 20 seconds in usual)." << std::endl;

	if(m_bUseAdditionalPoints) {
	
		m_vMapPCN += m_vMapPCNAdded;
		m_vMapPCN += m_vMapPCNTrueAdded;
	}

	// TODO： output fix

	if(m_bUseUnionSetConnection) {
		m_pVolume->RebuildUnionSetAll(m_fStrictDotRef, m_fSoftDotRef, m_fConfidenceLevelLength);
		m_pVolume->UpdateUnionConflict(m_iRemoveSizeRef, m_fRemoveTimeRef);
	}

	HashVoxeler::HashVolume vVolumeCopy;
	SignedDistance oSDF(-1, m_iConvDim, m_iConvAddPointNumRef, m_fConvFusionDistanceRef1);
	std::unordered_map<HashPos, float, HashFunc> vSignedDis;
	if(m_bUseUnionSetConnection) vSignedDis = oSDF.ConvedGlanceAllUnion(*m_pVolume, m_iRemoveSizeRef);
	else vSignedDis = oSDF.ConvedGlance(*m_pVolume);
	vVolumeCopy = oSDF.m_vVolumeCopy;

	// output mesh static
	{
		pcl::PolygonMesh oResultMesh;
		CIsoSurface<float> oMarchingCuber;
		Eigen::Vector3f vVoxelLength = m_pVolume->GetVoxelLength();
		oMarchingCuber.GenerateSurface(vSignedDis, oSDF.m_vVolumeCopy, 0, vVoxelLength.x(), vVoxelLength.y(), vVoxelLength.z());
		pcl::PointCloud<pcl::PointXYZ>::Ptr pMCResultCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointXYZ oOffset(0, 0, 0);
		oMarchingCuber.OutputMesh(oOffset, oResultMesh, pMCResultCloud);
		for(auto & polygon : oResultMesh.polygons) {
			polygon.vertices.pop_back();
		}
		std::stringstream sOutputPath;
		sOutputPath << m_sFileHead << "final_mesh.ply";
		pcl::io::savePLYFileBinary(sOutputPath.str(), oResultMesh);
		std::cout << output::format_purple << "The output file is " << sOutputPath.str() << output::format_white << std::endl;
	}
	// output mesh dynamic
	{
		SignedDistance oSDF(-1, m_iConvDim, 0, m_fConvFusionDistanceRef1);
		auto vSignedDis = oSDF.ConvedGlanceAll(*m_pVolume);
		
		pcl::PolygonMesh oResultMesh;
		CIsoSurface<float> oMarchingCuber;
		Eigen::Vector3f vVoxelLength = m_pVolume->GetVoxelLength();
		oMarchingCuber.GenerateSurface(vSignedDis, oSDF.m_vVolumeCopy, 0, vVoxelLength.x(), vVoxelLength.y(), vVoxelLength.z());
		pcl::PointCloud<pcl::PointXYZ>::Ptr pMCResultCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointXYZ oOffset(0, 0, 0);
		oMarchingCuber.OutputMesh(oOffset, oResultMesh, pMCResultCloud);
		for(auto & polygon : oResultMesh.polygons) {
			polygon.vertices.pop_back();
		}
		std::stringstream sOutputPath;
		sOutputPath << m_sFileHead << "final_mesh_worm.ply";
		pcl::io::savePLYFileBinary(sOutputPath.str(), oResultMesh);
		std::cout << output::format_purple << "The output file is " << sOutputPath.str() << output::format_white << std::endl;
	}

	// output pc
	auto vAllPoints = AllCloud(m_vMapPCN);
	pcl::PointCloud<pcl::PointNormal> pc; 
	pcl::PointCloud<pcl::PointXYZINormal> static_pc;
	pcl::PointCloud<pcl::PointNormal> dynamic_pc;
	for(int i = 0; i < vAllPoints->size(); ++i) {

		// if the point cloud is too large, open this to random sample
		// if(rand() % 8 > 3) continue;

		std::cout << "All ready process points: " << i+1 << "/" << vAllPoints->size() << "\r";

		pcl::PointNormal& point = (*vAllPoints)[i];
		pc.push_back(point);

		HashPos pos;
		m_pVolume->PointBelongVoxelPos(point, pos);
		if(!vVolumeCopy.count(pos)) {
			dynamic_pc.push_back(point);
			continue;
		}

		// filter inner points
		float distance_ref, normal_ref;
		Eigen::Vector3f oCorner = vVolumeCopy[pos].getVector3fMap();
		Eigen::Vector3f oNormal = vVolumeCopy[pos].getNormalVector3fMap();
		Eigen::Vector3f vPoint = point.getVector3fMap();
		Eigen::Vector3f vNormal = point.getNormalVector3fMap();
		float & token = vVolumeCopy[pos].data_c[1];
		if(vVolumeCopy[pos].data_c[3] > m_fConvFusionDistanceRef1 || token >= __INT_MAX__) {
			distance_ref = m_pVolume->GetVoxelLength().norm() * 0.1;
			normal_ref = 0.8;
		}
		else {
			distance_ref = m_pVolume->GetVoxelLength().norm() * 0.3;
			normal_ref = 0;
		}

		if(abs(oNormal.dot(oCorner - vPoint)) > distance_ref || vNormal.dot(oNormal) < normal_ref) {
			// if(vNormal.dot(oNormal) < normal_ref)
			// 	dynamic_pc.push_back(point);
			continue;
		}

		// put point in final result
		pcl::PointXYZINormal new_point;
		new_point.x = point.x;
		new_point.y = point.y;
		new_point.z = point.z;
		new_point.normal_x = point.normal_x;
		new_point.normal_y = point.normal_y;
		new_point.normal_z = point.normal_z;
		
		// point confidence (impacted by depth & support & conflict)
		new_point.intensity = min(vVolumeCopy[pos].data_n[3], 100.0f); 
		
		// maybe use to save gauss distance, but now don't make scence
		new_point.curvature = vVolumeCopy[pos].data_c[2];

		static_pc.push_back(new_point);
	}
	std::cout << "\nSaving file ..." << std::endl;
	pcl::io::savePLYFileBinary(sOutPCNormalFileName.str(), pc);
	pcl::io::savePLYFileBinary(sOutPCNormalFileName.str()+".static.ply", static_pc);
	pcl::io::savePLYFileBinary(sOutPCNormalFileName.str()+".dynamic.ply", dynamic_pc);
	std::cout << output::format_purple << "The output file is " << sOutPCNormalFileName.str() << output::format_white << std::endl;

	/** 
		if the point cloud has too many points, ros may not wait it to save.
		to solve this problem, change the file: /opt/ros/melodic/lib/python2.7/dist-packages/roslaunch/nodeprocess.py
			DEFAULT_TIMEOUT_SIGINT = 15.0  ->  DEFAULT_TIMEOUT_SIGINT = 60.0 
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

	// volume type
	int iVolumeType;
	nodeHandle.param("volume_type", iVolumeType, static_cast<int>(VolumeBase::VolumeType::HASH_VOXELER));
	m_pVolume.reset(VolumeBase::CreateVolume(static_cast<VolumeBase::VolumeType>(iVolumeType)));
	m_pVolume->InitLog();

 	//output file name
 	nodeHandle.param("file_output_path", m_sFileHead, std::string());
	m_bOutputFiles = !m_sFileHead.empty();

	if(m_bOutputFiles) {

		std::cout << output::format_blue << "mf_output_path:=" << m_sFileHead << output::format_white << std::endl;
		if(m_sFileHead.back() != '/') m_sFileHead += "/";
		std::stringstream sOutputCommand;
		sOutputCommand << "mkdir -p " << m_sFileHead;
		system(sOutputCommand.str().c_str());
		m_sFileHead += "mf_";
	}


 	//input point cloud topic
	nodeHandle.param("mesh_in_topic", m_sInMeshTopic, std::string("/frame_mesh_algo"));

	//input odom topic
  	nodeHandle.param("odom_in_topic", m_sInOdomTopic, std::string("/odometry/filtered"));

	//input odom topic
	nodeHandle.param("cloud_out_topic", m_sOutCloudTopic, std::string("/processed_clouds"));

	//input point cloud topic
	nodeHandle.param("outcloud_tf_id", m_sOutCloudTFId, std::string("map"));
	m_oRpManager.m_sFrameId = m_sOutCloudTFId;
	m_oRpManager.m_pNodeHandle = &nodeHandle;

	//input odom topic
	nodeHandle.param("polygon_out_topic", m_sOutMeshTopic, std::string("/surrounding_meshes"));

	//input point cloud topic
	nodeHandle.param("polygon_tf_id", m_sOutMeshTFId, std::string("map"));

	//nearbt lengths
	nodeHandle.param("voxel_total_size", m_fNearLengths, 20.0f);
	m_fNearLengths = m_fNearLengths / 2.0f;

	//point cloud sampling number
	nodeHandle.param("sample_pcframe_num", m_iFrameSmpNum, 1);

	//point cloud sampling number
	nodeHandle.param("sample_inputpoints_num", m_iSampleInPNum, 1);

	//nearby mesh update period in second
	nodeHandle.param("mesh_update_period", m_fNearMeshPeriod, 2.0f);

	//side length of cube (voxel)
  	float fCubeSize;
  	nodeHandle.param("voxel_cube_size", fCubeSize, 0.5f);
 	m_oVoxelResolution = pcl::PointXYZ(fCubeSize, fCubeSize, fCubeSize);
	m_pVolume->SetResolution(m_oVoxelResolution);

	int eStrategyType;
  	nodeHandle.param("strategy_type", eStrategyType, static_cast<int>(eEmptyStrategy));
	m_pVolume->SetStrategy(static_cast<vus>(eStrategyType));

	//use surfel fusion?
	nodeHandle.param("use_surfel_fusion", m_bSurfelFusion, true);
	// use additional points for reconstruction?
	nodeHandle.param("additional_points", m_bUseAdditionalPoints, true);

	nodeHandle.param("async_reconstruct", m_bAsyncReconstruction, true);
	nodeHandle.param("use_union_set", m_bUseUnionSetConnection, true);
	nodeHandle.param("only_max_union_set", m_bOnlyMaxUnionSet, true);
	nodeHandle.param("recon_range", m_fReconstructRange, 30.0f);
	nodeHandle.param("strict_dot_ref", m_fStrictDotRef, 0.95f);
	nodeHandle.param("soft_dot_ref", m_fSoftDotRef, 0.3f);
	nodeHandle.param("remove_size_ref", m_iRemoveSizeRef, 200);
	nodeHandle.param("remove_time_ref", m_fRemoveTimeRef, 10.0f);
	nodeHandle.param("confidence_level_length", m_fConfidenceLevelLength, 8.0f);
	nodeHandle.param("center_based_recon", m_bCenterBasedRecon, false);

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
	nodeHandle.param("dynamic_debug", m_bDynamicDebug, false);
	nodeHandle.param("keep_voxel", m_bKeepVoxel, false);
	// m_pSdf = new SignedDistance(m_iKeepTime, m_iConvDim, m_iConvAddPointNumRef, m_fConvFusionDistanceRef1);
	// m_pVolume->m_iMaxRecentKeep = max(500u, (uint32_t)m_iKeepTime);

	return true;

}

void InitMeshMsg(visualization_msgs::Marker& oMeshMsgs, string frame_id, int id, float r, float g, float b) {
	
	oMeshMsgs.header.frame_id = frame_id;
	oMeshMsgs.header.stamp = ros::Time::now();
	oMeshMsgs.type = visualization_msgs::Marker::TRIANGLE_LIST;
	oMeshMsgs.action = visualization_msgs::Marker::MODIFY;
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
Function: PublishMeshs
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
	// InitMeshMsg(oMeshFused,		m_sOutMeshTFId, 3, 1. , 0.95, 0.2);
	// InitMeshMsg(oMeshAdded, 	m_sOutMeshTFId, 2, 1. , 0.9, 0.2);

	// dynamic
	// InitMeshMsg(oMeshAdded, 	m_sOutMeshTFId, 2, 1. , 0.95, 0.2);
	// InitMeshMsg(oMeshFused,		m_sOutMeshTFId, 3, 1. , 0.95, 0.2);

	// add
	InitMeshMsg(oMeshAdded, 	m_sOutMeshTFId, 2, 0.4, 0.6, 1. );

	// fused
	InitMeshMsg(oMeshFused,		m_sOutMeshTFId, 3, 0.9, 1. , 0.2);

	//for each face
	for (int i = 0; i != oMeshModel.polygons.size(); ++i){

		uint32_t mesh_type = oMeshModel.polygons[i].vertices.back();
		
		//for each face vertex id
		for (int j = 0; j != 3; ++j){

			//vertex id in each sector
			int iVertexIdx =  oMeshModel.polygons[i].vertices[j];

			//temp point
    		geometry_msgs::Point oPTemp;
        	oPTemp.x = vPublishCloud.points[iVertexIdx].x;
        	oPTemp.y = vPublishCloud.points[iVertexIdx].y;
        	oPTemp.z = vPublishCloud.points[iVertexIdx].z;

			if(mesh_type >= __INT_MAX__) {
				oMeshAdded.points.push_back(oPTemp);
				// oMeshMsgs.points.push_back(oPTemp);
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
	m_oRpManager.PublishPointCloud(*pProcessedCloud, temp_feature, "/temp_near_cloud");


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
	m_pVolume->VoxelizePointsAndFusion(*pProcessedCloud);

	//******voxelization********
	//using signed distance
	SignedDistance oSDer(m_iKeepTime, m_iConvDim, m_iConvAddPointNumRef, m_fConvFusionDistanceRef1);

	//compute signed distance based on centroids and its normals within voxels
	std::unordered_map<HashPos, float, HashFunc> vSignedDis = oSDer.ConvedGlance(*m_pVolume);

	//******construction********
	//marching cuber
	CIsoSurface<float> oMarchingCuber;
	//get surface mesh based on voxel related algorithm
	Eigen::Vector3f vVoxelLength = m_pVolume->GetVoxelLength();
	oMarchingCuber.GenerateSurface(vSignedDis, oSDer.m_vVolumeCopy, 0, vVoxelLength.x(), vVoxelLength.y(), vVoxelLength.z());

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

// TODO: Start from this, 主要包含 传入SigedDistance能力，传入MC能力，构建连通集的能力

/*************************************************
Function: SlideModeling
Description: Build surface models based on new points that received from ros
Called By: 
Input:	m_vNewPoints - new points that received from ros
Output: oResultMesh - result mesh of the surface model
		m_vMapPCN - the fused points is added to m_vMapPCN
*************************************************/
void FramesFusion::SlideModeling(pcl::PolygonMesh & oResultMesh, const Eigen::Vector3f& vCenter, const int iFrameId) {
	
	TimeDebuggerProxy timer(iFrameId, &reconstruct_timer);

	//clear mesh
	//oResultMesh.cloud.clear();
	oResultMesh.polygons.clear();

	timer.DebugTime("1_clear_mesh");

	//******make mesh********
	//check connection
	if(m_bUseUnionSetConnection) {
		m_pVolume->RebuildUnionSet(m_fStrictDotRef, m_fSoftDotRef, m_fConfidenceLevelLength);
		m_pVolume->UpdateUnionConflict(m_iRemoveSizeRef, m_fRemoveTimeRef);
		timer.DebugTime("2_main_connect");

		// output unionset result
		visualization_msgs::MarkerArray union_set_marker;
		m_pVolume->DrawUnionSet(union_set_marker);
		m_oRpManager.PublishMarkerArray(union_set_marker, "/union_set");
		timer.DebugTime("3_publish_connect");
	}

	//using signed distance
	SignedDistance oSDer(m_iKeepTime, m_iConvDim, m_iConvAddPointNumRef, m_fConvFusionDistanceRef1);
	//compute signed distance based on centroids and its normals within voxels
	std::unordered_map<HashPos, float, HashFunc> vSignedDis;
	// std::unique_ptr<visualization_msgs::MarkerArray> pStaticDebug(new visualization_msgs::MarkerArray);
	std::unique_ptr<visualization_msgs::MarkerArray> pStaticDebug(nullptr);
	// choose which mode to use
	if(m_bDynamicDebug) {
		vSignedDis = oSDer.DebugGlance(*m_pVolume);
	}
	else if(m_bCenterBasedRecon) {
		vSignedDis = oSDer.CenterBasedGlance(*m_pVolume, vCenter, m_fNearLengths, m_iRemoveSizeRef, pStaticDebug.get());
	}
	else if(!m_bUseUnionSetConnection) {
		vSignedDis = oSDer.ConvedGlance(*m_pVolume, pStaticDebug.get());
	}
	else if(m_bOnlyMaxUnionSet) {
		vSignedDis = oSDer.ConvedGlanceOnlyMaxUnion(*m_pVolume, pStaticDebug.get());
	}
	else {
		vSignedDis = oSDer.ConvedGlanceLargeUnion(*m_pVolume, m_iRemoveSizeRef, pStaticDebug.get());
	}
	// publish intermediate result
	if(pStaticDebug != nullptr) {
		m_oRpManager.PublishMarkerArray(*pStaticDebug, "/static_expand");
	}

	timer.DebugTime("4_main_conv");

	//marching cuber
	CIsoSurface<float> oMarchingCuber;
	//get surface mesh based on voxel related algorithm
	Eigen::Vector3f vVoxelLength = m_pVolume->GetVoxelLength();
	oMarchingCuber.GenerateSurface(vSignedDis, oSDer.m_vVolumeCopy, 0, vVoxelLength.x(), vVoxelLength.y(), vVoxelLength.z());

	//move the mesh position to the actual starting point
	pcl::PointCloud<pcl::PointXYZ>::Ptr pMCResultCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZ oOffset(0, 0, 0);
	oMarchingCuber.OutputMesh(oOffset, oResultMesh, pMCResultCloud);
	
	timer.DebugTime("5_marching_cubes");

	///* output result mesh
	if(m_bOutputFiles) {

		pcl::PolygonMesh oStaticMesh, oDynamicMesh;
		oStaticMesh.cloud = oResultMesh.cloud;
		oDynamicMesh.cloud = oResultMesh.cloud;

		std::stringstream sOutputPath;
		sOutputPath << m_sFileHead << std::setw(4) << std::setfill('0') << iFrameId << "_mesh.ply";
		auto oCopyMesh = oResultMesh;
		for(auto & polygon : oCopyMesh.polygons) {
			uint32_t token = polygon.vertices.back();
			polygon.vertices.pop_back();
			if(token < 2) oStaticMesh.polygons.push_back(polygon);
			else oDynamicMesh.polygons.push_back(polygon);
		}
			
		pcl::io::savePLYFileBinary(sOutputPath.str(), oCopyMesh);

		if(m_bDynamicDebug) {
			pcl::io::savePLYFileBinary(sOutputPath.str() + ".static.ply", oStaticMesh);
			pcl::io::savePLYFileBinary(sOutputPath.str() + ".dyamic.ply", oDynamicMesh);
		}
		timer.DebugTime("6_output_file");
	}
	//*/
	timer.GetCurrentLineTime();
}


void FramesFusion::HandleMesh(const shape_msgs::Mesh & vMeshRosData)
{
	++m_iFusionFrameNum;

	fuse_timer.NewLine();

	pcl::PolygonMesh::Ptr pFrameMesh(new pcl::PolygonMesh);
	pcl::PointCloud<pcl::PointXYZ> oCloud;
	for(auto&& point : vMeshRosData.vertices) {
		oCloud.push_back(pcl::PointXYZ(point.x, point.y, point.z));
	}
	pcl::toPCLPointCloud2(oCloud, pFrameMesh->cloud);
	for(auto&& triangle : vMeshRosData.triangles) {
		pcl::Vertices oVertices;
		oVertices.vertices.push_back(triangle.vertex_indices[0]);
		oVertices.vertices.push_back(triangle.vertex_indices[1]);
		oVertices.vertices.push_back(triangle.vertex_indices[2]);
		pFrameMesh->polygons.push_back(oVertices);
	}

	fuse_timer.DebugTime("1_transfer_mesh");

	// m_oProjectUpdater.SurfelFusionQuick(oViewPoint, *pFramePN, *m_pVolume, m_bDynamicDebug || m_bKeepVoxel);
	// m_oRayUpdater.RayFusion(oViewPoint, *pFramePN, *m_pVolume, m_bDynamicDebug || m_bKeepVoxel);
	m_oMeshUpdater.MeshFusion(pcl::PointNormal(), *pFrameMesh, *m_pVolume, m_bDynamicDebug || m_bKeepVoxel);

	double frames_fusion_time = fuse_timer.DebugTime("2_main_fusion");
	
	std::cout << output::format_purple 
		<< "The No. " << m_iFusionFrameNum 
		<< ";\tframes_fusion_time: " << frames_fusion_time << "ms" 
		<< output::format_white;
	m_dAverageFusionTime += frames_fusion_time;
	m_dMaxFusionTime = frames_fusion_time > m_dMaxFusionTime ? frames_fusion_time : m_dMaxFusionTime;

	//merge one frame data
	// UpdateOneFrame(oViewPoint, *pFramePN);
	// double voxelize_time = fuse_timer.DebugTime("3_main_voxelize");
	// std::cout << output::format_blue << ";\tvoxelize_time: " << voxelize_time << "ms" << output::format_white;
	std::cout << std::endl;


	// // output point cloud with (depth & view) confidence
	// if(m_bSurfelFusion) {
		
	// 	pcl::PointCloud<pcl::PointNormal> temp;
	// 	m_pVolume->GetVolumeCloud(temp);
	// 	vector<float> confidence(temp.size());
	// 	constexpr float confidence_scalar = 0.8f;
	// 	for(int i = 0; i < confidence.size(); ++i) {
	// 		confidence[i] = int(temp.at(i).data_n[3] / m_fConfidenceLevelLength) * 0.2;
	// 		if(confidence[i] > confidence_scalar) confidence[i] = confidence_scalar;
	// 		if(temp[i].data_c[1] > 1 || temp[i].data_n[3] == .0f) confidence[i] = -1;
	// 	}
	// 	m_oRpManager.PublishPointCloud(temp, confidence, "/all_cloud_confidence");
	// 	fuse_timer.DebugTime("4_debug_publish");
	// }

	fuse_timer.GetCurrentLineTime();
}


void FramesFusion::UpdateOneFrame(const pcl::PointNormal& oViewPoint, pcl::PointCloud<pcl::PointNormal>& vFilteredMeasurementCloud) {

	///* limit the distance
	int n = vFilteredMeasurementCloud.size();
	for(int i = 0; i < n; ++i) {
		auto & oPoint = vFilteredMeasurementCloud[i];
		Eigen::Vector3f vDistance(oViewPoint.x - oPoint.x, oViewPoint.y - oPoint.y, oViewPoint.z - oPoint.z);
		if(vDistance.norm() > m_fReconstructRange) {
			swap(vFilteredMeasurementCloud[i--], vFilteredMeasurementCloud[--n]);
		}
	}
	vFilteredMeasurementCloud.erase(vFilteredMeasurementCloud.begin()+n, vFilteredMeasurementCloud.end());
	//*/

	m_pVolume->VoxelizePointsAndFusion(vFilteredMeasurementCloud);
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
	m_pVolume->UpdateLidarCenter(oLidarPos);

	//get the reconstructed surfaces
	pcl::PolygonMesh oNearbyMeshes;

	// Mesh Generate
	clock_t start_time = clock();

	// if(!m_bSurfelFusion && m_bUseAdditionalPoints) 
	// 	SurroundModelingWithPointProcessing(oOdomPoint.oLocation, oNearbyMeshes, m_iReconstructFrameNum);
	// else 
	SlideModeling(oNearbyMeshes, oLidarPos, m_iReconstructFrameNum);
	clock_t frames_fusion_time = 1000.0 * (clock() - start_time) / CLOCKS_PER_SEC;

	++m_iReconstructFrameNum;
	std::cout << output::format_blue 
		<< "The No. " << m_iReconstructFrameNum 
		<< ";\tframes_fusion_time: " << frames_fusion_time << "ms" 
		<< output::format_white << std::endl;
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
	m_pVolume->UpdateLidarCenter(oLidarPos);

	//if need to be updated
	//get the newest information
	int now_frame_num = m_iReconstructFrameNum++;

	auto ModelingFunction = [&, now_frame_num, oLidarPos]() {

		std::cout << output::format_purple << "No. " << now_frame_num << " reconstruct start" << output::format_white << std::endl;

		//get the reconstructed surfaces
		pcl::PolygonMesh oResultMeshes;

		struct timeval start;
		gettimeofday(&start, NULL);

		// if(!m_bSurfelFusion && m_bUseAdditionalPoints) 
		// 	SurroundModelingWithPointProcessing(oOdomPoint.oLocation, oResultMeshes, now_frame_num);
		// else 
		SlideModeling(oResultMeshes, oLidarPos, now_frame_num);

		struct timeval end;
		gettimeofday(&end,NULL);
		double frame_reconstruct_time = (end.tv_sec - start.tv_sec) * 1000.0 +(end.tv_usec - start.tv_usec) * 0.001;
		std::cout << output::format_blue 
			<< "The No. " << now_frame_num
			<< ";\tframes_reconstruct_time: " << frame_reconstruct_time << "ms" 
			<< ";\tgen_face_num: " << oResultMeshes.polygons.size()
			<< output::format_white << std::endl;
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

pcl::PointCloud<pcl::PointNormal>::Ptr AllCloud(pcl::PointCloud<pcl::PointNormal>& cloud_vector) {

    return cloud_vector.makeShared();
}
