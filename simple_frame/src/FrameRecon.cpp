#include "FrameRecon.h"
#include "OutputUtils.h"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <pcl/io/ply_io.h>

/*************************************************
Function: FrameRecon
Description: constrcution function for FrameRecon class
Calls: all member functions
Called By: main function of project
Table Accessed: none
Table Updated: none
Input: node - a ros node class
     nodeHandle - a private ros node class
*************************************************/
FrameRecon::FrameRecon(ros::NodeHandle & node,
                       ros::NodeHandle & nodeHandle):
                       m_iTrajFrameNum(0),m_dAverageReconstructTime(0),m_iReconstructFrameNum(0),m_dMaxReconstructTime(0),node(node),nodeHandle(nodeHandle){
}

void FrameRecon::LazyLoading() {

    std::cout << "Load GHPR Reconstruction..." << std::endl;

	//read parameters
	ReadLaunchParams(nodeHandle);

	//***subscriber related*** 
	//subscribe (hear) the odometry information (trajectory)
	m_oOdomSuber = nodeHandle.subscribe(m_sInOdomTopic, 2, &FrameRecon::HandleTrajectory, this);	//记录运动信息到 m_vOdomHistory 的循环数组之中，并且 m_iTrajCount++

	//subscribe (hear) the point cloud topic 
	m_oCloudSuber = nodeHandle.subscribe(m_sInCloudTopic, 1, &FrameRecon::HandlePointClouds, this);	//在m_vMapPCN记录法向点集，发布Mesh主题

	//***publisher related*** 
	//publish point cloud after processing
	m_oCloudPublisher = nodeHandle.advertise<sensor_msgs::PointCloud2>(m_sOutCloudTopic, 1, true);	//暂无发布

  	//publish polygon constructed from one frame point cloud
	m_oMeshPublisher = nodeHandle.advertise<visualization_msgs::Marker>(m_sOutMeshTopic, 1, true);		//在接受到点云重建完之后， 被 PublishMeshs() 函数调用

    m_oAdditionalPointPublisher = nodeHandle.advertise<sensor_msgs::PointCloud2>(m_sAdditionalPointTopic, 1, true); //发布补充的点云
}

/*************************************************
Function: ~FrameRecon
Description: deconstrcution function for FrameRecon class
Calls: all member functions
Called By: main function of project
Table Accessed: none
Table Updated: none
Input: none
Output: a file storing the point clouds with correct normal for accurate reconstruction
*************************************************/

FrameRecon::~FrameRecon() {

	std::cout << std::format_yellow 
		<< "Reconstructed frame numbers: " << m_iReconstructFrameNum << ";\tTotal frame numbers : " << m_iTotalFrameNum << std::endl
		<< "Average recontime per frame: " << m_dAverageReconstructTime / m_iReconstructFrameNum << "ms"
		<< ";\t Max frame time: " << m_dMaxReconstructTime << "ms"
		<< std::format_white << std::endl;

	/* TODO: output raw point cloud
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
	pcl::io::savePLYFileASCII(m_sOutPCNormalFileName.str(), m_vMapPCN);

	std::cout << "Output is complete! The process will be automatically terminated. Thank you for waiting. " << std::endl;
	//*/

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

bool FrameRecon::ReadLaunchParams(ros::NodeHandle & nodeHandle) {

	//output file name
	node.param("sf_output_path", m_sFileHead, std::string());
	if(m_sFileHead.empty())
		nodeHandle.param("file_output_path", m_sFileHead, std::string(""));
	m_bOutputFiles = !m_sFileHead.empty();

	if(m_bOutputFiles) {

		if(m_sFileHead.back() != '/') m_sFileHead += "/";
		std::stringstream sOutputCommand;
		sOutputCommand << "mkdir -p " << m_sFileHead;
		system(sOutputCommand.str().c_str());
		m_sFileHead += "sf_";
	}


	//input odom topic
	nodeHandle.param("odom_in_topic", m_sInOdomTopic, std::string("/odometry/filtered"));

	//input point cloud topic
	nodeHandle.param("cloud_in_topic", m_sInCloudTopic, std::string("/cloud_points"));


	//input odom topic
	nodeHandle.param("cloud_out_topic", m_sOutCloudTopic, std::string("/processed_clouds"));

	//input point cloud topic
	nodeHandle.param("outcloud_tf_id", m_sOutCloudTFId, std::string("camera_init"));

	//input odom topic
	nodeHandle.param("polygon_out_topic", m_sOutMeshTopic, std::string("/processed_clouds"));

	//input point cloud topic
	nodeHandle.param("polygon_tf_id", m_sOutMeshTFId, std::string("camera_init"));

	//point cloud sampling number
	nodeHandle.param("sample_pcframe_num", m_iFrameSmpNum, 1);

	//point cloud sampling number
	nodeHandle.param("sample_inputpoints_num", m_iSampleInPNum, 1);

	nodeHandle.param("lidar_line_min", m_iLidarLineMin, 0);
	nodeHandle.param("lidar_line_max", m_iLidarLineMax, 15);

	//height of viewpoint
	double dViewZOffset;
	nodeHandle.param("viewp_zoffset", dViewZOffset, 0.0);
	m_fViewZOffset = float(dViewZOffset);
	
	//explicit reconstruction related
	//number of sectors
	nodeHandle.param("sector_num", m_iSectorNum, 1);
	m_oExplicitBuilder.HorizontalSectorSize(m_iSectorNum);

	bool bMultiThread;
	nodeHandle.param("multi_thread", bMultiThread, true);
	m_oExplicitBuilder.SetMultiThread(bMultiThread);

	//count processed point cloud frame
	m_iPCFrameCount = 0;

	//count processed odom frame
	m_iTrajCount = 0;

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
void FrameRecon::PublishPointCloud(const pcl::PointCloud<pcl::PointXYZI> & vCloud){
  
	//publish obstacle points
	sensor_msgs::PointCloud2 vCloudData;

	pcl::toROSMsg(vCloud, vCloudData);

	vCloudData.header.frame_id = m_sOutCloudTFId;

	vCloudData.header.stamp = ros::Time::now();

	m_oCloudPublisher.publish(vCloudData);

}


/*************************************************
Function: PublishPointCloud
Description: publish point clouds (mainly used for display and test)
Calls: none
Called By: ComputeConfidence()
Table Accessed: none
Table Updated: none
Input: vCloud - a point clouds with its corresponding normals for publication
Output: none
Return: none
Others: none
*************************************************/
void FrameRecon::PublishPointCloud(const pcl::PointCloud<pcl::PointNormal> & vCloudNormal){

    //convert to pc2 message
	sensor_msgs::PointCloud2 vCloudData;

	pcl::toROSMsg(vCloudNormal, vCloudData);

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
void FrameRecon::PublishPointCloud(const pcl::PointCloud<pcl::PointXYZI> & vCloud, const std::vector<float> & vFeatures){
  
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

template<class T>
void FrameRecon::PublishPointCloud(pcl::PointCloud<T>& pointcloud, ros::Publisher& publisher)
{
    sensor_msgs::PointCloud2 vCloudData;

	pcl::toROSMsg(pointcloud, vCloudData);

	vCloudData.header.frame_id = m_sOutCloudTFId;

	vCloudData.header.stamp = ros::Time::now();

	publisher.publish(vCloudData);
}
template void FrameRecon::PublishPointCloud(pcl::PointCloud<pcl::PointXYZI>& pointcloud, ros::Publisher& publisher);
template void FrameRecon::PublishPointCloud(pcl::PointCloud<pcl::PointNormal>& pointcloud, ros::Publisher& publisher);

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
void FrameRecon::PublishMeshs(){
  	
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
	color.r = 200 / 255.f * 1.5f;
	color.g = 128 / 255.f * 1.5f;
	color.b = 54  / 255.f * 1.5f;
	
	color.r = 248 / 255.f;
	color.g = 220 / 255.f;
	color.b = 180 / 255.f;

	//repeatable vertices
	pcl::PointCloud<pcl::PointXYZI> vMeshVertices;

	//get the reconstruted mesh
	m_oExplicitBuilder.OutputAllMeshes(vMeshVertices);

	//convert to publishable message
	for (int k = 0; k < vMeshVertices.points.size(); ++k){

		//temp point
    	geometry_msgs::Point oPTemp;
        oPTemp.x = vMeshVertices.points[k].x;
        oPTemp.y = vMeshVertices.points[k].y;
        oPTemp.z = vMeshVertices.points[k].z;

        //color
        oMeshMsgs.points.push_back(oPTemp);
        oMeshMsgs.color = color;

	}//end k

	m_oMeshPublisher.publish(oMeshMsgs);

}

std::ostream& operator<<(std::ostream& out, const sensor_msgs::PointCloud2::_header_type& header) {
	out << header.frame_id << ", " << header.seq << ", " << header.stamp;
	return out;
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
void FrameRecon::HandlePointClouds(const sensor_msgs::PointCloud2 & vLaserData)
{

	if (!(m_iPCFrameCount % m_iFrameSmpNum)){ //根据帧采样频率记录

		std::cout << "Now frame count is: " << m_iPCFrameCount << ";\t"
			<< "header is: {" << vLaserData.header << "}";
		m_iTotalFrameNum = vLaserData.header.seq + 1;

		struct timeval start;
		gettimeofday(&start, NULL);

		////a point clouds in PCL type
		pcl::PointCloud<pcl::PointXYZI>::Ptr pRawCloud(new pcl::PointCloud<pcl::PointXYZI>);
		////message from ROS type to PCL type
		pcl::fromROSMsg(vLaserData, *pRawCloud);

		// /*TODO: out put raw point cloud
		if(m_bOutputFiles) {
			
			std::stringstream filename;
			filename << m_sFileHead << std::setw(4) << std::setfill('0') << m_iReconstructFrameNum << "_pc.ply";
			pcl::io::savePLYFileASCII(filename.str(), *pRawCloud);
		}
		//*/
		
		//if have corresponding trajectory point (viewpoint)
		pcl::PointXYZI oCurrentViewP;

		if (m_vOdomHistory.size() /*&& vLaserData.header.stamp <= m_vOdomHistory.last().oTimeStamp*/){
			//
			oCurrentViewP = ComputeQueryTraj(vLaserData.header.stamp);	//当前点云对应的观测位置（Odom与frame并非一一对应，因此需要计算插值）

			// std::cout << "\tView: " << oCurrentViewP;
		
		//else waiting for sync
		}else{

			std::cout << std::format_red << " Error: No odom matched!" << std::format_white << std::endl;

			return;
		}

		std::cout << ";\tsize: " << pRawCloud->size();
		

		// point sample
		pcl::PointCloud<pcl::PointXYZI>::Ptr pSceneCloud(new pcl::PointCloud<pcl::PointXYZI>);
		SamplePoints(*pRawCloud, *pSceneCloud, m_iSampleInPNum);
		

		// frame reconstruct
		struct timeval reconstruct_start;
		gettimeofday(&reconstruct_start, NULL);

		pcl::PointCloud<pcl::PointNormal>::Ptr pFramePNormal(new pcl::PointCloud<pcl::PointNormal>);
		m_oExplicitBuilder.setWorkingFrameCount(m_iPCFrameCount);
		m_oExplicitBuilder.SetViewPoint(oCurrentViewP, m_fViewZOffset);
		m_oExplicitBuilder.FrameReconstruction(*pSceneCloud, *pFramePNormal, m_iLidarLineMin, m_iLidarLineMax);	//得到带法向的点云

		struct timeval reconstruct_end;
		gettimeofday(&reconstruct_end,NULL);
		double parallel_time = (reconstruct_end.tv_sec - reconstruct_start.tv_sec) * 1000.0 +(reconstruct_end.tv_usec - reconstruct_start.tv_usec) * 0.001;
		std::cout << "\trecon_time:" << parallel_time << "ms";

		//************output value******************
		// for(int i=0;i!=pFramePNormal->points.size();++i)
		// 	m_vMapPCN.points.push_back(pFramePNormal->points[i]);
		

		//************additional points**************
        pcl::PointCloud<pcl::PointNormal> vAdditionalPoints;
        pcl::PointCloud<pcl::PointXYZI> vDisplayAdditionalPoints;
		for(int i = 0; i < m_oExplicitBuilder.m_vAllSectorClouds.size(); ++i) {
			MeshSample::GetAdditionalPointCloud(
				*(m_oExplicitBuilder.m_vAllSectorClouds[i]) , m_oExplicitBuilder.m_vAllSectorFaces[i], 
				m_oExplicitBuilder.m_vFaceWeight[i], m_oExplicitBuilder.m_vMatNormal[i],
				vAdditionalPoints, vDisplayAdditionalPoints
			);
		}

		// publish time
		PublishPointCloud(vDisplayAdditionalPoints, m_oAdditionalPointPublisher);

		// 添加中心视点，方便多帧进程识别
		pcl::PointNormal oViewPoint;
		oViewPoint.x = oCurrentViewP.x;
		oViewPoint.y = oCurrentViewP.y;
		oViewPoint.z = oCurrentViewP.z;
		oViewPoint.curvature = -1;      //识别码
		pFramePNormal->push_back(oViewPoint);
		vAdditionalPoints.push_back(oViewPoint);

		// *pFramePNormal += vAdditionalPoints;
        vAdditionalPoints.is_dense = false;
        PublishPointCloud(vAdditionalPoints, m_oCloudPublisher);

		PublishMeshs();	//发布 m_oExplicitBuilder 中建立的 mesh
		
		///* output mesh file:
		if(m_bOutputFiles) {

			pcl::PolygonMesh oFullMesh;
			pcl::PointCloud<pcl::PointXYZI> vFullCloud;
			for(int sector_index = 0; sector_index < m_oExplicitBuilder.m_vAllSectorClouds.size(); ++sector_index) {

				// pcl::PolygonMesh oCurrentMesh;
				// pcl::toPCLPointCloud2(*m_oExplicitBuilder.m_vAllSectorClouds[sector_index], oCurrentMesh.cloud);
				// oCurrentMesh.polygons.assign(m_oExplicitBuilder.m_vAllSectorFaces[sector_index].begin(), m_oExplicitBuilder.m_vAllSectorFaces[sector_index].end());
				// std::stringstream sOutputPath;
				// sOutputPath << m_sFileHead << m_iReconstructFrameNum << "_" << sector_index << ".ply";
				// pcl::io::savePLYFileBinary(sOutputPath.str(), oCurrentMesh); 

				for(int face_index = 0; face_index < m_oExplicitBuilder.m_vAllSectorFaces[sector_index].size(); ++face_index) {

					pcl::Vertices oCurrentFace;
					oCurrentFace.vertices.push_back(m_oExplicitBuilder.m_vAllSectorFaces[sector_index][face_index].vertices[0] + vFullCloud.size());
					oCurrentFace.vertices.push_back(m_oExplicitBuilder.m_vAllSectorFaces[sector_index][face_index].vertices[1] + vFullCloud.size());
					oCurrentFace.vertices.push_back(m_oExplicitBuilder.m_vAllSectorFaces[sector_index][face_index].vertices[2] + vFullCloud.size());
					oFullMesh.polygons.push_back(oCurrentFace);
				} 
				vFullCloud += *m_oExplicitBuilder.m_vAllSectorClouds[sector_index];
			}
			pcl::toPCLPointCloud2(vFullCloud, oFullMesh.cloud);

			std::stringstream sOutputPath;
			sOutputPath << m_sFileHead << std::setw(4) << std::setfill('0') << m_iReconstructFrameNum << "_mesh.ply";
			pcl::io::savePLYFileBinary(sOutputPath.str(), oFullMesh); 
		}
		
		//*/

		PublishPointCloud(*pFramePNormal);

		/*output the points and normals
		{
			std::stringstream ss;
			ss << "../Dense_ROS/save/FramePNormal_" << m_iPCFrameCount << ".ply";
			pcl::io::savePLYFileASCII(ss.str(), *pFramePNormal);
		}
		//*/

		//clear this frame result
		m_oExplicitBuilder.ClearData();


		//结束算法计时并记录执行时间
		struct timeval end;
		gettimeofday(&end,NULL);
		double frame_reconstruct_time = (end.tv_sec - start.tv_sec) * 1000.0 +(end.tv_usec - start.tv_usec) * 0.001;
		std::cout << ";\tframe_time:" << frame_reconstruct_time << "ms" << std::endl;
		m_dAverageReconstructTime += frame_reconstruct_time;
		m_dMaxReconstructTime = frame_reconstruct_time > m_dMaxReconstructTime ? frame_reconstruct_time : m_dMaxReconstructTime;
		++m_iReconstructFrameNum;
	}

	//count
	m_iPCFrameCount++;

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
void FrameRecon::HandleTrajectory(const nav_msgs::Odometry & oTrajectory)
{

	std::cout << std::format_yellow << "Now Odome count is: " << m_iTrajCount << ";\t"
		<< "header is: {" << oTrajectory.header << "}" << "\tPose: (" << oTrajectory.pose.pose.position.x << ","
		<< oTrajectory.pose.pose.position.y << "," << oTrajectory.pose.pose.position.z << ")" << std::format_white << std::endl;

	//count input frames
	//m_iTrajPointNum++;

	//save the into the memory
	//save the position of trajectory
	RosTimePoint oOdomPoint;
	oOdomPoint.oLocation.x = oTrajectory.pose.pose.position.x;
	oOdomPoint.oLocation.y = oTrajectory.pose.pose.position.y;
	oOdomPoint.oLocation.z = oTrajectory.pose.pose.position.z;

	// oTrajectory.twist.twist.angular/linear 表示角速度和线速度
	/**
	 * 一. odom和imu传感器位姿信息输入
        odom传感器数据格式: odom( x, y, z, roll,pitch,yaw): 其中,x,y作为智能车在平面地图的x和y坐标;z坐标忽略恒等于0; 
		roll(车身纵向翻滚角)和pitch(车身俯仰角)也不做考虑恒等于0. yaw(偏航角)作为智能车在平面地图上左右转向方向角. 
		即: odom( x, y, 0, 0,0,yaw),共6个测量值数据.   
		此外odom传感器还提供自身的噪音协方差矩阵,为一个 6x6矩阵.

        imu惯性传感器数据格式: imu(roll,pitch,yaw): 
		其中roll(车身纵向翻滚角)和pitch(车身俯仰角)也不做考虑恒等于0. yaw(偏航角)作为智能车在平面地图上左右转向方向角. 
		即: imu( 0,0,yaw),共3个测量值数据.   
		此外imu传感器还提供自身的噪音协方差矩阵,为一个 3x3矩阵.

		关于利用噪音协方差矩阵进行位姿的矫正，或许可以从“卡尔曼滤波EKF”入手进行研究
		这里不考虑该因素，aloam 的 /slam_odom 中也只有 pose 和 orientation, 因此可以忽略其存在。
	*/
	//至于把点反投影到当前坐标系中，或许不需要雷达的旋转位姿，毕竟是360度全角度扫描，可以直接在世界坐标系的平移系下作投影。


	//save record time
	oOdomPoint.oTimeStamp = oTrajectory.header.stamp;

	//add to trajectory array
	m_vOdomHistory.push(oOdomPoint);

	m_iTrajCount++;
}

/*************************************************
Function: InterpolateTraj
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
void FrameRecon::InterpolateTraj(const RosTimePoint & oCurrent, const RosTimePoint & oPast, const float& fRatio,
	pcl::PointXYZI & oInter){


	//The ratio is from the interpolated value to oCurrent value 
	//Complementary ratio
	float fCompRatio = 1 - fRatio;
	//p+(c-p)(1-r)
	oInter.x = oCurrent.oLocation.x * fCompRatio + oPast.oLocation.x * fRatio;
	oInter.y = oCurrent.oLocation.y * fCompRatio + oPast.oLocation.y * fRatio;
	oInter.z = oCurrent.oLocation.z * fCompRatio + oPast.oLocation.z * fRatio;

}

/*************************************************
Function: ComputeQueryTraj
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
pcl::PointXYZI FrameRecon::ComputeQueryTraj(const ros::Time & oQueryTime){

	pcl::PointXYZI oResTraj;
	//clear the output
	oResTraj.x = 0.0;
	oResTraj.y = 0.0;
	oResTraj.z = 0.0;
	//index
	int iTrajIdx = 0;
	//time different
	double timeDiff = (oQueryTime - m_vOdomHistory[iTrajIdx].oTimeStamp).toSec();
	//search the most recent time
	while (iTrajIdx < m_vOdomHistory.size() - 1 && timeDiff > 0) {
		//increase index
		iTrajIdx++;
		//time different
		timeDiff = (oQueryTime - m_vOdomHistory[iTrajIdx].oTimeStamp).toSec();
	}

	//if the querytime is out of the stored time section 
	if (iTrajIdx == 0 || timeDiff > 0) {
		//turn back zero
		oResTraj.x = m_vOdomHistory[iTrajIdx].oLocation.x;
		oResTraj.y = m_vOdomHistory[iTrajIdx].oLocation.y;
		oResTraj.z = m_vOdomHistory[iTrajIdx].oLocation.z;

	}else {//if it is between two stored times
		//get the ratio
		//ROS_INFO("Trajtime between: %f and %f", m_vOdomHistory[iTrajIdx].oTimeStamp.toSec(), m_vOdomHistory[iTrajIdx - 1].oTimeStamp.toSec());

		float ratio = -timeDiff / (m_vOdomHistory[iTrajIdx].oTimeStamp - m_vOdomHistory[iTrajIdx - 1].oTimeStamp).toSec();
		//interpolate an accuracy value
		InterpolateTraj(m_vOdomHistory[iTrajIdx], m_vOdomHistory[iTrajIdx - 1], ratio, oResTraj);
	}

	return oResTraj;

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
void FrameRecon::SamplePoints(const pcl::PointCloud<pcl::PointXYZI> & vCloud, pcl::PointCloud<pcl::PointXYZI> & vNewCloud, int iSampleNum, bool bIntervalSamp){

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
void FrameRecon::OutputPCFile(const pcl::PointCloud<pcl::PointXYZI> & vCloud, bool bAllRecord){
  
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
void FrameRecon::OutputPCFile(const pcl::PointCloud<pcl::PointXYZI> & vCloud, const std::vector<float> & vFeatures, bool bAllRecord){
  
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




