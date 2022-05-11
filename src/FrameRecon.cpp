#include "FrameRecon.h"


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
                       m_iTrajFrameNum(0){

	//read parameters
	ReadLaunchParams(nodeHandle);

	//***subscriber related*** 
	//subscribe (hear) the odometry information (trajectory)
	m_oOdomSuber = nodeHandle.subscribe(m_sInOdomTopic, 1, &FrameRecon::HandleTrajectory, this);

	//subscribe (hear) the point cloud topic 
	m_oCloudSuber = nodeHandle.subscribe(m_sInCloudTopic, 1, &FrameRecon::HandlePointClouds, this);

	//***publisher related*** 
	//publish point cloud after processing
	m_oCloudPublisher = nodeHandle.advertise<sensor_msgs::PointCloud2>(m_sOutCloudTopic, 1, true);

  	//publish polygon constructed from one frame point cloud
	m_oMeshPublisher = nodeHandle.advertise<visualization_msgs::Marker>(m_sOutMeshTopic, 1);


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

	//define ouput ply file name
	m_sOutPCNormalFileName << m_sFileHead << "Map_PCNormal.ply"; 

	//output point clouds with computed normals to the files when the node logs out
	pcl::io::savePLYFileASCII(m_sOutPCNormalFileName.str(), m_vMapPCN);

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
  nodeHandle.param("file_outputpath", m_sFileHead, std::string("./"));

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

  //height of viewpoint
  double dViewZOffset;
  nodeHandle.param("viewp_zoffset", dViewZOffset, 0.0);
  m_fViewZOffset = float(dViewZOffset);
  
  //explicit reconstruction related
  //number of sectors
  nodeHandle.param("sector_num", m_iSectorNum, 1);
  m_oExplicitBuilder.HorizontalSectorSize(m_iSectorNum);

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
void FrameRecon::PublishPointCloud(const pcl::PointCloud<pcl::PointXYZ> & vCloud){
  
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
Input: vCloud - a point clouds to be published
Output: none
Return: none
Others: none
*************************************************/
void FrameRecon::PublishPointCloud(const pcl::PointCloud<pcl::PointXYZ> & vCloud, const std::vector<float> & vFeatures){
  
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
	color.a = 1;
	color.r = 255;
	color.g = 255;
	color.b = 255;
	
	//repeatable vertices
	pcl::PointCloud<pcl::PointXYZ> vMeshVertices;

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

	if (!(m_iPCFrameCount%m_iFrameSmpNum)){

		////a point clouds in PCL type
		pcl::PointCloud<pcl::PointXYZ>::Ptr pSLAMCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr pRawCloud(new pcl::PointCloud<pcl::PointXYZ>);
		////message from ROS type to PCL type
		pcl::fromROSMsg(vLaserData, *pSLAMCloud);
		for(int i=0;i!=pSLAMCloud->points.size();++i){

			pcl::PointXYZ oPoint;
			oPoint.x = pSLAMCloud->points[i].z;
			oPoint.y = pSLAMCloud->points[i].x;
			oPoint.z = pSLAMCloud->points[i].y;
			pRawCloud->points.push_back(oPoint);

		}

		//if have corresponding trajectory point (viewpoint)
		pcl::PointXYZ oCurrentViewP;

		if (m_vOdomHistory.size()){
			//
			oCurrentViewP = ComputeQueryTraj(vLaserData.header.stamp);
		
		//else waiting for sync
		}else{

			return;
		}
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr pSceneCloud(new pcl::PointCloud<pcl::PointXYZ>);
		SamplePoints(*pRawCloud, *pSceneCloud, m_iSampleInPNum);
		
		pcl::PointCloud<pcl::PointNormal>::Ptr pFramePNormal(new pcl::PointCloud<pcl::PointNormal>);

		m_oExplicitBuilder.SetViewPoint(oCurrentViewP);
		m_oExplicitBuilder.FrameReconstruction(*pSceneCloud, *pFramePNormal);

		//************output value******************
		for(int i=0;i!=pFramePNormal->points.size();++i)
			m_vMapPCN.points.push_back(pFramePNormal->points[i]);
				

		PublishMeshs();

		//clear this frame result
		m_oExplicitBuilder.ClearData();


		//count
		m_iPCFrameCount++;

	}

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

	//count input frames
	//m_iTrajPointNum++;

	//save the into the memory
	//save the position of trajectory
	RosTimePoint oOdomPoint;
	oOdomPoint.oLocation.x = oTrajectory.pose.pose.position.z;
	oOdomPoint.oLocation.y = oTrajectory.pose.pose.position.x;
	oOdomPoint.oLocation.z = oTrajectory.pose.pose.position.y;

	//save record time
	oOdomPoint.oTimeStamp = oTrajectory.header.stamp;

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
	pcl::PointXYZ & oInter){


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
pcl::PointXYZ FrameRecon::ComputeQueryTraj(const ros::Time & oQueryTime){

	pcl::PointXYZ oResTraj;
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
void FrameRecon::SamplePoints(const pcl::PointCloud<pcl::PointXYZ> & vCloud, pcl::PointCloud<pcl::PointXYZ> & vNewCloud, int iSampleNum, bool bIntervalSamp){

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
void FrameRecon::OutputPCFile(const pcl::PointCloud<pcl::PointXYZ> & vCloud, bool bAllRecord){
  
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
void FrameRecon::OutputPCFile(const pcl::PointCloud<pcl::PointXYZ> & vCloud, const std::vector<float> & vFeatures, bool bAllRecord){
  
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




