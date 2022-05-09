#include "FrameRecon.h"

//*********************************Initialization function*********************************

/*************************************************
Function: FrameRecon
Description: constrcution function for FrameRecon class
Calls: all member functions
Called By: main function of project
Table Accessed: none
Table Updated: none
Input: node - a ros node class
     nodeHandle - a private ros node class
Return: Parameter initialization
Others: m_oCnfdnSolver - (f_fSigma - the computed radius of robot
                        f_fGHPRParam - the parameter of GHPR algorithm
                        f_fVisTermThr - the threshold of visbility term - useless
                        f_fMinNodeThr - minimum value to generate node
                          m_fTraversWeight - traverel weight
                          m_fExploreWeight - visibility weight
                          m_fDisWeight - distance term weight
                          m_fBoundWeight - bound term weight)
      m_pBoundCloud - a point clouds storing the received boundary points
      m_pObstacleCloud - a point clouds storing the received obstacle points
      m_iTrajFrameNum - record received odometry frame
      m_iGroundFrames - record received ground point cloud frame
      m_iBoundFrames - record received boundary point cloud frame
      m_iObstacleFrames - record received obstacle point cloud frame 
      m_iComputedFrame - record computed times of processing point cloud frame
      m_iNodeTimes - count node generation times
      m_iAncherCount - count visited anchor in a trip 
      m_iOdomSampingNum - smapling number of odometry points
      m_bGridMapReadyFlag - a flag indicating the grid map has been initialized (true) or not (false)
      m_bCoverFileFlag - a flag indicating whether an coverage file is generated (true) or not (false)
      m_bOutTrajFileFlag - a flag indicating whether an out trajectroy file is generated
      m_bAnchorGoalFlag - a flag indicating the robot is moving Moving on a local optimization path
*************************************************/
FrameRecon::FrameRecon(ros::NodeHandle & node,
                       ros::NodeHandle & nodeHandle):
                       m_iTrajFrameNum(0){

  //read parameters
  ReadLaunchParams(nodeHandle);

  //subscribe (hear) the odometry information (trajectory)
  m_oOdomSuber = nodeHandle.subscribe(m_sOdomTopic, 1, &FrameRecon::HandleTrajectory, this);

  //subscribe (hear) the point cloud topic 
  m_oCloudSuber = nodeHandle.subscribe(m_sCloudTopic, 1, &FrameRecon::HandlePointClouds, this);

  //publish topic
  m_oCloudPublisher = nodeHandle.advertise<sensor_msgs::PointCloud2>("test_clouds", 1, true);

}


/*************************************************
Function: ~FrameRecon
Description: deconstrcution function for FrameRecon class
Calls: all member functions
Called By: main function of project
Table Accessed: none
Table Updated: none
Input: none
Output: none
Return: none
Others: none
*************************************************/

FrameRecon::~FrameRecon() {

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
  //nodeHandle.param("file_outputpath", m_sFileHead, std::string("./"));

  //input odom topic
  nodeHandle.param("odom_in_topic", m_sOdomTopic, std::string("/odometry/filtered"));

  //input point cloud topic
  nodeHandle.param("cloud_topic", m_sCloudTopic, std::string("/cloud_points"));

  //point cloud sampling number
  nodeHandle.param("sample_pcframe_num", m_iFrameSmpNum, 1);

  //point cloud sampling number
  nodeHandle.param("sample_inputpoints_num", m_iSampleInPNum, 1);

  //height of viewpoint
  double dViewZOffset;
  nodeHandle.param("pastview_zoffset", dViewZOffset, 0.0);
  m_fViewZOffset = float(dViewZOffset);
  
  //explicit reconstruction related
  //number of sectors
  nodeHandle.param("sector_num", m_iSectorNum, 1);
  m_oExplicitBuilder.HorizontalSectorSize(m_iSectorNum);

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
/*
void FrameRecon::PublishPointCloud(pcl::PointCloud<pcl::PointXYZ> & vCloud){
  
	//publish obstacle points
  sensor_msgs::PointCloud2 vCloudData;

  pcl::toROSMsg(vCloud, vCloudData);

  vCloudData.header.frame_id = m_oGMer.m_oFeatureMap.getFrameId();

  vCloudData.header.stamp = ros::Time::now();

  m_oCloudPublisher.publish(vCloudData);

}
*/

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

	//count input frames
	m_iPCFrameCount++;
	//ROS_INFO("frame number: %d", m_iFrames);

	if (!(m_iPCFrameCount%m_iFrameSmpNum)){

		////a point clouds in PCL type
		pcl::PointCloud<pcl::PointXYZ> vInputCloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr pRawCloud(new pcl::PointCloud<pcl::PointXYZ>);
		////message from ROS type to PCL type
		pcl::fromROSMsg(vLaserData, vInputCloud);

		//get right point clouds from LOAM output
		for (int i = 0; i != vInputCloud.size(); ++i){

			pcl::PointXYZ oArgPoint;
			oArgPoint.x = vInputCloud.points[i].z;
			oArgPoint.y = vInputCloud.points[i].x;
			oArgPoint.z = vInputCloud.points[i].y;
			pRawCloud->points.push_back(oArgPoint);

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
		pcl::PointCloud<pcl::PointXYZ> vGroundCloud;
		sensor_msgs::PointCloud2 vGroundPub;


		//publish ground points
		pcl::toROSMsg(vGroundCloud, vGroundPub);
		vGroundPub.header.frame_id = "camera_init";
		vGroundPub.header.stamp = vLaserData.header.stamp;
		//m_oGroundPub.publish(vGroundPub);
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
	oOdomPoint.oLocation.x = oTrajectory.pose.pose.position.z;//z in loam is x
	oOdomPoint.oLocation.y = oTrajectory.pose.pose.position.x;//x in loam is y
	oOdomPoint.oLocation.z = oTrajectory.pose.pose.position.y;//y in loam is z
	//save record time
	oOdomPoint.oTimeStamp = oTrajectory.header.stamp;

	m_vOdomHistory.push(oOdomPoint);

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









