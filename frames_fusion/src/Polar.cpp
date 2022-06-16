#include "Polar.h"

//https://blog.csdn.net/xuanzhea/article/details/115762999

//
Polar CoordinateTrans::EuclidToPolar(const float & x, const float & y, const float & z){

	Polar oPoint;
	oPoint.r = sqrt(x * x + y * y + z*z);
	oPoint.theta = acos(z / oPoint.r);
	oPoint.phi = atan(y / x); 

	return oPoint;

}



pcl::PointXYZ CoordinateTrans::PolarToEuclid(const Polar & oPolarValue){

	pcl::PointXYZ oPoint;
	oPoint.x = oPolarValue.r * sin(oPolarValue.theta) * cos(oPolarValue.phi);
	oPoint.y = oPolarValue.r * sin(oPolarValue.theta) * sin(oPolarValue.phi);;
	oPoint.z = oPolarValue.r * cos(oPolarValue.theta);

	return oPoint;

}



void CoordinateTrans::EuclidToPolarCloud(const pcl::PointCloud<pcl::PointXYZ> & vCloud, std::vector<Polar> & vPolarCloud){

	vPolarCloud.clear();

	for (int i = 0; i != vCloud.size(); ++i){

		Polar oPoint = EuclidToPolar(vCloud.points[i].x, vCloud.points[i].y, vCloud.points[i].z);
		vPolarCloud.push_back(oPoint);

	}


}


void CoordinateTrans::EuclidToPolarCloud(const pcl::PointCloud<pcl::PointXYZ> & vCloud, const pcl::PointXYZ & oViewPoint,
	                                     pcl::PointCloud<pcl::PointXYZ> & vLocalCloud, std::vector<Polar> & vPolarCloud,
	                                     float & fMinPhi, float & fMaxPhi, float & fMinTheta, float & fMaxTheta){

	vPolarCloud.clear();
	vLocalCloud.clear();

	fMinPhi = FLT_MAX;
	fMaxPhi = -FLT_MAX;
	fMinTheta = FLT_MAX;
	fMaxTheta = -FLT_MAX;

	//transform the coordinate value for E to P
	for (int i = 0; i != vCloud.size(); ++i){

		//tranform the point
		pcl::PointXYZ oLocalPoint;
		oLocalPoint.x = vCloud.points[i].x - oViewPoint.x;
		oLocalPoint.y = vCloud.points[i].y - oViewPoint.y;
		oLocalPoint.z = vCloud.points[i].z - oViewPoint.z;
		Polar oPoint = EuclidToPolar(oLocalPoint.x, oLocalPoint.y, oLocalPoint.z);
		//point in local coordinate system
		vLocalCloud.push_back(oLocalPoint);
		//point in polar coordinate system
		vPolarCloud.push_back(oPoint);

		//get the angle range
		if (fMinPhi > oPoint.phi)
			fMinPhi = oPoint.phi;
		if (fMaxPhi < oPoint.phi)
			fMaxPhi = oPoint.phi;
		if (fMinTheta > oPoint.theta)
			fMinTheta = oPoint.theta;
		if (fMaxTheta < oPoint.theta)
			fMaxTheta = oPoint.theta;

	}

}




void CoordinateTrans::PolarToEuclidCloud(const std::vector<Polar> & oPolarCloud , pcl::PointCloud<pcl::PointXYZ> & vCloud){


	vCloud.clear();

	for (int i = 0; i != oPolarCloud.size(); ++i){

		pcl::PointXYZ oPoint = PolarToEuclid(oPolarCloud[i]);
		vCloud.points.push_back(oPoint);
	
	}
	


}








Sector::Sector(float fDefaultDis) : m_fMinPhi(FLT_MAX), m_fMaxPhi(-FLT_MAX),
                   m_fMinTheta(FLT_MAX), m_fMaxTheta(-FLT_MAX),
				   m_fPhiRes(PI / 180.0f), m_fThetaRes(PI / 180.0f){

	//set the defaults of each signed distance
	m_fDefaultDis = fDefaultDis;


}



Sector::~Sector(){



}

	//set phi range
void Sector::SetPhiRange(float fMinPhi, float fMaxPhi){

	m_fMinPhi = fMinPhi;
	m_fMaxPhi = fMaxPhi;

}
	//set theta range
void Sector::SetThetaRange(float fMinTheta, float fMaxTheta){

	m_fMinTheta = fMinTheta;
	m_fMaxTheta = fMaxTheta;

}
	//set the resolution
void Sector::SetResolution(float fPhiRes, float fThetaRes){

	m_fPhiRes = fPhiRes;
	m_fThetaRes = fThetaRes;

}
	//set the viewpoint position (location)
void Sector::SetViewPoint(const pcl::PointXYZ & oViewPoint){

	m_oViewPoint.x = oViewPoint.x;
	m_oViewPoint.y = oViewPoint.y;
	m_oViewPoint.z = oViewPoint.z;

}

//
int Sector::PointToGrid(const Polar & oPolarPoint){

	//get the angle indexes
	int iPhiIdx = ceil((oPolarPoint.phi - m_fMinPhi) / m_fPhiRes);
	int iThetaIdx = ceil((oPolarPoint.theta - m_fMinTheta) / m_fThetaRes);
	//put the point into grid
	//label which point is in this grid
	return iPhiIdx * int(m_fThetaGridNum) + iThetaIdx;

}

//
int Sector::PointToGrid(const Polar & oPolarPoint, int & iPhiIdx, int & iThetaIdx){

	//get the angle indexes
	iPhiIdx = ceil((oPolarPoint.phi - m_fMinPhi) / m_fPhiRes);
	iThetaIdx = ceil((oPolarPoint.theta - m_fMinTheta) / m_fThetaRes);
	//put the point into grid
	//label which point is in this grid
	return iPhiIdx * int(m_fThetaGridNum) + iThetaIdx;

}

void Sector::PointToGrid(const Polar & oPolarPoint, int & iGirdIdx){

	//get the angle indexes
	int iPhiIdx = ceil((oPolarPoint.phi - m_fMinPhi) / m_fPhiRes);
	int iThetaIdx = ceil((oPolarPoint.theta - m_fMinTheta) / m_fThetaRes);
	
	//put the point into grid
	//label which point is in this grid
	//if the gird is within the input point clouds
	iGirdIdx = -1;
	//if the gird is out of the input point clouds
	if (iPhiIdx < m_fPhiGridNum && iPhiIdx>=0){
		if (iThetaIdx < m_fThetaGridNum && iThetaIdx>=0){
			iGirdIdx = iPhiIdx * int(m_fThetaGridNum) + iThetaIdx;
		}
	}

}



void Sector::GridToViewPDistance(const pcl::PointCloud<pcl::PointXYZ> & vCloud, int iType){


	m_vGridToViewDis.clear();

	for (int i = 0; i != m_vGirdCloudIndex.size(); ++i){

		float fGridToVDis = m_fDefaultDis;

		if (m_vGirdCloudIndex[i].size()){
			
			pcl::PointXYZ oPoint;
			oPoint.x = 0.0;
			oPoint.y = 0.0;
			oPoint.z = 0.0;

			for (int j = 0; j != m_vGirdCloudIndex[i].size(); ++j){
			
				int iPidx = m_vGirdCloudIndex[i][j];
				oPoint.x = oPoint.x + vCloud.points[iPidx].x;
				oPoint.y = oPoint.y + vCloud.points[iPidx].y;
				oPoint.z = oPoint.z + vCloud.points[iPidx].z;
			
			}//end j

			oPoint.x = oPoint.x / float(m_vGirdCloudIndex[i].size());
			oPoint.y = oPoint.y / float(m_vGirdCloudIndex[i].size());
			oPoint.z = oPoint.z / float(m_vGirdCloudIndex[i].size());
			//distance from surface to viewpoint
			fGridToVDis = ComputeDistance(oPoint);
		
		}//end if non-empty

		m_vGridToViewDis.push_back(fGridToVDis);
		
	}//end i

}

float Sector::ComputeDistance(const pcl::PointXYZ & oPA, const pcl::PointXYZ & oPB){

	//distance d = sqrt(x^2+y^2+z^2)
	float fDis = (oPA.x - oPB.x)*(oPA.x - oPB.x) + (oPA.y - oPB.y)*(oPA.y - oPB.y)
		+ (oPA.z - oPB.z)*(oPA.z - oPB.z);

	return sqrt(fDis);

}


float Sector::ComputeDistance(const pcl::PointXYZ & oPA){

	//distance d = sqrt(x^2+y^2+z^2)
	float fDis = oPA.x*oPA.x + oPA.y*oPA.y+ oPA.z*oPA.z ;

	return sqrt(fDis);

}

//assign points to sector
void Sector::AssignPoints(const pcl::PointCloud<pcl::PointXYZ> & vCloud){

	std::vector<Polar> vPolarCloud;
	pcl::PointCloud<pcl::PointXYZ> vLocalCloud;
	vPolarCloud.reserve(vCloud.points.size());
	m_vCloudGridIdx.resize(vCloud.points.size(),0);

	CoordinateTrans Coorder;
	//get the points in polar coordinate
	Coorder.EuclidToPolarCloud(vCloud, m_oViewPoint,vLocalCloud, vPolarCloud, m_fMinPhi, m_fMaxPhi, m_fMinTheta, m_fMaxTheta);
	//Coorder.EuclidToPolarCloud(vCloud, vPolarCloud);

	//compute the number of grid on phi and theta, respectively
	//2 is for precision beyond the bounds
	int iPhiGridNum = ceil((m_fMaxPhi - m_fMinPhi) / m_fPhiRes) + 2;
	int iThetaGridNum = ceil((m_fMaxTheta - m_fMinTheta) / m_fThetaRes) + 2;
	m_fPhiGridNum = float(iPhiGridNum);
	m_fThetaGridNum = float(iThetaGridNum);

	//construct the angle grids
	int iGridsNum = iPhiGridNum * iThetaGridNum;
	std::vector<int> vEmptyVec;
	for (int i = 0; i != iGridsNum; ++i)
		m_vGirdCloudIndex.push_back(vEmptyVec);

	//assign points to angle grids
	for (int i = 0; i != vPolarCloud.size(); ++i){
		
		//get the angle indexes
		int iGrididx = PointToGrid(vPolarCloud[i]);
		//put the point into grid
		//label which point is in this grid
		m_vGirdCloudIndex[iGrididx].push_back(i);
		m_vCloudGridIdx[i] = iGrididx;
		
	}	

	//compute the distance from each grid to viewpoint based on m_vGirdCloudIndex
	GridToViewPDistance(vLocalCloud);

}

	//compute a simple voxel based ray cast
std::vector<float> Sector::Raycast(const pcl::PointCloud<pcl::PointXYZ> & vCloud, const pcl::PointCloud<pcl::PointXYZ> & vNodes){

	//define output
	std::vector<float> vSignedDis;

	CoordinateTrans oCoorder;

	for (int i = 0; i != vNodes.size(); ++i){
		
		//get the grid idx correspoding to query corner
		//get the local node location based on the viewpoint
		pcl::PointXYZ oOneLocalNode;
		oOneLocalNode.x = vNodes.points[i].x - m_oViewPoint.x;
		oOneLocalNode.y = vNodes.points[i].y - m_oViewPoint.y;
		oOneLocalNode.z = vNodes.points[i].z - m_oViewPoint.z;
		//get point in polar coordinate system
		Polar oOneNodePolar = oCoorder.EuclidToPolar(oOneLocalNode.x, oOneLocalNode.y, oOneLocalNode.z);
		//get the node corrsesponding to gird
		int iGrididx;
		PointToGrid(oOneNodePolar, iGrididx);
		m_vNodeGridIdx.push_back(iGrididx);
		
		//compute the distance from viewpoint to node
		float fSignedDis;
		if (iGrididx >= 0){

			float vNodeToViewDis;
			vNodeToViewDis = ComputeDistance(vNodes.points[i]);
			//compute the signed distance based on the distances from surface and node to viewpoint
			fSignedDis = m_vGridToViewDis[iGrididx] - vNodeToViewDis;
		}else
			fSignedDis = m_fDefaultDis;

		vSignedDis.push_back(fSignedDis);
	
	}

	return vSignedDis;

}



//get the point clouds with grid id
void Sector::CloudGridIdx(std::vector<int> & vCloudGridLabels){

	//clear raw data
	vCloudGridLabels.clear();
	vCloudGridLabels.resize(m_vCloudGridIdx.size(),0);

	//assigment
	for (int i = 0; i != m_vCloudGridIdx.size(); ++i){
	
		vCloudGridLabels[i] = m_vCloudGridIdx[i];
	
	}

}

//get the point clouds with distance from its grid to viewpoint
//note that m_vGridToViewDis presents grid not point
void Sector::CloudGridDis(std::vector<float> & vCloudGridDis){

	//clear raw data
	vCloudGridDis.clear();
	vCloudGridDis.resize(m_vCloudGridIdx.size(), 0.0);

	//assigment
	for (int i = 0; i != m_vCloudGridIdx.size(); ++i){

		vCloudGridDis[i] = m_vGridToViewDis[m_vCloudGridIdx[i]];

	}

}


//grid point clouds
//void Sector::Get2DGridCloud(std::vector<int> & vLabels, const int & iPhiIdx, const int & iThetaIdx, const int & iSize){
//
//	vLabels.clear();
//	vLabels.resize(iSize, 0);
//	if (m_vGirdCloudIndex2D[iPhiIdx][iThetaIdx].size()){
//		//label the query gird
//		for (int i = 0; i != m_vGirdCloudIndex2D[iPhiIdx][iThetaIdx].size(); ++i){
//			int iPointId = m_vGirdCloudIndex2D[iPhiIdx][iThetaIdx][i];
//			vLabels[iPointId] = 1;
//		}//end for
//	//else if the grid is empty, then label the grid with the maiximum inner point
//	}else{
//	
//		//label the query gird
//		int iMaxI;
//		int iMaxJ;
//		int iMaxNum = 0;
//		for (int i = 0; i != m_vGirdCloudIndex2D[i].size(); ++i){
//			for (int j = 0; j != m_vGirdCloudIndex2D[i][j].size(); ++j){
//				if (m_vGirdCloudIndex2D[i][j].size() > iMaxNum){
//					iMaxNum = m_vGirdCloudIndex2D[i][j].size();
//					iMaxI = i;
//					iMaxJ = j;
//				}//end if
//			}//end for j
//		}//end for i
//	
//		for (int i = 0; i != m_vGirdCloudIndex2D[iMaxI][iMaxJ].size(); ++i){
//			int iPointId = m_vGirdCloudIndex2D[iMaxI][iMaxJ][i];
//			vLabels[iPointId] = 1;
//		}//end for
//
//	}//end else
//
//}