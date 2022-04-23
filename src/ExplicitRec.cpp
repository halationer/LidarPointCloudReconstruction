#include "ExplicitRec.h"


ExplicitRec::ExplicitRec(){



}


ExplicitRec::~ExplicitRec(){




}

/*=======================================
SetViewPoint
Input: oViewPoint - a given view point
Output: m_oViewPoint - view point
Function: Set the viewpoint
========================================*/
void ExplicitRec::SetViewPoint(const pcl::PointXYZ & oViewPoint){

	m_oViewPoint.x = oViewPoint.x;
	m_oViewPoint.y = oViewPoint.y;
	m_oViewPoint.z = oViewPoint.z;

}


/*=======================================
HorizontalSectorSize
Input: iSectNum
Output: m_iSectNum - number of sectors
       m_fHorizontalRes - the size of the included angle of the sector area
Function: Set sector area division parameters
========================================*/
//Set the horizontal angle interval size
void ExplicitRec::HorizontalSectorSize(int iSectNum){


	m_iSectNum = iSectNum;
	//at least one sector (2*PI) for a circle
	if (!m_iSectNum)
		m_iSectNum = 1;

	m_fHorizontalRes = 2.0f * PI / float(m_iSectNum);

}


/*=======================================
HorizontalSectorSize - reload
Input: iSectNum
Output: m_iSectNum - number of sectors
m_fHorizontalRes - the size of the included angle of the sector area
Function: Set sector area division parameters
note: 0.0 < fHorizontalRes <= 2*PI
========================================*/
void ExplicitRec::HorizontalSectorSize(float fHorizontalRes){

	//angle resolution
	m_fHorizontalRes = fHorizontalRes;
	//at least one sector (2*PI) for a circle
	if (!m_fHorizontalRes)
		m_fHorizontalRes = 2.0f * PI;

	//if <1.0 it is 1.0
	m_iSectNum = ceil(2.0f * PI/m_fHorizontalRes);

}


/*=======================================
FrameReconstruction(const pcl::PointCloud<pcl::PointXYZ> & vSceneCloud)
Input: vSceneCloud - one frame point clouds
Output: none
Function: clear some data
========================================*/
void ExplicitRec::FrameReconstruction(const pcl::PointCloud<pcl::PointXYZ> & vSceneCloud){

	//******Sector division******
	//point index for each sector
	std::vector<std::vector<int>> vPointSecIdxs;
	DivideSector oSectorDivider(m_iSectNum);
	//set the viewpoint as 2D coordinate origin
	oSectorDivider.SetOriginPoint(m_oViewPoint);
	//divide clouds into sector area
	oSectorDivider.ComputePointSectorIdxs(vSceneCloud, vPointSecIdxs);

	//point clouds in each sector
	m_vAllSectorClouds.clear();
	m_vAllSectorClouds.reserve(vPointSecIdxs.size());

	//triangular face in each sector
	m_vAllSectorFaces.clear();
	m_vAllSectorFaces.reserve(vPointSecIdxs.size());

	//triangular face in each sector
	for (int i = 0; i != vPointSecIdxs.size(); ++i) {

		//point clouds inside one sector
		pcl::PointCloud<pcl::PointXYZ>::Ptr pSectorCloud(new pcl::PointCloud<pcl::PointXYZ>);
		//get a point clouds in one section
		for (int j = 0; j != vPointSecIdxs[i].size(); ++j) {
			//get point index
			int iSecPointInAllIdx = vPointSecIdxs[i][j];
			//construct point clouds
			pSectorCloud->points.push_back(vSceneCloud.points[iSecPointInAllIdx]);
		}

	//******Mesh building******
		//***GHPR mesh building***
		GHPR hpdhpr(m_oViewPoint, m_GHPRParam);

		//perform reconstruction
		hpdhpr.Compute(pSectorCloud);
		
		//get the surfaces that are not connected to the viewpoint
		std::vector<pcl::Vertices> vOneFaces;
		vOneFaces = hpdhpr.ConstructSurfaceIdx();

		pcl::PointCloud<pcl::PointXYZ>::Ptr pCenterPoints(new pcl::PointCloud<pcl::PointXYZ>);
		MeshOperation oMeshOper;
		Eigen::MatrixXf oMatNormal;
		Eigen::VectorXf vfDParam;
		std::vector<bool> vTrueFaceStatus(vOneFaces.size(),true);
		float fOrthogThr = 0.05;
		oMeshOper.ComputeCenterPoint(*pSectorCloud, vOneFaces,*pCenterPoints);
		oMeshOper.ComputeAllFaceParams(*pSectorCloud, vOneFaces, oMatNormal,vfDParam);

		for (int i = 0; i != vOneFaces.size(); ++i){
			
			Eigen::Vector3f oToCenterVec(pCenterPoints->points[i].x - m_oViewPoint.x, pCenterPoints->points[i].y - m_oViewPoint.y, pCenterPoints->points[i].z - m_oViewPoint.z);
			//degree of orthogonality
			float fOrth = oToCenterVec.dot(oMatNormal.row(i));
			fOrth = fOrth / oToCenterVec.norm();
			if (fabs(fOrth) < fOrthogThr)
				vTrueFaceStatus[i] = false;
		
		}

		std::vector<pcl::Vertices> vOneNewFaces;
		for (int i = 0; i != vTrueFaceStatus.size(); ++i){
		
			if (vTrueFaceStatus[i]){
				pcl::Vertices oOneFace(vOneFaces[i]);
				vOneNewFaces.push_back(oOneFace);
			}
		
		}

		m_vAllSectorClouds.push_back(pSectorCloud);
		m_vAllSectorFaces.push_back(vOneNewFaces);
	}




}


/*=======================================
ClearData
Input: none
Output: none
Function: clear some data 
========================================*/
void ExplicitRec::ClearData(){

	//clear data
	m_vAllSectorClouds.clear();

	m_vAllSectorFaces.clear();


}

