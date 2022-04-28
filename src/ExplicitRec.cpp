#include "ExplicitRec.h"


ExplicitRec::ExplicitRec() :m_bElevationFlag(false),m_pCenterNormal(new pcl::PointCloud<pcl::PointNormal>){



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
SetViewPoint
Input: oViewPoint - a given view point
fViewElevation -  a given height value in global system for view point
Output: m_oViewPoint - view point
Function: Set the viewpoint
========================================*/
void ExplicitRec::SetViewPoint(const pcl::PointXYZ & oViewPoint, float fViewElevation){

	m_oViewPoint.x = oViewPoint.x;
	m_oViewPoint.y = oViewPoint.y;
	m_oViewPoint.z = oViewPoint.z;

	//the elevation value of the viewpoint has been obtained
	m_bElevationFlag = true;

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
RemovePseudoFaces
Input:vCenterPoints - centerpoints of faces
             vFaces - faces (vertex relationships)
         oMatNormal - face normals
Output: vTrueFaceStatus - face status indicating which face is a pseudo face
            vFaceWeight - face weight indicating How much each face resembles a pseudoface
Function: remove pseudo faces
========================================*/
void ExplicitRec::RemovePseudoFaces(const pcl::PointCloud<pcl::PointXYZ> & vCenterPoints, const std::vector<pcl::Vertices> & vFaces, const Eigen::MatrixXf & oMatNormal,
	std::vector<bool> & vTrueFaceStatus, std::vector<float> & vFaceWeight){

	//
	//Eigen::Vector3f oHorPlanN(0.0f, 0.0f, 1.0f);
	//float fVerticalThr = 0.17365;

	//for each face
	for (int i = 0; i != vFaces.size(); ++i){

		//vector from viewpoint to centerpoint of face
		Eigen::Vector3f oToCenterVec(vCenterPoints.points[i].x - m_oViewPoint.x, vCenterPoints.points[i].y - m_oViewPoint.y, vCenterPoints.points[i].z - m_oViewPoint.z);

		//calculate normal weights, e.g., the angle cospin value between normal and ray, where ranges are from - 1 to 1
		//its essence is the degree of orthogonality
		//dot product
		vFaceWeight[i] = oToCenterVec.dot(oMatNormal.row(i));
		//cospin value
		vFaceWeight[i] = vFaceWeight[i] / oToCenterVec.norm();

		//if it is close to be vertical
		if (fabs(vFaceWeight[i]) < m_fPseudoFaceThr)
			vTrueFaceStatus[i] = false;

		//	float  fVert = oHorPlanN.dot(oMatNormal.row(i));
		//	if (fVert < (-1.0f + fVerticalThr) && pCenterPoints->points[i].z < m_oViewPoint.z)
		//		vTrueFaceStatus[i] = false;
		//	if (fVert < fVerticalThr && pCenterPoints->points[i].z >= m_oViewPoint.z)
		//		vTrueFaceStatus[i] = false;


	}//end for i


}


/*=======================================
FrameReconstruction(const pcl::PointCloud<pcl::PointXYZ> & vSceneCloud)
Input: vSceneCloud - one frame point clouds
Outout: vScenePNormal - input point with mesh normal
Output: none
Function: clear some data
========================================*/
void ExplicitRec::FrameReconstruction(const pcl::PointCloud<pcl::PointXYZ> & vSceneCloud, pcl::PointCloud<pcl::PointNormal> & vScenePNormal){

	vScenePNormal.clear();
	vScenePNormal.reserve(vSceneCloud.points.size());

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

		//if (m_bElevationFlag){
		//	pcl::PointXYZ oViewGroundP;
		//	oViewGroundP.x = m_oViewPoint.x;
		//	oViewGroundP.y = m_oViewPoint.y;
		//	oViewGroundP.z = m_oViewPoint.z - m_fViewElevation;
		//	pSectorCloud->points.push_back(oViewGroundP);
		//}

	//******Mesh building******
		//***GHPR mesh building***
		GHPR hpdhpr(m_oViewPoint, m_GHPRParam);

		//perform reconstruction
		hpdhpr.Compute(pSectorCloud);
		
		//get the surfaces that are not connected to the viewpoint
		std::vector<pcl::Vertices> vOneFaces;
		vOneFaces = hpdhpr.ConstructSurfaceIdx();

		//***start the mesh related operation***
		//center point of each faces
		pcl::PointCloud<pcl::PointXYZ>::Ptr pCenterPoints(new pcl::PointCloud<pcl::PointXYZ>);

		//face normals
		Eigen::MatrixXf oMatNormal;

		//face parameters d
		Eigen::VectorXf vfDParam;

		//face status - whether the face is wrong
		std::vector<bool> vTrueFaceStatus(vOneFaces.size(),true);

		//face weight, e.g., confidence for each face
		//the confidence is higher, if the laser beam is orthophoto the surface 
		std::vector<float> vFaceWeight(vOneFaces.size(), 0.0f);
		
		//new a mesh operation object
		MeshOperation oMeshOper;
		
		//compute the centerpoint of each faces
		//The centerpoint re-represents its face
		oMeshOper.ComputeCenterPoint(*pSectorCloud, vOneFaces,*pCenterPoints);

		//compute the normal vector of each face for its centerpoint
		//the normal vector will be facing away from the viewpoint
		oMeshOper.ComputeAllFaceParams(m_oViewPoint, *pSectorCloud, vOneFaces, oMatNormal, vfDParam);

		//Remove pseudo triangles according to scanning rules of LiDAR
		RemovePseudoFaces(*pCenterPoints, vOneFaces, oMatNormal, vTrueFaceStatus, vFaceWeight);

		//point adj
		pcl::PointCloud<pcl::PointNormal> vCombinedNormal;

		//propagate the normal vector to each vertex
		//linearly compute weighted neighboring normal vector
		oMeshOper.LocalFaceNormal(*pSectorCloud, vOneFaces, oMatNormal, m_oViewPoint, vCombinedNormal);

		//***record data***
		//output normal
		oMeshOper.NormalMatrixToPCL(*pCenterPoints, oMatNormal, *m_pCenterNormal, true);

		//Record the remaining triangles
		std::vector<pcl::Vertices> vOneNewFaces;
		for (int j = 0; j != vTrueFaceStatus.size(); ++j){
		
			if (vTrueFaceStatus[j]){
				pcl::Vertices oOneFace(vOneFaces[j]);
				vOneNewFaces.push_back(oOneFace);
			}//end if
		
		}//end for j != vTrueFaceStatus.size()

		//collect the vertices and faces in each sector
		m_vAllSectorClouds.push_back(pSectorCloud);
		m_vAllSectorFaces.push_back(vOneNewFaces);

		//for point normal output
		for (int j = 0; j != vCombinedNormal.size(); ++j){
			pcl::PointNormal oOnePN(vCombinedNormal.points[j]);
			oOnePN.normal_x *= -1.0f;
			oOnePN.normal_y *= -1.0f;
			oOnePN.normal_z *= -1.0f;
			vScenePNormal.points.push_back(oOnePN);
		}

	}//end  i != vPointSecIdxs.size()

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





