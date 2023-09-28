#include "ExplicitRec.h"

#include <ros/ros.h>
#include <iostream>
#include <sstream>     
#include <pcl/io/ply_io.h>    
#include <thread>
#include <atomic>
#include <chrono>
#include <mutex>
#include <functional>

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
void ExplicitRec::SetViewPoint(const pcl::PointXYZI & oViewPoint){

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
void ExplicitRec::SetViewPoint(const pcl::PointXYZI & oViewPoint, float fViewElevation){

	m_oViewPoint.x = oViewPoint.x;
	m_oViewPoint.y = oViewPoint.y;
	m_oViewPoint.z = oViewPoint.z;

	//If the height of the laser scanner above the ground is known
	if(fViewElevation){

		m_fViewElevation = fViewElevation;
		//the elevation value of the viewpoint has been obtained
		m_bElevationFlag = true;

	}

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
void ExplicitRec::RemovePseudoFaces(const pcl::PointCloud<pcl::PointXYZI> & vCenterPoints, const std::vector<pcl::Vertices> & vFaces, const Eigen::MatrixXf & oMatNormal,
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
FrameReconstruction
Input: vSceneCloud - one frame point clouds
Outout: vScenePNormal - input point with mesh normal
Output: none
Function: clear some data
========================================*/
void ExplicitRec::FrameReconstruction(const pcl::PointCloud<pcl::PointXYZI> & vSceneCloud, pcl::PointCloud<pcl::PointNormal> & vScenePNormal, const int line_min, const int line_max){

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
	m_vAllSectorClouds.resize(vPointSecIdxs.size());

	//triangular face in each sector
	m_vAllSectorFaces.clear();
	m_vAllSectorFaces.resize(vPointSecIdxs.size());

	m_vMatNormal.clear();
	m_vMatNormal.resize(vPointSecIdxs.size());

	m_vFaceWeight.clear();
	m_vFaceWeight.resize(vPointSecIdxs.size());

	//多线程分离Normal
	std::vector<pcl::PointCloud<pcl::PointNormal>> vCombinedNormalList;
	vCombinedNormalList.resize(vPointSecIdxs.size());
	std::mutex convex_hull;
	std::vector<std::thread> thread_pool;
	std::vector<std::function<void(void)>> thread_func(vPointSecIdxs.size());
	const bool bMultiThread = m_bMultiThread;

	for(int i = 0; i != vPointSecIdxs.size(); ++i) {
		thread_func[i] = [&, i, bMultiThread]() { 
			//If the points in a sector is sufficient to calculate
			if (vPointSecIdxs[i].size() > m_iSectorMinPNum){

				//point clouds inside one sector
				pcl::PointCloud<pcl::PointXYZI>::Ptr pSectorCloud(new pcl::PointCloud<pcl::PointXYZI>);

				//get a point clouds in one section
				for (int j = 0; j != vPointSecIdxs[i].size(); ++j) {
					//get point index
					int iSecPointInAllIdx = vPointSecIdxs[i][j];
					//construct point clouds
					pSectorCloud->points.push_back(vSceneCloud.points[iSecPointInAllIdx]);
				}

				if (m_bElevationFlag){
					pcl::PointXYZI oViewGroundP;
					oViewGroundP.x = m_oViewPoint.x;
					oViewGroundP.y = m_oViewPoint.y;
					oViewGroundP.z = m_oViewPoint.z - m_fViewElevation;
					pSectorCloud->points.push_back(oViewGroundP);
				}

				//******Mesh building******
				//***GHPR mesh building***
				GHPR hpdhpr(m_oViewPoint, m_GHPRParam);

				//perform reconstruction
				if(bMultiThread)
					hpdhpr.ComputeMultiThread(pSectorCloud);
				else
					hpdhpr.Compute(pSectorCloud);

				//get the surfaces that are not connected to the viewpoint
				std::vector<pcl::Vertices> vOneFaces;
				vOneFaces = hpdhpr.ConstructSurfaceIdxFiltered(line_min, line_max);
				// vOneFaces = hpdhpr.ConstructSurfaceIdx();

				/*输出单份的重建网格结果
				{
					system("mkdir -p $PWD/../Dense_ROS/save");
					pcl::PolygonMesh OutMeshModel;
					pcl::toPCLPointCloud2(*pSectorCloud, OutMeshModel.cloud);
					OutMeshModel.polygons = vOneFaces;
					std::stringstream ss;
					ss << "../Dense_ROS/save/GhprMesh_" << m_working_frame_count << "_" << i << ".ply";
					pcl::io::savePLYFileBinary(ss.str(), OutMeshModel); 
				}
				//*/

				//***start the mesh related operation***
				//new a mesh operation object
				MeshOperation oMeshOper;

				//center point of each faces
				pcl::PointCloud<pcl::PointXYZI>::Ptr pCenterPoints(new pcl::PointCloud<pcl::PointXYZI>);
				//compute the centerpoint of each faces
				//The centerpoint re-represents its face
				oMeshOper.ComputeCenterPoint(*pSectorCloud, vOneFaces, *pCenterPoints);

				//face normals
				Eigen::MatrixXf& oMatNormal = m_vMatNormal[i];
				//face parameters d
				Eigen::VectorXf vfDParam;
				//compute the normal vector of each face for its centerpoint
				//the normal vector will be facing away from the viewpoint
				oMeshOper.ComputeAllFaceParams(m_oViewPoint, *pSectorCloud, vOneFaces, oMatNormal, vfDParam);

				/*此处是修正法向量之后的结果
				{
					system("mkdir -p $PWD/../Dense_ROS/save");
					pcl::PolygonMesh OutMeshModel;
					pcl::toPCLPointCloud2(*pSectorCloud, OutMeshModel.cloud);
					OutMeshModel.polygons = vOneFaces;
					std::stringstream ss;
					ss << "../Dense_ROS/save/GhprMesh_" << m_working_frame_count << "_" << i << ".ply";
					pcl::io::savePLYFileBinary(ss.str(), OutMeshModel); 
				}
				//*/

				//face status - whether the face is wrong
				std::vector<bool> vTrueFaceStatus(vOneFaces.size(), true);
				//face weight, e.g., confidence for each face
				//the confidence is higher, if the laser beam is orthophoto the surface 
				std::vector<float>& vFaceWeight = m_vFaceWeight[i];
				vFaceWeight.resize(vOneFaces.size(), 0.0f);
				//Remove pseudo triangles according to scanning rules of LiDAR
				RemovePseudoFaces(*pCenterPoints, vOneFaces, oMatNormal, vTrueFaceStatus, vFaceWeight);

				//propagate the normal vector to each vertex
				//linearly compute weighted neighboring normal vector
				oMeshOper.LocalFaceNormalAndDistanceConfidence(*pSectorCloud, vOneFaces, oMatNormal, m_oViewPoint, vTrueFaceStatus, vCombinedNormalList[i]);

				//***record data***
				//output normal
				// oMeshOper.NormalMatrixToPCL(*pCenterPoints, oMatNormal, *m_pCenterNormal, true);

				//Record the remaining triangles
				std::vector<pcl::Vertices> vOneNewFaces;
				for (int j = 0; j != vTrueFaceStatus.size(); ++j){

					if ( vTrueFaceStatus[j] || false ){			// TODO: 是否要剔除面片呢?
						pcl::Vertices oOneFace(vOneFaces[j]);
						vOneNewFaces.push_back(oOneFace);
					}//end if

				}//end for j != vTrueFaceStatus.size()

				//collect the vertices and faces in each sector
				m_vAllSectorClouds[i] = pSectorCloud;
				m_vAllSectorFaces[i] = vOneNewFaces;

				/*此处是剔除不正确face的结果
				{
					system("mkdir -p $PWD/../Dense_ROS/save");
					pcl::PolygonMesh OutMeshModel;
					pcl::toPCLPointCloud2(*pSectorCloud, OutMeshModel.cloud);
					OutMeshModel.polygons = vOneNewFaces;
					std::stringstream ss;
					ss << "../Dense_ROS/save/GhprMesh_" << m_working_frame_count << "_" << i << ".ply";
					pcl::io::savePLYFileBinary(ss.str(), OutMeshModel); 
				}
				//*/
				
			
			}else{
				
				pcl::PointCloud<pcl::PointXYZI>::Ptr pSectorCloud(new pcl::PointCloud<pcl::PointXYZI>);
				m_vAllSectorClouds[i] = pSectorCloud;

				std::vector<pcl::Vertices> vOneNewFaces;
				m_vAllSectorFaces[i] = vOneNewFaces;
			
			}//end if vPointSecIdxs[i].size() > m_iSectorMinPNum
		};
	}
	
	if(bMultiThread) {

		//triangular mesh in each sector
		for (int i = 0; i != vPointSecIdxs.size(); ++i)
			thread_pool.emplace_back(thread_func[i]);

		for(int i = 0; i != vPointSecIdxs.size(); ++i)
				thread_pool[i].join(); 
	}
	else {

		for(int i = 0; i != vPointSecIdxs.size(); ++i)
			thread_func[i]();
	}


	//output 1ms
	for (int i = 0; i != vPointSecIdxs.size(); ++i)
		for (int j = 0; j != vCombinedNormalList[i].size(); ++j)
			vScenePNormal.points.push_back(vCombinedNormalList[i].points[j]);
}

void ExplicitRec::OriginalReconstruction(const pcl::PointCloud<pcl::PointXYZI> & vSceneCloud) {

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
	m_vAllSectorClouds.resize(vPointSecIdxs.size());

	//triangular face in each sector
	m_vAllSectorFaces.clear();
	m_vAllSectorFaces.resize(vPointSecIdxs.size());

	//多线程分离Normal
	std::vector<pcl::PointCloud<pcl::PointNormal>> vCombinedNormalList;
	vCombinedNormalList.resize(vPointSecIdxs.size());
	std::mutex convex_hull;
	std::vector<std::thread> thread_pool;
	std::vector<std::function<void(void)>> thread_func(vPointSecIdxs.size());
	const bool bMultiThread = m_bMultiThread;

	for(int i = 0; i != vPointSecIdxs.size(); ++i) {
		thread_func[i] = [&, i, bMultiThread]() { 
			//If the points in a sector is sufficient to calculate
			if (vPointSecIdxs[i].size() > m_iSectorMinPNum){

				//point clouds inside one sector
				pcl::PointCloud<pcl::PointXYZI>::Ptr pSectorCloud(new pcl::PointCloud<pcl::PointXYZI>);

				//get a point clouds in one section
				for (int j = 0; j != vPointSecIdxs[i].size(); ++j) {
					//get point index
					int iSecPointInAllIdx = vPointSecIdxs[i][j];
					//construct point clouds
					pSectorCloud->points.push_back(vSceneCloud.points[iSecPointInAllIdx]);
				}

				if (m_bElevationFlag){
					pcl::PointXYZI oViewGroundP;
					oViewGroundP.x = m_oViewPoint.x;
					oViewGroundP.y = m_oViewPoint.y;
					oViewGroundP.z = m_oViewPoint.z - m_fViewElevation;
					pSectorCloud->points.push_back(oViewGroundP);
				}

				//******Mesh building******
				//***GHPR mesh building***
				GHPR hpdhpr(m_oViewPoint, m_GHPRParam);

				//perform reconstruction
				if(bMultiThread)
					hpdhpr.ComputeMultiThread(pSectorCloud);
				else
					hpdhpr.Compute(pSectorCloud);

				//get the surfaces that are not connected to the viewpoint
				std::vector<pcl::Vertices> vOneFaces;
				vOneFaces = hpdhpr.ConstructSurfaceIdx(false);

				//collect the vertices and faces in each sector
				m_vAllSectorClouds[i] = pSectorCloud;
				m_vAllSectorClouds[i]->push_back(m_oViewPoint);
				m_vAllSectorFaces[i] = vOneFaces;
				
			}else{
				
				pcl::PointCloud<pcl::PointXYZI>::Ptr pSectorCloud(new pcl::PointCloud<pcl::PointXYZI>);
				m_vAllSectorClouds[i] = pSectorCloud;

				std::vector<pcl::Vertices> vOneNewFaces;
				m_vAllSectorFaces[i] = vOneNewFaces;
			
			}//end if vPointSecIdxs[i].size() > m_iSectorMinPNum
		};
	}
	
	if(bMultiThread) {

		//triangular mesh in each sector
		for (int i = 0; i != vPointSecIdxs.size(); ++i)
			thread_pool.emplace_back(thread_func[i]);

		for(int i = 0; i != vPointSecIdxs.size(); ++i)
				thread_pool[i].join(); 
	}
	else {

		for(int i = 0; i != vPointSecIdxs.size(); ++i)
			thread_func[i]();
	}
}

/*=======================================
OutputAllMeshes
Input: none
Outout: MeshModel - mesh of all point clouds
Function: combine mesh from each sector and output mesh results
========================================*/
void ExplicitRec::OutputAllMeshes(pcl::PolygonMesh & MeshModel){

	//new a point idx in scene point clouds
	std::vector<std::vector<int>> vPointInAllDataIdx;
	vPointInAllDataIdx.reserve(m_vAllSectorClouds.size());
	
	std::vector<int> vEmptyVec;
	for (int i = 0; i != m_vAllSectorClouds.size(); ++i)
		vPointInAllDataIdx.push_back(vEmptyVec);

	pcl::PointCloud<pcl::PointXYZI>::Ptr pAllCloud(new pcl::PointCloud<pcl::PointXYZI>);

	int iPNum = 0;
	
	//for each sector
	for (int i = 0; i != m_vAllSectorClouds.size(); ++i){

		//for each point in a sector
		for (int j = 0; j != m_vAllSectorClouds[i]->points.size(); ++j){
			
			//get the point
			pAllCloud->points.push_back(m_vAllSectorClouds[i]->points[j]);
			
			//get point index in all point clouds
			vPointInAllDataIdx[i].push_back(iPNum);
			iPNum++;

		}//end j

	}//end i

	//relaitionships of vertexs in all point clouds
	std::vector<pcl::Vertices> oPolygons;

	//for each sector
	for (int i = 0; i != m_vAllSectorFaces.size(); ++i){
		
		//for each face
		for (int j = 0; j != m_vAllSectorFaces[i].size(); ++j){

			pcl::Vertices oOneFace;

			//for each face vertex id
			for (int k = 0; k != m_vAllSectorFaces[i][j].vertices.size(); ++k){
				
				//vertex id in each sector
				int iVertexSectorIdx = m_vAllSectorFaces[i][j].vertices[k];

				//vertex id in all data
				int iVertexGlobalIdx = vPointInAllDataIdx[i][iVertexSectorIdx];

				//get a vertex
				oOneFace.vertices.push_back(iVertexGlobalIdx);
					
			}//end k

			oPolygons.push_back(oOneFace);
		
		}//end j

	}//end i

	//get the polygon
	pcl::toPCLPointCloud2(*pAllCloud, MeshModel.cloud);
	MeshModel.polygons = oPolygons;

	//pcl::io::savePLYFileBinary("reconstruction_res.ply", MeshModel);

}

void ExplicitRec::OutputSectorMesh(shape_msgs::Mesh& mesh, int sectorId) {

	if(sectorId >= m_vAllSectorClouds.size()){
		ROS_ERROR("SectorId out of size");
		return;
	}

	//new a point idx in scene point clouds
	mesh.vertices.clear();
	mesh.triangles.clear();

	//for each point in a sector
	for (int j = 0; j != m_vAllSectorClouds[sectorId]->points.size(); ++j){
		
		geometry_msgs::Point point;
		point.x = m_vAllSectorClouds[sectorId]->points[j].x;
		point.y = m_vAllSectorClouds[sectorId]->points[j].y;
		point.z = m_vAllSectorClouds[sectorId]->points[j].z;
		mesh.vertices.push_back(point);

	}//end j

	//for each face
	for (int j = 0; j != m_vAllSectorFaces[sectorId].size(); ++j){

		shape_msgs::MeshTriangle triangle;

		//for each face vertex id
		for (int k = 0; k != 3; ++k){
			
			//vertex id in each sector
			int iVertexSectorIdx = m_vAllSectorFaces[sectorId][j].vertices[k];

			triangle.vertex_indices[k] = iVertexSectorIdx;
				
		}//end k

		mesh.triangles.push_back(triangle);

	}//end j
}

/*=======================================
OutputAllMeshes
Input: none
Outout: mesh - mesh msg of all point clouds
========================================*/
void ExplicitRec::OutputAllMeshes(shape_msgs::Mesh & mesh){

	//new a point idx in scene point clouds
	std::vector<std::vector<int>> vPointInAllDataIdx(m_vAllSectorClouds.size(), std::vector<int>());
	mesh.vertices.clear();
	mesh.triangles.clear();

	int iPNum = 0;
	
	//for each sector
	for (int i = 0; i != m_vAllSectorClouds.size(); ++i){

		//for each point in a sector
		for (int j = 0; j != m_vAllSectorClouds[i]->points.size(); ++j){
			
			geometry_msgs::Point point;
			point.x = m_vAllSectorClouds[i]->points[j].x;
			point.y = m_vAllSectorClouds[i]->points[j].y;
			point.z = m_vAllSectorClouds[i]->points[j].z;
			mesh.vertices.push_back(point);

			//get point index in all point clouds
			vPointInAllDataIdx[i].push_back(iPNum);
			iPNum++;

		}//end j

	}//end i

	//for each sector
	for (int i = 0; i != m_vAllSectorFaces.size(); ++i){
		
		//for each face
		for (int j = 0; j != m_vAllSectorFaces[i].size(); ++j){

			shape_msgs::MeshTriangle triangle;

			//for each face vertex id
			for (int k = 0; k != 3; ++k){
				
				//vertex id in each sector
				int iVertexSectorIdx = m_vAllSectorFaces[i][j].vertices[k];

				//vertex id in all data
				int iVertexGlobalIdx = vPointInAllDataIdx[i][iVertexSectorIdx];

				triangle.vertex_indices[k] = iVertexGlobalIdx;
					
			}//end k

			mesh.triangles.push_back(triangle);
		
		}//end j

	}//end i
}

/*=======================================
OutputAllMeshes
Input: none
Outout: vCloud - points presenting face relationship (point repeatable)
Function: output all vertices in a point repeatable way using three-point arrangement
========================================*/
void ExplicitRec::OutputAllMeshes(pcl::PointCloud<pcl::PointXYZI> & vCloud){

	//relaitionships of vertexs in all point clouds
	 vCloud.points.clear();

	//for each sector
	for (int i = 0; i != m_vAllSectorFaces.size(); ++i){

		//for each face
		for (int j = 0; j != m_vAllSectorFaces[i].size(); ++j){

			//for each face vertex id
			for (int k = 0; k != m_vAllSectorFaces[i][j].vertices.size(); ++k){

				//vertex id in each sector
				int iVertexSectorIdx = m_vAllSectorFaces[i][j].vertices[k];

				//vertex id in all data
				vCloud.points.push_back(m_vAllSectorClouds[i]->points[iVertexSectorIdx]);

			}//end k

		}//end j

	}//end i

}

/*=======================================
OutputClouds reload
Input: none
Output: vCloud - all processed point
vNorSectLabels - sector index of each processed point
Function: output the all processed point
========================================*/
void ExplicitRec::OutputClouds(pcl::PointCloud<pcl::PointXYZI> & vCloud){

	vCloud.clear();

	//for each sector
	for (int i = 0; i != m_vAllSectorClouds.size(); ++i){

		//for each point in a sector
		for (int j = 0; j != m_vAllSectorClouds[i]->points.size(); ++j){

			//get the point
			vCloud.points.push_back(m_vAllSectorClouds[i]->points[j]);

		}//end j

	}//end i

}

/*=======================================
OutputClouds reload
Input: none
Output: vCloud - all processed point 
vNorSectLabels - sector index of each processed point 
Function: output processed point with labels
========================================*/
void ExplicitRec::OutputClouds(pcl::PointCloud<pcl::PointXYZI> & vCloud, std::vector<float> & vNorSectLabels){

	vCloud.clear();

	//for each sector
	for (int i = 0; i != m_vAllSectorClouds.size(); ++i){
		
		//compute the normalized sector label
		float fNorSecLabel = float(i) / float(m_iSectNum);

		//for each point in a sector
		for (int j = 0; j != m_vAllSectorClouds[i]->points.size(); ++j){

			//get the point
			vCloud.points.push_back(m_vAllSectorClouds[i]->points[j]);	

			//get the label
			vNorSectLabels.push_back(fNorSecLabel);

		}//end j

	}//end i

}


/*=======================================
CountNumber
Input: iVerticesNum - total number of points/vertices
Outout: iFacesNum - total number of faces
Output: none
Function: count number of points and faces on all sector, respectively
========================================*/
void ExplicitRec::CountNumber(int & iVerticesNum, int & iFacesNum){

	//define output
	iVerticesNum = 0;
	iFacesNum = 0;

	//point number
	for (int i = 0; i != m_vAllSectorClouds.size(); ++i){
		iVerticesNum += m_vAllSectorClouds[i]->points.size();
	}

	//face number
	for (int i = 0; i != m_vAllSectorFaces.size(); ++i){
		iFacesNum += m_vAllSectorFaces[i].size();
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





