#include "MeshOperation.h"


MeshOperation::MeshOperation(){


}

MeshOperation::~MeshOperation(){



}

/*=======================================
VectorNormalization
Input: fX,fY,fZ - vector coordinate value
Output: fX,fY,fZ - normalized value
Function: Normalize the input and get a unit vector
========================================*/
void MeshOperation::VectorNormalization(float & fX, float & fY, float & fZ){

	//get norm
	float fNorm = sqrt(fX*fX + fY*fY + fZ*fZ);

	//a = a/|a|
	fX = fX / fNorm;
	fY = fY / fNorm;
	fZ = fZ / fNorm;

}



/*=======================================
ComputeCenterPoint
Input: oA, oB, oC - 3 vertices of all faces
Output:oCenterPoint - center point of a triangular face
Function: calculate the center point of each triangular face
========================================*/
pcl::PointXYZ MeshOperation::ComputeFaceCenterPoint(const pcl::PointXYZ & oA, const pcl::PointXYZ & oB, const pcl::PointXYZ & oC){

	pcl::PointXYZ oCenterPoint;
	oCenterPoint.x = 0.0;
	oCenterPoint.y = 0.0;
	oCenterPoint.z = 0.0;

	//accumulation
	//compute the center value among 3 vertices
	//point A
	oCenterPoint.x = oA.x + oCenterPoint.x;
	oCenterPoint.y = oA.y + oCenterPoint.y;
	oCenterPoint.z = oA.z + oCenterPoint.z;
	//point B
	oCenterPoint.x = oB.x + oCenterPoint.x;
	oCenterPoint.y = oB.y + oCenterPoint.y;
	oCenterPoint.z = oB.z + oCenterPoint.z;
	//point C
	oCenterPoint.x = oC.x + oCenterPoint.x;
	oCenterPoint.y = oC.y + oCenterPoint.y;
	oCenterPoint.z = oC.z + oCenterPoint.z;

	//take the average
	oCenterPoint.x = oCenterPoint.x / 3.0f;
	oCenterPoint.y = oCenterPoint.y / 3.0f;
	oCenterPoint.z = oCenterPoint.z / 3.0f;

	return oCenterPoint;

}


/*=======================================
ComputeCenterPoint -  Reload
Input:  vVertices - vertices of all faces
Output:oCenterPoint - center point of a triangular face
Function: calculate the center point of each triangular face
========================================*/
pcl::PointXYZ MeshOperation::ComputeFaceCenterPoint(const pcl::PointCloud<pcl::PointXYZ> & vVertices){

	pcl::PointXYZ oCenterPoint;
	oCenterPoint.x = 0.0;
	oCenterPoint.y = 0.0;
	oCenterPoint.z = 0.0;

	//accumulation
	//compute the center value among vertices
	for (int i = 0; i != vVertices.points.size(); ++i){

		oCenterPoint.x = oCenterPoint.x + vVertices.points[i].x;
		oCenterPoint.y = oCenterPoint.y + vVertices.points[i].y;
		oCenterPoint.z = oCenterPoint.z + vVertices.points[i].z;

	}

	//take the average
	float fNum = float(vVertices.points.size());
	oCenterPoint.x = oCenterPoint.x / fNum;
	oCenterPoint.y = oCenterPoint.y / fNum;
	oCenterPoint.z = oCenterPoint.z / fNum;

	return oCenterPoint;

}


/*=======================================
AdjacentFaceIdxes -  Reload
Input:  vVertices - vertices of all faces
  vMeshVertexIdxs - vertice indexes of each face
Output: vPointAdjFaceIdx - the adjacent face indexes of each vertice
Function: compute the indexes of faces that consist of this point
========================================*/
void MeshOperation::AdjacentFaceIdxes(const std::vector<pcl::Vertices> & vMeshVertexIdxs, const int & iVertexNum, std::vector<std::vector<int>> & vPointAdjFaceIdx){

	//define output
	vPointAdjFaceIdx.clear();
	vPointAdjFaceIdx.reserve(iVertexNum);
	std::vector<int> vEmpty;
	for (int i = 0; i != iVertexNum; ++i)
		vPointAdjFaceIdx.push_back(vEmpty);

	//for each face
	for (int i = 0; i != vMeshVertexIdxs.size(); ++i){
		//for each vertice
		for (int j = 0; j != vMeshVertexIdxs[i].vertices.size(); ++j){
			//get the vertice id
			int iVertexId = vMeshVertexIdxs[i].vertices[j];
			//get the face id for the iVertexId point
			vPointAdjFaceIdx[iVertexId].push_back(i);
		
		}//end j
	
	}//end i

}


/*=======================================
LocalFaceNormal -  Reload
Input:  vPointAdjFaceIdx - the adjacent face indexes of each vertice
              oMatNormal - the unit normal vector of each face
Output: vPointAdjFaceIdx - the adjacent face indexes of each vertice
Function: compute the indexes of faces that consist of this point
========================================*/
//Calculate the local triangular face normal vector
//void MeshOperation::LocalFaceNormal(const std::vector<std::vector<int>> & vPointAdjFaceIdx, const Eigen::MatrixXf & oMatNormal, pcl::PointCloud<pcl::Normal> & vCombinedNormal){
//
//	//define the output
//	vCombinedNormal.clear();
//	vCombinedNormal.reserve(vPointAdjFaceIdx.size());
//
//	//for each vertex
//	for (int i = 0; i != vPointAdjFaceIdx.size(); ++i){
//		
//		Eigen::Vector3f oPointNormal(0.0,0.0,0.0);
//		//traverse its adjacent triangle faces
//		for (int j = 0; j != vPointAdjFaceIdx[i].size(); ++j){
//			//get the normal vector of the adjacent face
//			int iFaceIdx = vPointAdjFaceIdx[i][j];
//			oPointNormal = oPointNormal + oMatNormal.row(iFaceIdx);
//
//		}//end j
//
//		//average
//		oPointNormal = oPointNormal / float(vPointAdjFaceIdx[i].size());
//
//		//convert to PCL format
//		pcl::Normal oPCLNormal;
//		oPCLNormal.normal_x = oPointNormal(0);
//		oPCLNormal.normal_y = oPointNormal(1);
//		oPCLNormal.normal_z = oPointNormal(2);
//		vCombinedNormal.push_back(oPCLNormal);
//	
//	}//end i
//
//
//}

//compute the local triangular face normal vector
/*=======================================
LocalFaceNormalAndConfidence -  Reload
Input:  vVertices - point clouds
        vMeshVertexIdxs - faces (point relationships)
		oMatNormal - face normals
		oViewPoint - viewpoint location
Output: vCombinedNormal - point with its local combined normal
Function: compute the local normal for each point
========================================*/
//Calculate the local triangular face normal vector
void MeshOperation::LocalFaceNormalAndConfidence(const pcl::PointCloud<pcl::PointXYZ> & vVertices,
	                              const std::vector<pcl::Vertices> & vMeshVertexIdxs,
	                                              const Eigen::MatrixXf & oMatNormal,
	                                                  const pcl::PointXYZ oViewPoint,
	                             pcl::PointCloud<pcl::PointNormal> & vCombinedNormal){

	//the vector indicating the normal of point is computed or not
	std::vector<unsigned int> vNormalCount(vVertices.points.size());

	//define output
	vCombinedNormal.clear();
	vCombinedNormal.reserve(vVertices.points.size());
	//initialization
	for (int i = 0; i != vVertices.points.size(); ++i){
		//a point and its normal starting with (0,0,0)
		pcl::PointNormal oPointN;
		oPointN.x = vVertices.points[i].x;
		oPointN.y = vVertices.points[i].y;
		oPointN.z = vVertices.points[i].z;
		oPointN.normal_x = 0.0;
		oPointN.normal_y = 0.0;
		oPointN.normal_z = 0.0;
		vCombinedNormal.points.push_back(oPointN);
	
	}

	//for each face
	for (int i = 0; i != vMeshVertexIdxs.size(); ++i){
		
		//for each vertice
		for (int j = 0; j != vMeshVertexIdxs[i].vertices.size(); ++j){
			
			//get the vertice id
			int iVertexId = vMeshVertexIdxs[i].vertices[j];
			
			//get the face id for the iVertexId point
			vCombinedNormal.points[iVertexId].normal_x += oMatNormal.row(i)(0);
			vCombinedNormal.points[iVertexId].normal_y += oMatNormal.row(i)(1);
			vCombinedNormal.points[iVertexId].normal_z += oMatNormal.row(i)(2);

			//count hit
			vNormalCount[iVertexId] += 1;

		}//end j

	}//end i

	//average
	//for the point whose normal has not been calculated:
	//Mode 1(*), compute ray from point to viewpoint as normals (note: consistently reversed)
	//Mode 2, culling
	for (int i = 0; i != vNormalCount.size(); ++i){
		
		//if the normal of this query point is computed
		if (vNormalCount[i]){
			
			vCombinedNormal.points[i].normal_x /= float(vNormalCount[i]);
			vCombinedNormal.points[i].normal_y /= float(vNormalCount[i]);
			vCombinedNormal.points[i].normal_z /= float(vNormalCount[i]);
			//get unit vector
			VectorNormalization(vCombinedNormal.points[i].normal_x, vCombinedNormal.points[i].normal_y, vCombinedNormal.points[i].normal_z);
		
		//instead by rays if normal vector loss
		}else{
			
			//normal vector facing away from the viewpoint
			vCombinedNormal.points[i].normal_x = vVertices.points[i].x - oViewPoint.x;
			vCombinedNormal.points[i].normal_y = vVertices.points[i].y - oViewPoint.y;
			vCombinedNormal.points[i].normal_z = vVertices.points[i].z - oViewPoint.z;
			
			//get unit vector
			VectorNormalization(vCombinedNormal.points[i].normal_x, vCombinedNormal.points[i].normal_y, vCombinedNormal.points[i].normal_z);

		}

		Eigen::Vector3f oToCenterVec(vCombinedNormal.points[i].x - oViewPoint.x, vCombinedNormal.points[i].y - oViewPoint.y, vCombinedNormal.points[i].z - oViewPoint.z);
		Eigen::Vector3f oPointNormal(vCombinedNormal.points[i].normal_x, vCombinedNormal.points[i].normal_y, vCombinedNormal.points[i].normal_z);
		vCombinedNormal.points[i].data_c[3] = - oToCenterVec.dot(oPointNormal) / oToCenterVec.norm();

	}//end for i

}

/*=======================================
LocalFaceNormal -  Reload
Input:  vVertices - point clouds
vMeshVertexIdxs - faces (point relationships)
oMatNormal - face normals
Output: vCombinedNormal - point with its local combined normal
Function: compute the local normal for each point and remove points without normals
========================================*/
void MeshOperation::LocalFaceNormal(const pcl::PointCloud<pcl::PointXYZ> & vVertices, const std::vector<pcl::Vertices> & vMeshVertexIdxs, const Eigen::MatrixXf & oMatNormal, pcl::PointCloud<pcl::PointNormal> & vCombinedNormal){

	//the vector indicating the normal of point is computed or not
	std::vector<unsigned int> vNormalCount(vVertices.points.size());

	//new a point with normal for start
	pcl::PointCloud<pcl::PointNormal>::Ptr pLocalNormals(new pcl::PointCloud<pcl::PointNormal>);
	pLocalNormals->reserve(vVertices.points.size());

	//initialization
	for (int i = 0; i != vVertices.points.size(); ++i){
		//a point and its normal starting with (0,0,0)
		pcl::PointNormal oPointN;
		oPointN.x = vVertices.points[i].x;
		oPointN.y = vVertices.points[i].y;
		oPointN.z = vVertices.points[i].z;
		oPointN.normal_x = 0.0;
		oPointN.normal_y = 0.0;
		oPointN.normal_z = 0.0;
		pLocalNormals->points.push_back(oPointN);

	}

	//for each face
	for (int i = 0; i != vMeshVertexIdxs.size(); ++i){

		//for each vertice
		for (int j = 0; j != vMeshVertexIdxs[i].vertices.size(); ++j){

			//get the vertice id
			int iVertexId = vMeshVertexIdxs[i].vertices[j];

			//get the face id for the iVertexId point
			pLocalNormals->points[iVertexId].normal_x += oMatNormal.row(i)(0);
			pLocalNormals->points[iVertexId].normal_y += oMatNormal.row(i)(1);
			pLocalNormals->points[iVertexId].normal_z += oMatNormal.row(i)(2);

			//count hit
			vNormalCount[iVertexId] += 1;

		}//end j

	}//end i


	vCombinedNormal.clear();
	//average
	//for the point whose normal has not been calculated:
	//Mode 1, compute ray from point to viewpoint as normals (note: consistently reversed)
	//Mode 2(*), culling the loss
	for (int i = 0; i != vNormalCount.size(); ++i){

		//if the normal of this query point is computed
		if (vNormalCount[i]){

			pLocalNormals->points[i].normal_x /= float(vNormalCount[i]);
			pLocalNormals->points[i].normal_y /= float(vNormalCount[i]);
			pLocalNormals->points[i].normal_z /= float(vNormalCount[i]);
			
			//get unit vector
			VectorNormalization(pLocalNormals->points[i].normal_x, pLocalNormals->points[i].normal_y, pLocalNormals->points[i].normal_z);

			//for output
			vCombinedNormal.points.push_back(pLocalNormals->points[i]);

		}//end if

	}//end for i


}

/*=======================================
NormalMatrixToPCL
Input:  vPoints - the point that is going to carry the normal vector
     oMatNormal - the unit normal vector of each face
Output: vPointNormal - point normal vector in PCL format
Function: convert matrix data to PCL data
========================================*/
//convert the normal vector represented by the matrix to the normal vector of the point cloud
void MeshOperation::NormalMatrixToPCL(const pcl::PointCloud<pcl::PointXYZ> & vPoints, const Eigen::MatrixXf & oMatNormal, pcl::PointCloud<pcl::PointNormal> & vPointNormal, bool bReversed){

	//accumulate all center points and normal vectors
	//vPointNormal.clear();
	//vPointNormal.reserve(vPoints.points.size());
	
	pcl::PointNormal oOnePN;

	//get pcl point and normal
	//The normal direction does not need to be reversed
	if (!bReversed){

		for (int i = 0; i != vPoints.points.size();++i){

		oOnePN.x = vPoints.points[i].x;
		oOnePN.y = vPoints.points[i].y;
		oOnePN.z = vPoints.points[i].z;
		oOnePN.normal_x = oMatNormal.row(i)(0);
		oOnePN.normal_y = oMatNormal.row(i)(1);
		oOnePN.normal_z = oMatNormal.row(i)(2);
		vPointNormal.push_back(oOnePN);

		}//end for i
	//The normal direction need to be reversed
	}else{
	
		for (int i = 0; i != vPoints.points.size(); ++i){

			oOnePN.x = vPoints.points[i].x;
			oOnePN.y = vPoints.points[i].y;
			oOnePN.z = vPoints.points[i].z;
			oOnePN.normal_x = -1.0f*oMatNormal.row(i)(0);
			oOnePN.normal_y = -1.0f*oMatNormal.row(i)(1);
			oOnePN.normal_z = -1.0f*oMatNormal.row(i)(2);
			vPointNormal.push_back(oOnePN);

		}//end for i
	
	}//end else

}


