#include "ConvexHullOperation.h"

//
ConvexHullOperation::ConvexHullOperation(){

	//no param has been computed
	bParamFlag = false;


}


/*=======================================
ChangeNormalType
Input:  vClouds - a point clouds
    vFaceParams - a normal data corresponding to each point
Output:  vPNormals - a point cloud with its normal
Function: transform point and normal to pointnormal
========================================*/
void ConvexHullOperation::ChangeNormalType(const pcl::PointCloud<pcl::PointXYZI> & vClouds, const std::vector<FacePara> & vFaceParams, pcl::PointCloud<pcl::PointNormal> & vPNormals){

	vPNormals.clear();
	vPNormals.reserve(vClouds.points.size());

	for (int i = 0; i != vClouds.points.size(); ++i){
	
		pcl::PointNormal oPNormal;
		oPNormal.x = vClouds.points[i].x;
		oPNormal.y = vClouds.points[i].y;
		oPNormal.z = vClouds.points[i].z;
		oPNormal.normal_x = vFaceParams[i].oNormal(0);
		oPNormal.normal_y = vFaceParams[i].oNormal(1);
		oPNormal.normal_z = vFaceParams[i].oNormal(2);
		vPNormals.push_back(oPNormal);
	
	}


}


/*=======================================
PointToFaceDis
Input: vVecN - normal vector of face
       vVecP - a given 3d point
	   fD - D parameter of a plane equation
Output: the distance with signed value
Function: compute point-to-face distance along with the direction relative to the normal vector
========================================*/
float ConvexHullOperation::PointToFaceDis(const Eigen::Vector3f & vVecP, const Eigen::Vector3f & vVecN,  const float & fD){

	//Calculate the distance and direction of the reference point to the plane
	//d = Ax_ref + By_ref + Cz_ref + D
	return vVecN.dot(vVecP) + fD;

}
/*=======================================
PointToFaceDis - reload
Input: oMatN - a matrix where each row stores the normal vector of a face
       vVecP - a given 3d point
	   vfD - a vector where each row stores the D parameters of a facet
Output: the distance from point to each face with signed value
Function: compute point-to-face distance of all faces along with the direction relative to each normal vector
Due to l = a*x_ref + b*y_ref + c*z_ref + d (n=(a,b,c),p=(x_ref,y_ref,z_ref)), we have l = n(*)p+d ,where (*) is dot product
matrix each variable L = N*P+D
========================================*/
Eigen::VectorXf ConvexHullOperation::PointToFaceDis(const Eigen::Vector3f & vVecP, const Eigen::MatrixXf & oMatN, const Eigen::VectorXf & vfD){

	//Calculate the distance and direction of the reference point to each plane
	//n * 1 = (n*3)*(3*1) + (n*1)
	// L    =   N  *  P   +   D
	Eigen::VectorXf vVecDis = oMatN * vVecP;

	vVecDis = vVecDis + vfD;
	
	//output
	return vVecDis;

}



/*=======================================
CaculateTriangleNormal
Input: oPZero, oPOne, oPTwo - p0, p1, p2 in pcl type
Output: oPCLNormal - normal vector in pcl type
Function: Normal vector caculation of triangle mesh 
========================================*/
pcl::Normal ConvexHullOperation::CaculateTriangleNormal(pcl::PointXYZI & oPZero, 
	                                              pcl::PointXYZI & oPOne,
	                                              pcl::PointXYZI & oPTwo){

	pcl::Normal oPCLNormal;

	//vector A and B
	Eigen::Vector3f oAvec(oPOne.x - oPZero.x, oPOne.y - oPZero.y, oPOne.z - oPZero.z);
	Eigen::Vector3f oBvec(oPTwo.x - oPOne.x, oPTwo.y - oPOne.y, oPTwo.z - oPOne.z);

	//cross product of vector A and vector B
	Eigen::Vector3f oEigenNormal = oAvec.cross(oBvec);
	//compute the norm of normal
	float fLen = oEigenNormal.norm();

	//Normalization
	if (fLen == 0.0){
		//throw Exception();
		std::cout << "something wrong on normal vector calculation!" << std::endl;
	}else{
		//normalization by using norm
		oPCLNormal.normal_x = oEigenNormal(0) / fLen;
		oPCLNormal.normal_y = oEigenNormal(1) / fLen;
		oPCLNormal.normal_z = oEigenNormal(2) / fLen;
	}

	return oPCLNormal;

}


/*=======================================
CaculateTriangleNormal, reload
Input: oPZero, oPOne, oPTwo - p0, p1, p2 in pcl type
       oInnerP - a reference point
Output: oFacePara - normal vector OPPOSITE of reference point and d parameter
Function: triangular plane equation calculation with a base point (reference) 
========================================*/
void ConvexHullOperation::CaculateTriangleNormal(const pcl::PointXYZI & oPZero,
	                                       const pcl::PointXYZI & oPOne, 
	                                       const pcl::PointXYZI & oPTwo,
										   const pcl::PointXYZI & oInnerP,
	                                       FacePara & oFacePara){


	//vector A and B
	Eigen::Vector3f oAvec(oPOne.x - oPZero.x, oPOne.y - oPZero.y, oPOne.z - oPZero.z);
	Eigen::Vector3f oBvec(oPTwo.x - oPOne.x, oPTwo.y - oPOne.y, oPTwo.z - oPOne.z);

	//cross product of vector A and vector B
	oFacePara.oNormal = oAvec.cross(oBvec);
	//compute the norm of normal
	float fLen = oFacePara.oNormal.norm();

	//Normalization
	if (fLen == 0.0){
		//throw Exception();
		std::cout << "something wrong on normal vector calculation!" << std::endl;
	}
	else{
		//normalization by using norm
		oFacePara.oNormal(0) = oFacePara.oNormal(0) / fLen;
		oFacePara.oNormal(1) = oFacePara.oNormal(1) / fLen;
		oFacePara.oNormal(2) = oFacePara.oNormal(2) / fLen;
	}

	//compute the D parameter of plane formula
	//Based on the general equations of the plane: Axi + Byi + Czi + D = 0, we have
	//D =-(Axi + Byi + Czi), i can be 0,1,2,....n (n=2 in hand)
	Eigen::Vector3f oZeroVec(oPZero.x, oPZero.y, oPZero.z);
	//Axi + Byi + Czi is a dot product
	oFacePara.fDparam = oFacePara.oNormal.dot(oZeroVec);
	//take a negative number
	oFacePara.fDparam = -1.0f*oFacePara.fDparam;

	//correct the normal direction
	//make the normal vectors uniformly face outwards based on a reference point
	Eigen::Vector3f oRefPoint(oInnerP.x, oInnerP.y, oInnerP.z);
	//Calculate the distance and direction of the reference point to the plane
	//d = Ax_ref + By_ref + Cz_ref + D
	float fDis = PointToFaceDis(oRefPoint, oFacePara.oNormal, oFacePara.fDparam);

	//If the reference point and the normal direction are in the same side of a plane, 
	//take the opposite direction of the raw normal vector as the new normal vector
	if (fDis > 0){
		//recompute normal vector n + n' = 0
		oFacePara.oNormal(0) = -1.0f* oFacePara.oNormal(0);
		oFacePara.oNormal(1) = -1.0f* oFacePara.oNormal(1);
		oFacePara.oNormal(2) = -1.0f* oFacePara.oNormal(2);
		//recompute D
		oFacePara.fDparam = oFacePara.oNormal.dot(oZeroVec);
		oFacePara.fDparam = -1.0f*oFacePara.fDparam;

	}

	return;

}


/*=======================================
CaculateTriangleNormal
Input: vVertices - vertice point
      oInnerP - a reference point
	  vOneMeshVertexIdxs - vertex indexes
Output: oFacePara - normal vector SAME with reference point and d parameter
        vOneMeshVertexIdxs - vertex indexes after winding the orders
Function: triangular plane equation calculation with a base point (reference) and also change the vertex order
========================================*/
void ConvexHullOperation::CaculateTriangleNormal(const pcl::PointCloud<pcl::PointXYZI> & vVertices,
	                                                                const pcl::PointXYZI & oInnerP,
															   pcl::Vertices & vOneMeshVertexIdxs,
												                             FacePara & oFacePara){

	
	//point cloud 
	pcl::PointXYZI oPZero = vVertices.points[vOneMeshVertexIdxs.vertices[0]];
	pcl::PointXYZI oPOne = vVertices.points[vOneMeshVertexIdxs.vertices[1]];
	pcl::PointXYZI oPTwo = vVertices.points[vOneMeshVertexIdxs.vertices[2]];

	//vector A and B
	Eigen::Vector3f oAvec(oPOne.x - oPZero.x, oPOne.y - oPZero.y, oPOne.z - oPZero.z);
	Eigen::Vector3f oBvec(oPTwo.x - oPOne.x, oPTwo.y - oPOne.y, oPTwo.z - oPOne.z);

	//cross product of vector A and vector B
	oFacePara.oNormal = oAvec.cross(oBvec);
	//compute the norm of normal
	float fLen = oFacePara.oNormal.norm();

	//Normalization
	if (fLen == 0.0){
		//throw Exception();
		std::cout << "something wrong on normal vector calculation!" << std::endl;
	}
	else{
		//normalization by using norm
		oFacePara.oNormal(0) = oFacePara.oNormal(0) / fLen;
		oFacePara.oNormal(1) = oFacePara.oNormal(1) / fLen;
		oFacePara.oNormal(2) = oFacePara.oNormal(2) / fLen;
	}

	//compute the D parameter of plane formula
	//Based on the general equations of the plane: Axi + Byi + Czi + D = 0, we have
	//D =-(Axi + Byi + Czi), i can be 0,1,2,....n (n=2 in hand)
	Eigen::Vector3f oZeroVec(oPZero.x, oPZero.y, oPZero.z);
	//Axi + Byi + Czi is a dot product
	oFacePara.fDparam = oFacePara.oNormal.dot(oZeroVec);
	//take a negative number
	oFacePara.fDparam = -1.0f*oFacePara.fDparam;

	//correct the normal direction
	//make the normal vectors uniformly face outwards based on a reference point
	Eigen::Vector3f oRefPoint(oInnerP.x, oInnerP.y, oInnerP.z);
	//Calculate the distance and direction of the reference point to the plane
	//d = Ax_ref + By_ref + Cz_ref + D
	float fDis = PointToFaceDis(oRefPoint, oFacePara.oNormal, oFacePara.fDparam);

	//If the reference point and the normal direction are in the same side of a plane, 
	//take the opposite direction of the raw normal vector as the new normal vector
	if (fDis < 0){
		//recompute normal vector n + n' = 0
		oFacePara.oNormal(0) = -1.0f* oFacePara.oNormal(0);
		oFacePara.oNormal(1) = -1.0f* oFacePara.oNormal(1);
		oFacePara.oNormal(2) = -1.0f* oFacePara.oNormal(2);
		//recompute D
		oFacePara.fDparam = oFacePara.oNormal.dot(oZeroVec);
		oFacePara.fDparam = -1.0f*oFacePara.fDparam;

		WindingOrder(vOneMeshVertexIdxs);

	}

	return;

}

/*=======================================
CaculateTriangleNormal
Input: oPZero, oPOne, oPTwo - p0, p1, p2 in pcl type
oInnerP - a reference point
Output: oFacePara - normal vector and d parameter
Function: triangular plane equation calculation with a base point (reference)
========================================*/
void ConvexHullOperation::CaculateTriangleNormal(const Eigen::Vector3f & oRefPoint, const pcl::PointNormal & oPNormal,
	                                             pcl::PointXYZI & oPoint, FacePara & oFacePara){


	//compute the D parameter of plane formula
	//Based on the general equations of the plane: Axi + Byi + Czi + D = 0, we have
	//D =-(Axi + Byi + Czi), i can be 0,1,2,....n (n=2 in hand)
	Eigen::Vector3f oZeroVec(oPNormal.x, oPNormal.y, oPNormal.z);
	oPoint.x = oPNormal.x;
	oPoint.y = oPNormal.y;
	oPoint.z = oPNormal.z;

	//Axi + Byi + Czi is a dot product
	oFacePara.oNormal(0) = oPNormal.normal_x;
	oFacePara.oNormal(1) = oPNormal.normal_y;
	oFacePara.oNormal(2) = oPNormal.normal_z;
	oFacePara.fDparam = oFacePara.oNormal.dot(oZeroVec);
	//take a negative number
	oFacePara.fDparam = -1.0f*oFacePara.fDparam;

	//correct the normal direction
	//make the normal vectors uniformly face outwards based on a reference point
	//Calculate the distance and direction of the reference point to the plane
	//d = Ax_ref + By_ref + Cz_ref + D
	float fDis = PointToFaceDis(oRefPoint, oFacePara.oNormal, oFacePara.fDparam);

	//If the reference point and the normal direction are in the same side of a plane, 
	//take the opposite direction of the raw normal vector as the new normal vector
	if (fDis < 0){
		//recompute normal vector n + n' = 0
		oFacePara.oNormal(0) = -1.0f* oFacePara.oNormal(0);
		oFacePara.oNormal(1) = -1.0f* oFacePara.oNormal(1);
		oFacePara.oNormal(2) = -1.0f* oFacePara.oNormal(2);
		//recompute D
		oFacePara.fDparam = oFacePara.oNormal.dot(oZeroVec);
		oFacePara.fDparam = -1.0f*oFacePara.fDparam;
	}

	return ;

}

/*=======================================
CaculateTriangleNormal
Input: oPZero, oPOne, oPTwo - p0, p1, p2 in pcl type
Output: oFacePara - normal vector and d parameter
Function: triangular plane equation calculation
========================================*/
void ConvexHullOperation::CaculateTriangleNormal(const pcl::PointXYZI & oPZero,
	                                             const pcl::PointXYZI & oPOne,
	                                             const pcl::PointXYZI & oPTwo,
	                                             FacePara & oFacePara){


	//vector A and B
	Eigen::Vector3f oAvec(oPOne.x - oPZero.x, oPOne.y - oPZero.y, oPOne.z - oPZero.z);
	Eigen::Vector3f oBvec(oPTwo.x - oPOne.x, oPTwo.y - oPOne.y, oPTwo.z - oPOne.z);

	//cross product of vector A and vector B
	oFacePara.oNormal = oAvec.cross(oBvec);
	//compute the norm of normal
	float fLen = oFacePara.oNormal.norm();

	//Normalization
	if (fLen == 0.0){
		//throw Exception();
		std::cout << "something wrong on normal vector calculation!" << std::endl;
	}
	else{
		//normalization by using norm
		oFacePara.oNormal(0) = oFacePara.oNormal(0) / fLen;
		oFacePara.oNormal(1) = oFacePara.oNormal(1) / fLen;
		oFacePara.oNormal(2) = oFacePara.oNormal(2) / fLen;
	}

	//compute the D parameter of plane formula
	//Based on the general equations of the plane: Axi + Byi + Czi + D = 0, we have
	//D =-(Axi + Byi + Czi), i can be 0,1,2,....n (n=2 in hand)
	Eigen::Vector3f oZeroVec(oPZero.x, oPZero.y, oPZero.z);
	//Axi + Byi + Czi is a dot product
	oFacePara.fDparam = oFacePara.oNormal.dot(oZeroVec);
	//take a negative number
	oFacePara.fDparam = -1.0f*oFacePara.fDparam;

	return;

}

/*=======================================
ComputeCenterPoint
Input: vVertices - vertices/points of all faces
 vMeshVertexIdxs - vertex idx of each face
Output: vCenterPoints - center point of each face
Function: calculate the center point of each triangular face
========================================*/
void ConvexHullOperation::ComputeCenterPoint(const pcl::PointCloud<pcl::PointXYZI> & vVertices, const std::vector<pcl::Vertices> & vMeshVertexIdxs,
	                                   pcl::PointCloud<pcl::PointXYZI> & vCenterPoints){

	vCenterPoints.clear();
	vCenterPoints.reserve(vMeshVertexIdxs.size());

	//compute the center value among vertices
	for (int i = 0; i != vMeshVertexIdxs.size(); ++i){
		
		pcl::PointXYZI oCenterPoint;
		oCenterPoint.x = 0.0;
		oCenterPoint.y = 0.0;
		oCenterPoint.z = 0.0;

		//accumulation
		//in fact, for a triangular face, the number of vertices is three, e.g., j<=2
		for (int j = 0; j != vMeshVertexIdxs[i].vertices.size(); ++j){
			oCenterPoint.x = oCenterPoint.x + vVertices.points[vMeshVertexIdxs[i].vertices[j]].x;
			oCenterPoint.y = oCenterPoint.y + vVertices.points[vMeshVertexIdxs[i].vertices[j]].y;
			oCenterPoint.z = oCenterPoint.z + vVertices.points[vMeshVertexIdxs[i].vertices[j]].z;
		}

		//take the average
		float fNum = float(vMeshVertexIdxs[i].vertices.size());
		oCenterPoint.x = oCenterPoint.x / fNum;
		oCenterPoint.y = oCenterPoint.y / fNum;
		oCenterPoint.z = oCenterPoint.z / fNum;

		vCenterPoints.push_back(oCenterPoint);
	
	}

}
/*=======================================
ComputeCenterPoint -  Reload
Input: vVertices - vertices/points of all faces
Output: vCenterPoints - center point of all vertices
Function: calculate the center point of each triangular face
****Note that this function does not consider density. In fact, the vertices of all triangular faces are non-uniform.
However, this does not prevent the properties of the center point of the convex hull
========================================*/
void ConvexHullOperation::ComputeCenterPoint(const pcl::PointCloud<pcl::PointXYZI> & vVertices, pcl::PointXYZI & oCenterPoint){

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

}


/*=======================================
ComputeAllFaceParams - reload
Input: vVertices - vertices of all triangular planes (no duplicates)
       vMeshVertexIdxs - vertex index for each face, p.s., each face has three vertices
	   m_oCenterPoint - center point of all faces as reference point
Output: vFaces - parameters for each face
Function: compute the parameters of all plane equations and save them in matrix form, which is convenient for subsequent calculations
========================================*/
void ConvexHullOperation::ComputeAllFaceParams(const pcl::PointCloud<pcl::PointXYZI> & vVertices, 
	                                           const std::vector<pcl::Vertices> & vMeshVertexIdxs, 
											   pcl::PointXYZI & oCenterPoint,
	                                           std::vector<FacePara> & vFaces){

	//the number of faces
	int iFaceNum = vMeshVertexIdxs.size();

	//set the number to matrix and vector
	vFaces.clear();
	vFaces.reserve(iFaceNum);

	//get the center point of all faces
	ComputeCenterPoint(vVertices, oCenterPoint);

	//to each face
	for (int i = 0; i != vMeshVertexIdxs.size(); ++i){

		//compute the parameters of plane equation 
		//the normal vector consistently faces outside the face (different side from center point)
		FacePara oOneFaceParam;
		CaculateTriangleNormal(vVertices.points[vMeshVertexIdxs[i].vertices[0]],
			                   vVertices.points[vMeshVertexIdxs[i].vertices[1]],
			                   vVertices.points[vMeshVertexIdxs[i].vertices[2]],oCenterPoint,oOneFaceParam);

		//get normal vector
		//**test**Check if it is possible to directly assign a column vector to a row vector in Eigen lib 
		vFaces.push_back(oOneFaceParam);

	}

}

/*=======================================
ComputeAllFaceParams
Input: vVertices - vertices of all triangular planes (no duplicates)
vMeshVertexIdxs - vertex index for each face, p.s., each face has three vertices
Output: m_oCenterPoint - center point of all faces as reference point
m_oMatN - a matrix of all face vectors
m_vfD - A vector of D parameter values
Function: compute the parameters of all plane equations and save them in matrix form, which is convenient for subsequent calculations
========================================*/
void ConvexHullOperation::ComputeAllFaceParams(const pcl::PointCloud<pcl::PointXYZI> & vVertices, 
	                                           const std::vector<pcl::Vertices> & vMeshVertexIdxs,
                                               pcl::PointXYZI & oCenterPoint,
											   Eigen::MatrixXf & oMatNormal,
											   Eigen::VectorXf & vfDParam){

	//the number of faces
	int iFaceNum = vMeshVertexIdxs.size();

	//set the number to matrix and vector
	oMatNormal.resize(iFaceNum, 3);
	vfDParam.resize(iFaceNum);

	//get the center point of all faces
	ComputeCenterPoint(vVertices, oCenterPoint);

	//to each face
	for (int i = 0; i != vMeshVertexIdxs.size(); ++i){

		//compute the parameters of plane equation 
		//the normal vector consistently faces outside the face (different side from center point)
		FacePara oOneFaceParam;
		CaculateTriangleNormal(vVertices.points[vMeshVertexIdxs[i].vertices[0]],
			vVertices.points[vMeshVertexIdxs[i].vertices[1]],
			vVertices.points[vMeshVertexIdxs[i].vertices[2]], oCenterPoint, oOneFaceParam);

		//get normal vector
		//**test**Check if it is possible to directly assign a column vector to a row vector in Eigen lib 
		oMatNormal.row(i) = oOneFaceParam.oNormal;
		//get D parameter
		vfDParam(i) = oOneFaceParam.fDparam;

	}

	//parameters have beeb computed
	bParamFlag = true;

}


/*=======================================
ComputeAllFaceParams
Input: oViewPoint - a reference point
        vVertices - points
		vMeshVertexIdxs - vertex index
Output: vMeshVertexIdxs - a series of mesh vertice index after winding each face vertex orders
             oMatNormal - a matrix of all face vectors
               vfDParam - A vector of D parameter values
Function: compute the parameters of all plane equations and save them in matrix form, which is convenient for subsequent calculations
========================================*/
void ConvexHullOperation::ComputeAllFaceParams(const pcl::PointXYZI & oViewPoint,
	                                           const pcl::PointCloud<pcl::PointXYZI> & vVertices,
	                                           std::vector<pcl::Vertices> & vMeshVertexIdxs,
											   Eigen::MatrixXf & oMatNormal,
											   Eigen::VectorXf & vfDParam){

	//the number of faces
	int iFaceNum = vMeshVertexIdxs.size();

	//set the number to matrix and vector
	oMatNormal.resize(iFaceNum, 3);
	vfDParam.resize(iFaceNum);

	//to each face
	for (int i = 0; i != vMeshVertexIdxs.size(); ++i){

		//compute the parameters of plane equation 
		//the normal vector consistently faces outside the face (different side from center point)
		FacePara oOneFaceParam;

		//compute the normal and change vertex order
		CaculateTriangleNormal(vVertices, oViewPoint, vMeshVertexIdxs[i], oOneFaceParam);

		//get normal vector
		//**test**Check if it is possible to directly assign a column vector to a row vector in Eigen lib 
		oMatNormal.row(i) = oOneFaceParam.oNormal;
		//get D parameter
		vfDParam(i) = oOneFaceParam.fDparam;

	}

	//parameters have beeb computed
	bParamFlag = true;

}

/*=======================================
ComputeAllFaceParams
Input: vVertices - vertices of all triangular planes (no duplicates)
vMeshVertexIdxs - vertex index for each face, p.s., each face has three vertices
Output: m_oCenterPoint - center point of all faces as reference point
m_oMatN - a matrix of all face vectors
m_vfD - A vector of D parameter values
Function: compute the parameters of all plane equations and save them in matrix form, which is convenient for subsequent calculations
========================================*/
void ConvexHullOperation::ComputeAllFaceParams(const pcl::PointXYZI & oViewPoint,const pcl::PointCloud<pcl::PointNormal> & vPNormals,
	                                                   pcl::PointCloud<pcl::PointXYZI> & vClouds, std::vector<FacePara> & vFaceParams){

	//define output
	vFaceParams.clear();
	vFaceParams.reserve(vPNormals.points.size());
	vClouds.clear();
	vClouds.reserve(vPNormals.points.size());

	//reference point
	Eigen::Vector3f oRefPoint(oViewPoint.x, oViewPoint.y, oViewPoint.z);

	//to each face
	for (int i = 0; i != vPNormals.size(); ++i){

		//set point and parameters
		pcl::PointXYZI oPoint;
		FacePara oFacePara;

		//the normal vector consistently faces outside the face (different side from reference point)
		CaculateTriangleNormal(oRefPoint, vPNormals[i], oPoint, oFacePara);

		//get parameters of face in which the nearest point is
		vFaceParams.push_back(oFacePara);
		//divide point
		vClouds.push_back(oPoint);

	}

	//parameters have beeb computed
	bParamFlag = true;

}

/*=======================================
ComputeAllFaceParams
Input: vVertices - vertices of all triangular planes (no duplicates)
vMeshVertexIdxs - vertex index for each face, p.s., each face has three vertices
Output: m_oMatN - a matrix of all face vectors
        m_vfD - A vector of D parameter values
Function: Plane parameter calculations without a reference point, so that orientation of normals can not be calibrated
========================================*/
void ConvexHullOperation::ComputeAllFaceParams(const pcl::PointCloud<pcl::PointXYZI> & vVertices,
	                                           const std::vector<pcl::Vertices> & vMeshVertexIdxs,
	                                           Eigen::MatrixXf & oMatNormal,
	                                           Eigen::VectorXf & vfDParam){

	//the number of faces
	int iFaceNum = vMeshVertexIdxs.size();

	//set the number to matrix and vector
	oMatNormal.resize(iFaceNum, 3);
	vfDParam.resize(iFaceNum);

	//to each face
	for (int i = 0; i != vMeshVertexIdxs.size(); ++i){

		//compute the parameters of plane equation 
		//the normal vector consistently faces outside the face (different side from center point)
		FacePara oOneFaceParam;
		CaculateTriangleNormal(vVertices.points[vMeshVertexIdxs[i].vertices[0]],
			                   vVertices.points[vMeshVertexIdxs[i].vertices[1]],
			                   vVertices.points[vMeshVertexIdxs[i].vertices[2]], 
							   oOneFaceParam);

		//get normal vector
		//**test**Check if it is possible to directly assign a column vector to a row vector in Eigen lib 
		oMatNormal.row(i) = oOneFaceParam.oNormal;
		//get D parameter
		vfDParam(i) = oOneFaceParam.fDparam;

	}

}

/*=======================================
ComputeAllFaceParams
Input: vConvertVertices - converted point
	   vConvertMeshIdxs - face vertiex index 
	   vEuclidVertices - mesh vertiex, note that triangle vertices consist of the original point cloud and the viewpoint
	   vEuclidMeshIdxs - reconstructed surface vertiex index
Output: m_oConvertMatN - N_c
           m_vConvertD - D_c
         m_oEuclidMatN - N_e
            m_vEuclidD - D_e
Function: Compute parametric information for all planes in two spaces
========================================*/
void ConvexHullOperation::FaceParamInTwoSpace(const pcl::PointCloud<pcl::PointXYZI> & vConvertVertices,
	                                          const std::vector<pcl::Vertices> & vConvertMeshIdxs,
	                                          const pcl::PointCloud<pcl::PointXYZI> & vEuclidVertices,
											  const std::vector<pcl::Vertices> & vEuclidMeshIdxs){

	//Calculate the face parameters in transform/converted space with a inner point
	ComputeAllFaceParams(vConvertVertices, vConvertMeshIdxs, m_oCenterPoint, m_oConvertMatN, m_vConvertD);

	//Calculate the face parameters in the global Euclidean space
	ComputeAllFaceParams(vEuclidVertices, vEuclidMeshIdxs, m_oEuclidMatN, m_vEuclidD);

	//parameters have beeb computed
	bParamFlag = true;

}

/*=======================================
NDToFacePara
Input: oMatN - normal
       vDParam - d	   
Output: vFaces - normal and d
Function: combines noraml and d
========================================*/
void ConvexHullOperation::NDToFacePara(const Eigen::MatrixXf & oMatN, const Eigen::VectorXf & vDParam, std::vector<FacePara> & vFaces){

	//get space
	vFaces.reserve(oMatN.rows());

	//transformation
	for (int i = 0; i != oMatN.rows(); ++i){
		FacePara oOneParam;
		oOneParam.oNormal = oMatN.row(i);
		oOneParam.fDparam = vDParam(i);
		vFaces.push_back(oOneParam);
	}

}


/*=======================================
JudgeSignedValue
Input: vAllDis - distance of a query point to all faces
Output: oKeyDis - a signed distance judged by distance from point to each face 
Function: set up the algorithm/rule of signed distance and use it for calculation IN TRANSFORMED COORDINATES
========================================*/
//get shortest distance and interior point situation from all point-to-face distances
SignedDis ConvexHullOperation::JudgeSignedValue(const Eigen::VectorXf & vAllDis){

	//key information for point-to-face distance
	SignedDis oKeyDis;

	float fMin = FLT_MAX;
	//rule to judge the signed distance
	//1. All distances are less than zero, then it is an interior point, l<0
	//2. take the minimum value and keep the sign as final signed distance. 
	//Therefore, the negative sign indicates the inner/inside point, otherwise indicates the outside point
	for (int i = 0; i != vAllDis.rows(); ++i){
		
		//for case 1 
		if (vAllDis(i) > 0)
			oKeyDis.bInner = false;
		
		//for case 2
		if (fMin > fabs(vAllDis(i))){
			//get the mimimum value
			fMin = fabs(vAllDis(i));
			//get the signed distance
			oKeyDis.fDis = vAllDis(i);
			//get the face index
			oKeyDis.iFaceNum = i;
		}//end if

	
	}//end for i

	return oKeyDis;

}

/*=======================================
ComputePointSignedDis  -- reload
Input: vConvexSD - the prior distance, mainly marking the occlusion relationship
vQueryCloud - vertex index for each face, p.s., each face has three vertices
vFaceParams - parameters of all surfaces to be compared
Output: vAllPointSignedDis - new signed distances computed in Euclidean space
Function: Based on the occlusion relations obtained in the transformed space,
the function computes the signed distance from each query point to the reconstructed surface
========================================*/
std::vector<float> ConvexHullOperation::ComputePointSignedDis(const std::vector<SignedDis> & vConvexSD,
	const pcl::PointCloud<pcl::PointXYZI> & vQueryCloud,
	const std::vector<FacePara> & vFaceParams){


	//define output
	std::vector<float> vAllPointSignedDis;

	//compute the signed distance of each query point based on a priori (vConvexSD)
	for (int i = 0; i != vQueryCloud.points.size(); ++i){
		//The nearest face corresponding to the point
		int iFaceNum = vConvexSD[i].iFaceNum;
		//the query vector
		Eigen::Vector3f vVecP(vQueryCloud.points[i].x, vQueryCloud.points[i].y, vQueryCloud.points[i].z);
		//get a point to face distance
		float fOneSignedDis = vVecP.dot(vFaceParams[iFaceNum].oNormal) + vFaceParams[iFaceNum].fDparam;
		//give positive and negative values based on the computed occlusion relationship
		if (vConvexSD[i].bInner)
			fOneSignedDis = -1.0f*fabs(fOneSignedDis);//visible
		else
			fOneSignedDis = fabs(fOneSignedDis);//occluded
		//get each distance
		vAllPointSignedDis.push_back(fOneSignedDis);

	}

	return vAllPointSignedDis;

}

/*=======================================
ComputePointSignedDis  -- reload
Input: vConvexSD - the prior distance, mainly marking the occlusion relationship
vQueryCloud - vertex index for each face, p.s., each face has three vertices
vFaceParams - parameters of all surfaces to be compared
Output: vAllPointSignedDis - new signed distances computed in Euclidean space
Function: Based on the occlusion relations obtained in the transformed space,
the function computes the signed distance from each query point to the reconstructed surface
========================================*/
std::vector<float> ConvexHullOperation::ComputePointSignedDis(const pcl::PointCloud<pcl::PointXYZI> & vQueryCloud, const Eigen::MatrixXf & m_oMatN, const Eigen::VectorXf & m_vD){

	//define output
	std::vector<float> vAllPointSignedDis;

	//If the face parameters have not been calculated
	//return null
	if (!bParamFlag){
		std::cout << "WARNING! For convex hull operations, the normal vector of face has not been computed!" << std::endl;
		return vAllPointSignedDis;
	}

	//compute each point signed distance based on the rule 
	for (int i = 0; i != vQueryCloud.points.size(); ++i){

		Eigen::Vector3f vVecP(vQueryCloud.points[i].x, vQueryCloud.points[i].y, vQueryCloud.points[i].z);
		//get all point-to-face distance
		Eigen::VectorXf vAllDis = PointToFaceDis(vVecP, m_oMatN, m_vD);
		//get the signed distance based on rule
		SignedDis oOneSD = JudgeSignedValue(vAllDis);
		//get the judged value as final distance
		vAllPointSignedDis.push_back(oOneSD.fDis);

	}

	return vAllPointSignedDis;

}

/*=======================================
ComputePointSignedDis  -- reload
Input: vConvexSD - the prior distance, mainly marking the occlusion relationship
vQueryCloud - vertex index for each face, p.s., each face has three vertices
vFaceParams - parameters of all surfaces to be compared
Output: vAllPointSignedDis - new signed distances computed in Euclidean space
Function: Based on the occlusion relations obtained in the transformed space,
the function computes the signed distance from each query point to the reconstructed surface
========================================*/
std::vector<float> ConvexHullOperation::ComputePointSignedDis(const std::vector<SignedDis> & vConvexSD,
	                                       const pcl::PointCloud<pcl::PointXYZI> & vQueryCloud,
	                                       const pcl::PointCloud<pcl::PointXYZI> & vCenterPoints){


	//define output
	std::vector<float> vAllPointSignedDis;

	//compute the signed distance of each query point based on a priori (vConvexSD)
	for (int i = 0; i != vQueryCloud.points.size(); ++i){
		
		//The nearest face corresponding to the point
		int iFaceNum = vConvexSD[i].iFaceNum;
	
		//get a point to center point of the nearest face 
		float fOneSignedDis = (vQueryCloud.points[i].x - vCenterPoints[iFaceNum].x)*(vQueryCloud.points[i].x - vCenterPoints[iFaceNum].x)
			+ (vQueryCloud.points[i].y - vCenterPoints[iFaceNum].y)*(vQueryCloud.points[i].y - vCenterPoints[iFaceNum].y)
			+ (vQueryCloud.points[i].z - vCenterPoints[iFaceNum].z)*(vQueryCloud.points[i].z - vCenterPoints[iFaceNum].z);
		fOneSignedDis = sqrt(fOneSignedDis);

		//give positive and negative values based on the computed occlusion relationship
		if (vConvexSD[i].bInner)
			fOneSignedDis = -1.0f*fOneSignedDis;//visible
		else
			fOneSignedDis = fOneSignedDis;//occluded
		//get each distance
		vAllPointSignedDis.push_back(fOneSignedDis);

	}

	return vAllPointSignedDis;

}


/*=======================================
ComputePointSignedDis
Input: vQueryCloud - the query point / node / voxel corner who is going to be calculated the distance to the face
Output: vAllPointSignedDis - signed distance for each query point
Function: Compute the signed distance of each query point based on the KNOWN surface parameters 
========================================*/
std::vector<SignedDis> ConvexHullOperation::ComputePointSignedDis(pcl::PointCloud<pcl::PointXYZI> & vQueryCloud){

	std::vector<SignedDis> vAllPointSignedDis;

	//If the face parameters have not been calculated
	//return null
	if (!bParamFlag){
		std::cout << "WARNING! For convex hull operations, the normal vector of face has not been computed!" << std::endl;
		return vAllPointSignedDis;
	}

	//compute each point signed distance based on the rule 
	for (int i = 0; i != vQueryCloud.points.size(); ++i){

		Eigen::Vector3f vVecP(vQueryCloud.points[i].x, vQueryCloud.points[i].y, vQueryCloud.points[i].z);
		//get all point-to-face distance
		Eigen::VectorXf vAllDis = PointToFaceDis(vVecP, m_oConvertMatN, m_vConvertD);
		//get the signed distance based on rule
		SignedDis oOneSD = JudgeSignedValue(vAllDis);
		//get the judged value as final distance
		vAllPointSignedDis.push_back(oOneSD);

	}

	return vAllPointSignedDis;

}

/*=======================================
ComputePointSignedDis  -- reload, but useless
Input: vConvexSD - the prior distance, mainly marking the occlusion relationship
vQueryCloud - vertex index for each face, p.s., each face has three vertices
vFaceParams - parameters of all surfaces to be compared
vVertices - query point set
vMeshVertexIdxs - face vertex index
Output: vAllPointSignedDis - new signed distances computed in Euclidean space
Function: Based on the occlusion relations obtained in the transformed space,
the function computes the signed distance from each query point to the reconstructed surface
========================================*/
std::vector<float> ConvexHullOperation::ComputePointSignedDis(const std::vector<SignedDis> & vConvexSD,
	                                                          const pcl::PointCloud<pcl::PointXYZI> & vQueryCloud){


	//define output
	std::vector<float> vAllPointSignedDis;

	//If the face parameters have not been calculated
	//return null
	if (!bParamFlag){
		std::cout << "WARNING! For convex hull operations, the normal vector of face has not been computed!" << std::endl;
		return vAllPointSignedDis;
	}

	//compute the signed distance of each query point based on a priori (vConvexSD)
	for (int i = 0; i != vQueryCloud.points.size(); ++i){

		//the query vector
		Eigen::Vector3f vVecP(vQueryCloud.points[i].x, vQueryCloud.points[i].y, vQueryCloud.points[i].z);
		//get a point to face distance
		Eigen::VectorXf vOneToAllDis = PointToFaceDis(vVecP, m_oEuclidMatN, m_vEuclidD);
		//abs
		vOneToAllDis = vOneToAllDis.array().abs();
		//get the minimum value as nearest distance value
		Eigen::VectorXf::Index oMinRow, oMinCol;
		float fOneSignedDis = vOneToAllDis.minCoeff(&oMinRow, &oMinCol);
		//give positive and negative values based on the computed occlusion relationship
		if (vConvexSD[i].bInner)
			fOneSignedDis = -1.0f*fOneSignedDis;//visible

		//get each distance
		vAllPointSignedDis.push_back(fOneSignedDis);

	}

	return vAllPointSignedDis;

}

/*=======================================
GetPCLNormal
Input: m_oConvertMatN - obtained normal vector of each face
Output: pFaceNormal - the normal vector of all faces
Function: convert normal vectors to PCL format
========================================*/
void ConvexHullOperation::GetPCLNormal(pcl::PointCloud<pcl::Normal>::Ptr & pFaceNormal){

	//Each row represents a normal vector
	for (int i = 0; i != m_oConvertMatN.rows(); ++i){
		//pcl
		pcl::Normal oOneNormal;
		oOneNormal.normal_x = m_oConvertMatN(i, 0);
		oOneNormal.normal_y = m_oConvertMatN(i, 1);
		oOneNormal.normal_z = m_oConvertMatN(i, 2);
		pFaceNormal->push_back(oOneNormal);
	}

}




/*=======================================
WindingOrder
Input: pcl::Vertices - triangle vertex
Output: pcl::Vertices - Triangle vertices after changing order
Function: change the order of triangle vertices
========================================*/
void ConvexHullOperation::WindingOrder(pcl::Vertices & vVertices){

	//exchange the vertice orders
	int iTemp = vVertices.vertices[1];

	vVertices.vertices[1] = vVertices.vertices[2];

	vVertices.vertices[2] = iTemp;

}