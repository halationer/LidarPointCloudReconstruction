#ifndef MESHOPERATION_H
#define MESHOPERATION_H
#include "ConvexHullOperation.h"


class MeshOperation :public ConvexHullOperation{

public:

	MeshOperation();
	~MeshOperation();

	//using ConvexHullOperation::ComputeCenterPoint;
	//using ConvexHullOperation::ComputeAllFaceParams;

	inline void VectorNormalization(float & fX, float & fY, float & fZ){

		//get norm
		float fNorm = sqrt(fX*fX + fY*fY + fZ*fZ);

		//a = a/|a|
		fX = fX / fNorm;
		fY = fY / fNorm;
		fZ = fZ / fNorm;

	}

	//compute center point of a face
	pcl::PointXYZ ComputeFaceCenterPoint(const pcl::PointXYZ & oA, const pcl::PointXYZ & oB, const pcl::PointXYZ & oC);

	//compute center point of a face
	pcl::PointXYZ ComputeFaceCenterPoint(const pcl::PointCloud<pcl::PointXYZ> & vVertices);

	//all adjacent faces of each vertex
	void AdjacentFaceIdxes(const std::vector<pcl::Vertices> & vMeshVertexIdxs, const int & iVertexNum, std::vector<std::vector<int>> & vPointAdjFaceIdx);
	
	//calculate the local triangular face normal vector based on the output of AdjacentFaceIdxes
	void LocalFaceNormal(const std::vector<std::vector<int>> & vPointAdjFaceIdx, const Eigen::MatrixXf & oMatNormal, pcl::PointCloud<pcl::Normal> & vCombinedNormal);

	//compute the local triangular face normal vector
	void LocalFaceNormal(const pcl::PointCloud<pcl::PointXYZ> & vVertices, const std::vector<pcl::Vertices> & vMeshVertexIdxs, const Eigen::MatrixXf & oMatNormal, const pcl::PointXYZ oViewPoint, pcl::PointCloud<pcl::PointNormal> & vCombinedNormal);

	//compute the local triangular face normal vector
	void LocalFaceNormal(const pcl::PointCloud<pcl::PointXYZ> & vVertices, const std::vector<pcl::Vertices> & vMeshVertexIdxs, const Eigen::MatrixXf & oMatNormal, pcl::PointCloud<pcl::PointNormal> & vCombinedNormal);

	//convert the normal vector represented by the matrix to the normal vector of the point cloud
	void NormalMatrixToPCL(const pcl::PointCloud<pcl::PointXYZ> & vPoints, const Eigen::MatrixXf & oMatNormal, pcl::PointCloud<pcl::PointNormal> & vPointNormal, bool bReversed = false);

	
private:



};



#endif

