#ifndef MESHOPERATION_H
#define MESHOPERATION_H
#include <vector>
#include "ConvexHullOperation.h"
#include "Confidence.h"

class MeshOperation :public ConvexHullOperation{

public:

	MeshOperation();
	~MeshOperation();

	//using ConvexHullOperation::ComputeCenterPoint;
	//using ConvexHullOperation::ComputeAllFaceParams;

	inline void VectorNormalization(float & fX, float & fY, float & fZ);

	//compute center point of a face
	pcl::PointXYZI ComputeFaceCenterPoint(const pcl::PointXYZI & oA, const pcl::PointXYZI & oB, const pcl::PointXYZI & oC);

	//compute center point of a face
	pcl::PointXYZI ComputeFaceCenterPoint(const pcl::PointCloud<pcl::PointXYZI> & vVertices);

	//all adjacent faces of each vertex
	void AdjacentFaceIdxes(const std::vector<pcl::Vertices> & vMeshVertexIdxs, const int & iVertexNum, std::vector<std::vector<int>> & vPointAdjFaceIdx);
	
	//calculate the local triangular face normal vector based on the output of AdjacentFaceIdxes
	void LocalFaceNormal(const std::vector<std::vector<int>> & vPointAdjFaceIdx, const Eigen::MatrixXf & oMatNormal, pcl::PointCloud<pcl::Normal> & vCombinedNormal);

	//compute the local triangular face normal vector
	void LocalFaceNormalAndConfidence(const pcl::PointCloud<pcl::PointXYZI> & vVertices, const std::vector<pcl::Vertices> & vMeshVertexIdxs, const Eigen::MatrixXf & oMatNormal, const pcl::PointXYZI oViewPoint, pcl::PointCloud<pcl::PointNormal> & vCombinedNormal);

	//compute the local triangular face normal vector
	void LocalFaceNormalAndConfidence(const pcl::PointCloud<pcl::PointXYZI> & vVertices, const std::vector<pcl::Vertices> & vMeshVertexIdxs, const Eigen::MatrixXf & oMatNormal, const pcl::PointXYZI oViewPoint, const std::vector<Confidence> vFaceWeight, pcl::PointCloud<pcl::PointNormal> & vCombinedNormal);

	//compute the local triangular face normal vector
	void LocalFaceNormal(const pcl::PointCloud<pcl::PointXYZI> & vVertices, const std::vector<pcl::Vertices> & vMeshVertexIdxs, const Eigen::MatrixXf & oMatNormal, pcl::PointCloud<pcl::PointNormal> & vCombinedNormal);

	//convert the normal vector represented by the matrix to the normal vector of the point cloud
	void NormalMatrixToPCL(const pcl::PointCloud<pcl::PointXYZI> & vPoints, const Eigen::MatrixXf & oMatNormal, pcl::PointCloud<pcl::PointNormal> & vPointNormal, bool bReversed = false);

	
private:



};



#endif

