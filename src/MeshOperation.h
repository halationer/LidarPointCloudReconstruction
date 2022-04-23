#ifndef MESHOPERATION_H
#define MESHOPERATION_H
#include "ConvexHullOperation.h"


class MeshOperation :public ConvexHullOperation{

public:

	MeshOperation();
	~MeshOperation();

	//using ConvexHullOperation::ComputeCenterPoint;
	//using ConvexHullOperation::ComputeAllFaceParams;

	//compute center point of a face
	pcl::PointXYZ ComputeFaceCenterPoint(const pcl::PointXYZ & oA, const pcl::PointXYZ & oB, const pcl::PointXYZ & oC);

	//compute center point of a face
	pcl::PointXYZ ComputeFaceCenterPoint(const pcl::PointCloud<pcl::PointXYZ> & vVertices);

	
private:



};



#endif

