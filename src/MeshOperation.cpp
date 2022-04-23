#include "MeshOperation.h"


MeshOperation::MeshOperation(){


}

MeshOperation::~MeshOperation(){



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



