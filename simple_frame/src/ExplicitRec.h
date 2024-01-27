#ifndef EXPLICITREC_H
#define EXPLICITREC_H

#include "GHPR.h"
#include "SectorPartition.h"
#include "MeshOperation.h"

#include <shape_msgs/Mesh.h>

#define PI 3.1415926

class ExplicitRecParam{

public:

	ExplicitRecParam() :m_GHPRParam(3.6), m_fPseudoFaceThr(0.02), m_iSectorMinPNum(5){


	};

	~ExplicitRecParam(){
	};

	//ghpr threshold
	float m_GHPRParam;

	//scan angle threshold for removing pseudo planes
	float m_fPseudoFaceThr;

	//the min point number for convex hull
	int m_iSectorMinPNum;


	void setWorkingFrameCount(int count) {m_working_frame_count = count;}

protected:
	int m_working_frame_count;
};


class ExplicitRec :public ExplicitRecParam{

public:

	ExplicitRec();
	~ExplicitRec();

	void SetMultiThread(const bool bMultiThread) { m_bMultiThread = bMultiThread; }

	//set the viewpoint location
	void SetViewPoint(const pcl::PointXYZI & oViewPoint);

	//reload, set the viewpoint location and its actual elevation 
	//the height of the lidar from the ground, the lidar is usually installed on the vehicle with constant height
	void SetViewPoint(const pcl::PointXYZI & oViewPoint, float fViewElevation);

	//set the horizontal angle interval size with the given number of divisions
	void HorizontalSectorSize(int iSectNum);
	//reload, set the horizontal angle interval size with the given horizontal angle size
	void HorizontalSectorSize(float fHorizontalRes);

	//Remove pseudo planes from reconstruction
	void RemovePseudoFaces(const pcl::PointCloud<pcl::PointXYZI> & vCenterPoints, const std::vector<pcl::Vertices> & vOneFaces, const Eigen::MatrixXf & oMatNormal,
		                   std::vector<bool> & vTrueFaceStatus, std::vector<float> & vFaceWeight);

	//reconstruction of one frame scanning point cloud and ouput mesh normal
	void FrameReconstruction(const pcl::PointCloud<pcl::PointXYZI> & vSceneCloud, pcl::PointCloud<pcl::PointNormal> & vScenePNormal, const int line_min = 0, const int line_max = 15);
	void OriginalReconstruction(const pcl::PointCloud<pcl::PointXYZI> & vSceneCloud, pcl::PointCloud<pcl::PointNormal> & vScenePNormal, const int line_min, const int line_max);

	//combine and output all vertices
	void OutputAllMeshes(pcl::PolygonMesh & MeshModel);
	//output shape_msg/Mesh for alogrithm
	void OutputSectorMesh(shape_msgs::Mesh & MeshModel, int sectorId);
	void OutputAllMeshes(shape_msgs::Mesh & mesh);
	//reload, output all vertices in a point repeatable way using three-point arrangement
	void OutputAllMeshes(pcl::PointCloud<pcl::PointXYZI> & vCloud);

	//output point clouds
	void OutputClouds(pcl::PointCloud<pcl::PointXYZI> & vCloud);
	//output point clouds with features
	void OutputClouds(pcl::PointCloud<pcl::PointXYZI> & vCloud, std::vector<float> & vNorSectLabel);

	//count the number of points
	void CountNumber(int & iVerticesNum, int & iFacesNum);

	//remove one frame data
	void ClearData();

	//***data***
	//each sector vertices
	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> m_vAllSectorClouds;

	//each sector face (vertice relationships)
	std::vector<std::vector<pcl::Vertices>> m_vAllSectorFaces;

	//center point of face with normal
	pcl::PointCloud<pcl::PointNormal>::Ptr m_pCenterNormal;

	std::vector<Eigen::MatrixXf> m_vMatNormal;
	std::vector<std::vector<float>> m_vFaceWeight;

private:

	//number of sectors
	int m_iSectNum;

	//horizontal angle resolution for sector division
	float m_fHorizontalRes;

	//the elevation value (height) of the viewpoint in global coordinates
	float m_fViewElevation;

	//the location of viewpoint in global coordinates
	pcl::PointXYZI m_oViewPoint;

	//the elevation value of the viewpoint has been obtained
	bool m_bElevationFlag;

	//wether to use the multi-thread algorithm
	bool m_bMultiThread;

};


#endif