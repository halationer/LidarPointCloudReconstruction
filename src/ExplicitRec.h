#ifndef EXPLICITREC_H
#define EXPLICITREC_H
#include "GHPR.h"
#include "SectorPartition.h"
#include "MeshOperation.h"
#include "MeshSample.h"


class ExplicitRecParam{

public:

	ExplicitRecParam() :m_GHPRParam(3.6), m_fPseudoFaceThr(0.05){


	};

	~ExplicitRecParam(){
	};

	float m_GHPRParam;

	//scan angle threshold for removing pseudo planes
	float m_fPseudoFaceThr;

};


class ExplicitRec :public ExplicitRecParam{

public:

	ExplicitRec();
	~ExplicitRec();

	//set the viewpoint location
	void SetViewPoint(const pcl::PointXYZ & oViewPoint);

	//reload, set the viewpoint location and its actual elevation 
	//the height of the lidar from the ground, the lidar is usually installed on the vehicle with constant height
	void SetViewPoint(const pcl::PointXYZ & oViewPoint, float fViewElevation);

	//set the horizontal angle interval size with the given number of divisions
	void HorizontalSectorSize(int iSectNum);
	//reload, set the horizontal angle interval size with the given horizontal angle size
	void HorizontalSectorSize(float fHorizontalRes);

	//Remove pseudo planes from reconstruction
	void RemovePseudoFaces(const pcl::PointCloud<pcl::PointXYZ> & vCenterPoints, const std::vector<pcl::Vertices> & vOneFaces, const Eigen::MatrixXf & oMatNormal,
		                   std::vector<bool> & vTrueFaceStatus, std::vector<float> & vFaceWeight);

	//reconstruction of one frame scanning point cloud and ouput mesh normal
	void FrameReconstruction(const pcl::PointCloud<pcl::PointXYZ> & vSceneCloud, pcl::PointCloud<pcl::PointNormal> & vScenePNormal);

	//count the number of points
	void CountNumber(int & iVerticesNum, int & iFacesNum);

	//remove one frame data
	void ClearData();

	//***data***
	//each sector vertices
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> m_vAllSectorClouds;

	//each sector face (vertice relationships)
	std::vector<std::vector<pcl::Vertices>> m_vAllSectorFaces;

	//center point of face with normal
	pcl::PointCloud<pcl::PointNormal>::Ptr m_pCenterNormal;


private:

	//number of sectors
	int m_iSectNum;

	//horizontal angle resolution for sector division
	float m_fHorizontalRes;

	//the elevation value (height) of the viewpoint in global coordinates
	float m_fViewElevation;

	//the location of viewpoint in global coordinates
	pcl::PointXYZ m_oViewPoint;

	//the elevation value of the viewpoint has been obtained
	bool m_bElevationFlag;



};


#endif