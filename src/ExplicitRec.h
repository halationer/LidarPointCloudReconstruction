#ifndef EXPLICITREC_H
#define EXPLICITREC_H
#include "GHPR.h"
#include "Cell.h"
#include "SectorPartition.h"
#include "MeshOperation.h"
#include "MeshSample.h"
#include "Polar.h"
#include "readtxt.h"

class ExplicitRecParam{

public:

	ExplicitRecParam() :m_GHPRParam(3.6){


	};

	~ExplicitRecParam(){
	};

	float m_GHPRParam;
};


class ExplicitRec :public ExplicitRecParam{

public:

	ExplicitRec();
	~ExplicitRec();

	//set the viewpoint location
	void SetViewPoint(const pcl::PointXYZ & oViewPoint);

	//Set the horizontal angle interval size
	void HorizontalSectorSize(int iSectNum);
	//reload
	void HorizontalSectorSize(float fHorizontalRes);

	//Set
	void FrameReconstruction(const pcl::PointCloud<pcl::PointXYZ> & vSceneCloud);

	void ClearData();

	//***data***
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> m_vAllSectorClouds;

	std::vector<std::vector<pcl::Vertices>> m_vAllSectorFaces;


private:


	int m_iSectNum;

	float m_fHorizontalRes;

	pcl::PointXYZ m_oViewPoint;

};


#endif