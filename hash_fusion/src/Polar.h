#ifndef POLAR_H
#define POLAR_H

#include<iostream>
#include<math.h>
#include<pcl/point_types.h>
#include<pcl/io/pcd_io.h>
#include"readtxt.h"
#define PI 3.1415926

struct Polar{

	float r;
	float phi;
	float theta;

};

class CoordinateTrans{

public:
	
	//convert an Euclidean coordinate point to a polar coordinate point
	Polar EuclidToPolar(const float & x, const float & y, const float & z);

	//convert a polar coordinate point to an Euclidean coordinate point
	pcl::PointXYZ PolarToEuclid(const Polar & oPoint);

	//convert Euclidean coordinate point cloud to polar coordinate point cloud
	void EuclidToPolarCloud(const pcl::PointCloud<pcl::PointXYZ> & vCloud, std::vector<Polar> & oPolarCloud);
	//relaod with the outputing the angle range of the points
	void EuclidToPolarCloud(const pcl::PointCloud<pcl::PointXYZ> & vCloud, const pcl::PointXYZ & oViewPoint, 
		                     pcl::PointCloud<pcl::PointXYZ> & vLocalCloud, std::vector<Polar> & vPolarCloud,
		                              float & fMinPhi, float & fMaxPhi, float & fMinTheta, float & fMaxTheta);

	//convert polar coordinate point cloud to Euclidean coordinate point cloud
	void PolarToEuclidCloud(const std::vector<Polar> & oPolarCloud, pcl::PointCloud<pcl::PointXYZ> & vCloud);

};

class Sector{

public:

	Sector(float fDefaultDis = 1.0);
	~Sector();

	//set phi range
	void SetPhiRange(float fMinPhi, float fMaxPhi);

	//set theta range
	void SetThetaRange(float fMinTheta, float fMaxTheta);

	//set the resolution
	void SetResolution(float fPhiRes, float fThetaRes);

	//set the viewpoint position (location)
	void SetViewPoint(const pcl::PointXYZ & oViewPoint);

	//compute distance between two points
	float ComputeDistance(const pcl::PointXYZ & oPA, const pcl::PointXYZ & oPB);
	//reload, oPB = (0,0,0)
	float ComputeDistance(const pcl::PointXYZ & oPA);

	//compute the grid index of query point
	int PointToGrid(const Polar & oPolarPoint);

	//reload, compute the grid index of query point
	void PointToGrid(const Polar & oPolarPoint, int & iGirdIdx);

	//reload,compute the grid index of query point with three outputs: iPhiIdx, iThetaIdx, iGridIdx
	int PointToGrid(const Polar & oPolarPoint, int & iPhiIdx, int & iThetaIdx);

	//compute distance from viewpoint to each grid points
	void GridToViewPDistance(const pcl::PointCloud<pcl::PointXYZ> & vCloud, int iType = 1);

	//assign points to sector
	void AssignPoints(const pcl::PointCloud<pcl::PointXYZ> & vCloud);

	//compute a simple voxel based ray cast
	std::vector<float> Raycast(const pcl::PointCloud<pcl::PointXYZ> & vCloud, const pcl::PointCloud<pcl::PointXYZ> & vNodes);

	//some test functions
	//get the point clouds with grid id
	void CloudGridIdx(std::vector<int> & vCloudGridLabels);
	
	//get the point clouds with distance from its grid to viewpoint
	//note that m_vGridToViewDis presents grid not point
	void CloudGridDis(std::vector<float> & vCloudGridDis);
	
	//grid point clouds
	//void Get2DGridCloud(std::vector<int> & vLabels, const int & iPhiIdx, const int & iThetaIdx, const int & iSize);


	//*********data************
	//point id in each grid
	std::vector<std::vector<int>> m_vGirdCloudIndex;

	//std::vector<std::vector<std::vector<int>>> m_vGirdCloudIndex2D;

	//pcl::PointCloud<pcl::PointXYZ>::Ptr m_pSectorCloud;

	//distance from each grid to viewpoint
	//if a grid is empty, it would has a infinite number (FLT__MAX)
	std::vector<float> m_vGridToViewDis;

	std::vector<int> m_vNodeGridIdx;

private:

	pcl::PointXYZ m_oViewPoint;

	float m_fMinPhi;
	float m_fMaxPhi;
	float m_fMinTheta;
	float m_fMaxTheta;

	float m_fPhiGridNum;
	float m_fThetaGridNum;

	//Angular resolution
	float m_fPhiRes;
	float m_fThetaRes;

	//distance default
	float m_fDefaultDis;

	//each point corres
	std::vector<int> m_vCloudGridIdx;

};




#endif