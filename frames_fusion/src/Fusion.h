#ifndef FUSION_H
#define FUSION_H

#include<cmath>
#include<vector>
#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/kdtree/kdtree.h>



class Fusion{

public:

	Fusion();
	~Fusion();

	//set node weight for kinect fusion mode
	void SetKinectMode(const int & iNodeNum, float fRawWeight = 1.0f);

	//set map size 
	void SetAccDisSize(const int & iNodeNum, float fRawDis = 0.0f);

	//fusion using kinect fusion mode - Weighted average
	//if vWeights and fRawWeight is the constant 1, respectively
	//then it is a mean least squares
	void KinectFusion(const std::vector<float> & vCurrentDis, std::vector<float> & vWeights);

	//
	void ConvexBasedFusion(const std::vector<float> & vCurrentDis, std::vector<float> & vDisMap);

	//
	void UnionMinimalFusion(const std::vector<float> & vCurrentDis);

	//fuses normal vector
	pcl::PointNormal NormalFusion(const std::vector<int> & vPointIdx, const pcl::PointCloud<pcl::PointNormal> & vCloudNormal);
	pcl::PointNormal NormalFusionWeighted(const std::vector<int> & vPointIdx, pcl::PointCloud<pcl::PointNormal> & vCloudNormal);

	//*******data********
	std::vector<float> m_vAccDis;

private:

	std::vector<float> m_vKinectWeight;

};



#endif