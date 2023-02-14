#ifndef SIGNEDDISTANCE_H
#define SIGNEDDISTANCE_H

#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include "GHPR.h"
#include "HashVoxeler.h"
#include "ConvexHullOperation.h"
#include "MeshSample.h"
#include "Polar.h"
#include "Fusion.h"
#include "readtxt.h"

class SignedDisParam{

public:

	SignedDisParam() :m_GHPRParam(3.6) { };

	~SignedDisParam() { };

	float m_GHPRParam;
};

class SignedDistance : public SignedDisParam{

public:

	std::unordered_map<HashPos, float, HashFunc> m_vSignedDistance;
	
	HashVoxeler::HashVolume m_vVolumeCopy;

	SignedDistance() { };

	~SignedDistance() { };

	//compute the signed distance based on surfel(point with normal)
	std::unordered_map<HashPos, float, HashFunc> & NormalBasedGlance(pcl::PointCloud<pcl::PointNormal>::Ptr & pCloudNormals, HashVoxeler & oVoxeler);

	//compute the signed distance based on corners to plan in a voxel
	std::unordered_map<HashPos, float, HashFunc> & PlanDistance(const HashVoxeler::HashVolume & vVolume, const std::unordered_map<HashPos, FacePara, HashFunc> & vNormalPara, const pcl::PointXYZ oVoxelSize);

	//=====data=====
	//
	std::vector<pcl::Vertices> m_vSurfaceIdxs;

	//
	std::vector<pcl::Vertices> m_vGlanceFaceIdxs;

};

#endif