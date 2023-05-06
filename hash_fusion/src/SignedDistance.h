#ifndef SIGNEDDISTANCE_H
#define SIGNEDDISTANCE_H

#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include "HashVoxeler.h"
#include "ConvexHullOperation.h"
#include "MeshSample.h"
#include "Polar.h"
#include "Fusion.h"
#include "readtxt.h"


class SignedDistance {

public:

	std::unordered_map<HashPos, float, HashFunc> m_vSignedDistance;
	
	HashVoxeler::HashVolume m_vVolumeCopy;

	SignedDistance(int iKeepTime = 50, int iConvDim = 3, int iConvAddPointNumRef = 5, float fConvFusionDistanceRef1 = 0.95f);

	~SignedDistance() { };

	// build union set according to 
	void BuildUnionSet(HashVoxeler & oVoxeler, UnionSet& oUnionSet);

	//compute the signed distance based on surfel(point with normal)
	std::unordered_map<HashPos, float, HashFunc> & NormalBasedGlance(HashVoxeler & oVoxeler);
	std::unordered_map<HashPos, float, HashFunc> & ConvedGlance(HashVoxeler & oVoxeler);
	std::unordered_map<HashPos, float, HashFunc> & ConvedGlanceNoneFlow(HashVoxeler & oVoxeler);
	std::unordered_map<HashPos, float, HashFunc> & ConvedGlanceLargeUnion(HashVoxeler & oVoxeler);
	std::unordered_map<HashPos, float, HashFunc> & ConvedGlanceOnlyMaxUnion(HashVoxeler & oVoxeler);

	//compute the signed distance based on corners to plan in a voxel
	std::unordered_map<HashPos, float, HashFunc> & PlanDistance(const HashVoxeler::HashVolume & vVolume, const std::unordered_map<HashPos, FacePara, HashFunc> & vNormalPara, const pcl::PointXYZ oVoxelSize);

	//=====data=====
	//
	std::vector<pcl::Vertices> m_vSurfaceIdxs;

	//
	std::vector<pcl::Vertices> m_vGlanceFaceIdxs;

protected:
	std::unordered_map<HashPos, float, HashFunc> & ConvedGlanceCore(HashVoxeler::HashVolume& vTempVolumeCopy, const pcl::PointXYZ oVoxelSize);

	int m_iKeepTime;
	int m_iConvDim;
	int m_iConvHalfDim;
	int m_iConvAddPointNumRef;
	float m_fConvFusionDistanceRef1;

};

#endif