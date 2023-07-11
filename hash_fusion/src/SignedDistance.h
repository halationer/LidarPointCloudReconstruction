#ifndef SIGNEDDISTANCE_H
#define SIGNEDDISTANCE_H

#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include "volume/VolumeBase.h"
#include "ConvexHullOperation.h"
#include "MeshSample.h"
#include "Polar.h"
#include "Fusion.h"
#include "readtxt.h"


class SignedDistance {

public:

	std::unordered_map<HashPos, float, HashFunc> m_vSignedDistance;
	
	VolumeBase::HashVolume m_vVolumeCopy;

	SignedDistance(int iKeepTime = 50, int iConvDim = 3, int iConvAddPointNumRef = 5, float fConvFusionDistanceRef1 = 0.95f);

	~SignedDistance() { };

	// build union set according to 
	void BuildUnionSet(VolumeBase & oVoxeler, UnionSet& oUnionSet);

	//compute the signed distance based on surfel(point with normal)
	std::unordered_map<HashPos, float, HashFunc> & DebugGlance(VolumeBase & oVoxeler);

	std::unordered_map<HashPos, float, HashFunc> & NormalBasedGlance(VolumeBase & oVoxeler);
	std::unordered_map<HashPos, float, HashFunc> & ConvedGlance(VolumeBase & oVoxeler, visualization_msgs::MarkerArray* oDebugMarker = nullptr);
	std::unordered_map<HashPos, float, HashFunc> & ConvedGlanceNoneFlow(VolumeBase & oVoxeler, visualization_msgs::MarkerArray* oDebugMarker = nullptr);
	std::unordered_map<HashPos, float, HashFunc> & ConvedGlanceLargeUnion(VolumeBase & oVoxeler, const int iRemoveSizeRef, visualization_msgs::MarkerArray* oDebugMarker = nullptr);
	std::unordered_map<HashPos, float, HashFunc> & ConvedGlanceOnlyMaxUnion(VolumeBase & oVoxeler, visualization_msgs::MarkerArray* oDebugMarker = nullptr);
	std::unordered_map<HashPos, float, HashFunc> & CenterBasedGlance(VolumeBase & oVoxeler, const Eigen::Vector3f vCenter, const float fRadius, const int iRemoveSizeRef, visualization_msgs::MarkerArray* oDebugMarker = nullptr);

	// output glance
	std::unordered_map<HashPos, float, HashFunc> & ConvedGlanceAll(VolumeBase & oVoxeler);
	std::unordered_map<HashPos, float, HashFunc> & ConvedGlanceAllUnion(VolumeBase & oVoxeler, const int iRemoveSizeRef);
	
	//compute the signed distance based on corners to plan in a voxel
	std::unordered_map<HashPos, float, HashFunc> & PlanDistance(const VolumeBase::HashVolume & vVolume, const std::unordered_map<HashPos, FacePara, HashFunc> & vNormalPara, const Eigen::Vector3f & oVoxelSize);

	//=====data=====
	//
	std::vector<pcl::Vertices> m_vSurfaceIdxs;

	//
	std::vector<pcl::Vertices> m_vGlanceFaceIdxs;

protected:
	void CopyAndExpandVolume(VolumeBase::HashVolume& vVolume);
	std::unordered_map<HashPos, float, HashFunc> & ConvedGlanceCore(VolumeBase::HashVolume& vTempVolumeCopy, const Eigen::Vector3f& oVoxelSize);

	int m_iKeepTime;
	int m_iConvDim;
	int m_iConvHalfDim;
	int m_iConvAddPointNumRef;
	float m_fConvFusionDistanceRef1;

};

#endif