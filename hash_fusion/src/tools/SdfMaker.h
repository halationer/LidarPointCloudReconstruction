#ifndef SDF_MAKER_H
#define SDF_MAKER_H

#include <embree4/rtcore.h>
#include <cmath>
#include <limits>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <unordered_map>

#include "DebugManager.h"
#include "BoundingBox.h"
#include "volume/DistanceIoVolume.h"

class SdfMaker {

public:
	SdfMaker();
	~SdfMaker();

	// end-to-end forward function
	std::vector<float> MakeSdf(
		const pcl::PointXYZ & oViewPoint, 
		const pcl::PointCloud<pcl::PointXYZ> & vQueryPoints, 
		const pcl::PointCloud<pcl::PointXYZI>& vClouds, 
		const std::vector<pcl::Vertices>& vMeshVertices);
	
	void NewTimeRecord();
	void CoutDetailedTime();

	// sprite function, first make scene
	void NewScene(const pcl::PointCloud<pcl::PointXYZI> & vClouds, const std::vector<pcl::Vertices> & vMeshVertices, const int iSectorId);
	// sprite function, second query
	void QuerySdf(const pcl::PointXYZ & oViewPoint, pcl::PointCloud<pcl::DistanceIoVoxel> & vQueryPoints, const int iSectorId);

	tools::BoundingBox GetBoundingBox(const int iSectorId);

private:
    struct Vertex { float x, y, z; };
    struct Triangle { int v0, v1, v2; };

	RTCDevice InitializeDevice(const char* config);
	void SetDefaultIntersectMode();
	RTCScene PushSingleMeshToScene(const pcl::PointCloud<pcl::PointXYZI> & vClouds, const std::vector<pcl::Vertices> & vMeshVertices);
	std::vector<float> CastRay(RTCScene & scene, const pcl::PointXYZ& oViewPoint, const pcl::PointCloud<pcl::PointXYZ>& vQueryPoints);

	// record the result into the query points' io/distance attribute
	void CastRay(RTCScene & scene, const pcl::PointXYZ& oViewPoint, pcl::PointCloud<pcl::DistanceIoVoxel>& vQueryPoints);
    
    static void ErrorCallback(void* userPtr, enum RTCError error, const char* str);

    RTCDevice m_pDevice;
    RTCIntersectArguments m_oIntersectArgument;
	TimeDebugger m_oTimeDebugger;
	std::unordered_map<int, RTCScene> m_vpScene;
};

#endif