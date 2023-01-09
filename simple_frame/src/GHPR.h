#ifndef GHPR_H
#define GHPR_H
#include<pcl/point_types.h>
#include<pcl/surface/convex_hull.h>
#include<pcl/surface/concave_hull.h>
#include<pcl/io/pcd_io.h>
#include<pcl/kdtree/kdtree.h>

#include<mutex>

//GHPR algorithm
//GHPR - General hidden point removal
//input: f_viewpoint - viewpoint, a 3d point
//           f_param - paramter of kernel function
//            pCloud - a point clouds to be processed
//output: m_vHullPolygonIdxs - convex hull vertices
//           m_pHullVertices - convex hull vertice relationship
//          GetOccludedIdx() - get occluded point index
//   GetConvexHullWorldIdx() - get surface's vertice relationship without viewpoint
//Functin: Reconstruct the potential surface of the point cloud at one view

class GHPR{

public:

	//**function
	GHPR(pcl::PointXYZI f_viewpoint, double f_param = 2);

	//set param
	void SetParam(double f_param);

	//set the viewpoint
	void Inputviewpoint(const pcl::PointXYZI & f_viewpoint);

	//get of maximum value
	float GetMaxValue(std::vector<float> & vVec);

	//calculate of vector norm 
	float NormVector(const pcl::PointXYZI & oPoint);

	//Convert point cloud
	void ConvertCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr & pCloud);
	//reload - Based on viewpoint, it produces geometric transformation for any given point set, but does not add viewpoint
	void ConvertCloud(const pcl::PointCloud<pcl::PointXYZI> & vSCloud, pcl::PointCloud<pcl::PointXYZI> & vTCloud);

	//build the convex hull to the index of the input
	void IndexFromHulltoInput(pcl::PointCloud<pcl::PointXYZI>::Ptr & pHullVertices);

	//compute visibility
	void Compute(const pcl::PointCloud<pcl::PointXYZI>::Ptr & pCloud, bool bIndexRelation = true);
	void ComputeMultiThread(const pcl::PointCloud<pcl::PointXYZI>::Ptr & pCloud, bool bIndexRelation = true);

	//output occluded points index in point clouds
	std::vector<int> GetOccludedIdx();

	//output the indices of vertices on the convex hull correspond to input points with viewpoint
	//
	std::vector<pcl::Vertices> GetConvexHullWorldIdx();

	//output the indices of vertices on the reconstructed surface correspond to input points without viewpoint
	std::vector<pcl::Vertices> ConstructSurfaceIdx();
	std::vector<pcl::Vertices> ConstructSurfaceIdxFiltered(const int line_min = 0, const int line_max = 15, const bool debug = false);

	//**data
	//transformed point cloud
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_pTransCloud;

	//polygon of convex hull
	std::vector<pcl::Vertices> m_vHullPolygonIdxs;

	//vertices / points of convex hull
	//Unable to ensure that the point order of the convex hull points and the original point cloud is the same after PCL function
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_pHullVertices;

	//view point in world coordinate
	pcl::PointXYZI m_oViewPInWorld;

	//viewpoint idx
	int m_iViewWorldIdx;

	//Hull To Local idx
	std::vector<int> m_vHullInInputIdx;

private:
	
	//param
	double param;

	//a flag indicating that index from the convex hull to the input point set has been established
	bool m_bToWorldIndex;

	//the radius of the spherical mirror transpose transform
	float m_fRadius;

	//Whether the radius has been calculated.
	bool m_bComputeRadius;//true - computed

};


#endif


