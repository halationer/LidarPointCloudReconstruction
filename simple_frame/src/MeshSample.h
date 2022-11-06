#ifndef MESHSAMPLE_H
#define MESHSAMPLE_H

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/transforms.h>
#include <vtkVersion.h>
#include <vtkPLYReader.h>
#include <vtkOBJReader.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include "Confidence.h"
using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

class MeshSample{

public:

	//Initialize necessary parameter with default values
	MeshSample();

	~MeshSample();

	//set the sampled point number
	void SetSamplePointNum(int iSampNum);

	//set whether compute the normal for each sampled point
	void SetOutNormals(bool bOutNormals);

	//set the voxel length
	void SetLeafSize(float fLeafSize);

	//compute the random seed for sampled point
	inline static double uniform_deviate(int seed);

	//compute the random point from a triangle
	static void randomPointTriangle(float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3, Eigen::Vector4f& p);

	//compute the point from surface
	inline void randPSurface(vtkPolyData * polydata, std::vector<double> * cumulativeAreas, double totalArea, Eigen::Vector4f& p, bool calcNormal, Eigen::Vector3f& n);

	//uniform sampling points by considering the area factor
	void uniform_sampling(vtkSmartPointer<vtkPolyData> polydata, size_t n_samples, bool calc_normal, pcl::PointCloud<pcl::PointNormal> & cloud_out);

	//sampling by a pcl input
	void SampleMeshToPoints(const pcl::PolygonMesh & oMesh);

	//outout the sampled point clouds without normal
	//note that sampled point clouds with normal can be be accessed directly
	void OutputSampledClouds(pcl::PointCloud<pcl::PointXYZ> & vCloud);

	//Sampled point clouds are voxelized with normal output
	void OutputVoxelizedClouds(pcl::PointCloud<pcl::PointNormal> & vCloud);

	//Sampled point clouds are voxelized without normal output
	void OutputVoxelizedClouds(pcl::PointCloud<pcl::PointXYZ> & vVoxelCloud);

    //*************** Point Cloud Completion*****************
    static void GetAdditionalPointCloud(const std::vector<std::vector<pcl::PointXYZI>>& in_cloud, std::vector<std::vector<pcl::PointXYZI>>& out_cloud);
    static void GetAdditionalPointCloud(const pcl::PointCloud<pcl::PointXYZI>& in_face_center, const std::vector<float>& in_face_weight, const Eigen::MatrixXf& in_face_normal, 
        pcl::PointCloud<pcl::PointNormal>& out_cloud, pcl::PointCloud<pcl::PointXYZI>& out_display_cloud);

	template<class T>
    static void GetAdditionalPointCloud(const pcl::PointCloud<T>& in_cloud, const std::vector<pcl::Vertices>& in_polygons, const std::vector<float>& in_face_weight, 
        const Eigen::MatrixXf& in_face_normal, pcl::PointCloud<pcl::PointNormal>& out_cloud, pcl::PointCloud<pcl::PointXYZI>& out_display_cloud);

	template<class T>
    static void GetAdditionalPointCloud(const pcl::PointCloud<T>& in_cloud, const std::vector<pcl::Vertices>& in_polygons, const std::vector<Confidence>& in_face_weight, 
        const Eigen::MatrixXf& in_face_normal, pcl::PointCloud<pcl::PointNormal>& out_cloud, pcl::PointCloud<pcl::PointXYZI>& out_display_cloud);

	//data
	//the sampled point clouds from mesh
	pcl::PointCloud<pcl::PointNormal>::Ptr m_pSampedCloud;

private:

	//the given sampled point number
	int m_iSamPointNum;

	//whether output normal of sampled point
	bool m_bOutNormals;

	//the length of voxelization
	float m_fLeafSize;


};


#endif

//=================how to use it=======================================
/*

pcl::PointCloud<pcl::PointXYZ>::Ptr pZeroCrossCloud(new pcl::PointCloud<pcl::PointXYZ>);
MeshSample oMeshSampler;
oMeshSampler.SetSamplePointNum(100000);
oMeshSampler.SampleMeshToPoints(MeshModel);
oMeshSampler.OutputSampledClouds(*pZeroCrossCloud);

*/