#ifndef GHPR_H
#define GHPR_H
#include<pcl/point_types.h>
#include<pcl/surface/convex_hull.h>
#include<pcl/surface/concave_hull.h>//important
#include<pcl/io/pcd_io.h>
#include<pcl/kdtree/kdtree.h>


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
	GHPR(pcl::PointXYZ f_viewpoint, double f_param = 2);

	//set param
	void SetParam(double f_param);

	//set the viewpoint
	void Inputviewpoint(const pcl::PointXYZ & f_viewpoint);

	//get of maximum value
	float GetMaxValue(std::vector<float> & vVec);

	//calculate of vector norm 
	float NormVector(const pcl::PointXYZ & oPoint);

	//Convert point cloud
	void ConvertCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pCloud);
	//reload - Based on viewpoint, it produces geometric transformation for any given point set, but does not add viewpoint
	void ConvertCloud(const pcl::PointCloud<pcl::PointXYZ> & vSCloud, pcl::PointCloud<pcl::PointXYZ> & vTCloud);

	//build the convex hull to the index of the input
	void IndexFromHulltoInput(pcl::PointCloud<pcl::PointXYZ>::Ptr & pHullVertices);

	//compute visibility
	void Compute(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pCloud, bool bIndexRelation = true);

	//output occluded points index in point clouds
	std::vector<int> GetOccludedIdx();

	//output the indices of vertices on the convex hull correspond to input points with viewpoint
	//
	std::vector<pcl::Vertices> GetConvexHullWorldIdx();

	//output the indices of vertices on the reconstructed surface correspond to input points without viewpoint
	std::vector<pcl::Vertices> ConstructSurfaceIdx();

	//**data
	//transformed point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_pTransCloud;

	//polygon of convex hull
	std::vector<pcl::Vertices> m_vHullPolygonIdxs;

	//vertices / points of convex hull
	//Unable to ensure that the point order of the convex hull points and the original point cloud is the same after PCL function
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_pHullVertices;

	//view point in world coordinate
	pcl::PointXYZ m_oViewPInWorld;

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

//******************************one example*************************************
//#include"GHPR.h"
//#include"HpdPointCloudDisplay.h"
//
//
//int main(){
//
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);//原始pcd点云
//	pcl::PointCloud<pcl::PointXYZ>::Ptr occloud (new pcl::PointCloud<pcl::PointXYZ>);
//	std::vector<Point3D> point3d;//Point3D目标的特征计算点云
//	//读取点云数据
//	HPDpointclouddataread("bunny.las",cloud,point3d,1);
//	std::vector<Point3D> oripoint3d(point3d);
//	//设置视点;
//	pcl::PointXYZ oViewPoint;
//	oViewPoint.x = 0.203490;
//	oViewPoint.y = 0.62239;
//	oViewPoint.z = 0.535947;
//	//初始化HPR类
//	GHPR hpdhpr(oViewPoint, 3.6);
//	//设置遮挡索引
//	std::vector<int> occindices;
//	hpdhpr.Compute(cloud);
//	occindices = hpdhpr.GetOccludedIdx();
//	std::vector<pcl::Vertices> vFaces;
//	vFaces = hpdhpr.GetModelingIdx();
//
//	//输出
//	for(size_t i=0;i!=oripoint3d.size();i++)
//		oripoint3d[i].classification=0;
//	for(size_t i=0;i!=occindices.size();i++){
//		//occloud->push_back(cloud->points[occindices[i]]);
//		oripoint3d[occindices[i]].classification=1;
//	}
//	//窗口开启
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
//    HpdDisplay hpdisplay;
//	//viewer=hpdisplay.Showsimplecolor(occloud,"grey");
//	viewer=hpdisplay.Showclassification(oripoint3d,"assign");
//	viewer->addSphere(oViewPoint, 0.002, 0.0, 0.0, 1.0, "viewpointer");
//	//cloud->points.push_back(oViewPoint);
//	viewer->addPolygonMesh<pcl::PointXYZ>(cloud, vFaces, "polyline");
//
//
//	while (!viewer->wasStopped()){
//
//		viewer->spinOnce ();
//	
//	}
//	
//	return 0;
//
//}
