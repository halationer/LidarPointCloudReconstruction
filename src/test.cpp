//
///*
//test our GHPR
//*/
//#include <pcl/point_types.h>//点的类型的头文件
//#include <pcl/io/pcd_io.h>//点云文件IO（pcd文件和ply文件）
//#include <pcl/io/ply_io.h>
//#include <pcl/kdtree/kdtree_flann.h>//kd树 快速搜索
//#include <pcl/features/normal_3d.h>//　法线特征
//#include <pcl/surface/gp3.h>// 贪婪投影三角化算法
//#include <pcl/visualization/pcl_visualizer.h>//可视化
//#include <boost/thread/thread.hpp>//多线程
//#include <fstream>// std
//#include <iostream>
//#include <stdio.h>
//#include <string.h>
//#include <string>
//#include "GHPR.h"
//#include "HpdPointCloudDisplay.h"
//#include "MarchingCubesGHPR.hpp"
//#include "ConvexHullOperation.h"
//#include "readtxt.h"
//
//int main(int argc, char** argv)
//{
//	//	// Read in the cloud data
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	//
//	std::vector<Point3D> point3d;//Point3D目标的特征计算点云
//	HPDpointclouddataread("bunny.las", cloud, point3d, 1);
//
//	// 估计法向量 x,y,x + 法向量　+　曲率
//	//pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
//	//pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//	//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//	//tree->setInputCloud(cloud);
//	//n.setInputCloud(cloud);
//	//n.setSearchMethod(tree);
//	//n.setKSearch(10);//20个邻居
//	//n.compute(*normals); //计算法线，结果存储在normals中
//	//std::cout << "normals Fields: " << pcl::getFieldsList(*normals) << std::endl;
//	//* normals 不能同时包含点的法向量和表面的曲率
//
//	//将点云和法线放到一起
//	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
//	//pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
//	//* cloud_with_normals = cloud + normals
//
//
//	//创建搜索树
//	//pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
//	//tree2->setInputCloud(cloud_with_normals);
//
//	//初始化 移动立方体算法 MarchingCubes对象，并设置参数
//	MarchingCubesGHPR<pcl::PointXYZ> *mc;
//	mc = new MarchingCubesGHPR<pcl::PointXYZ>();
//
//	//创建多变形网格，用于存储结果
//	pcl::PolygonMesh mesh;
//
//	//设置MarchingCubes对象的参数
//	mc->setIsoLevel(0.0f);
//	mc->setGridResolution(50, 50, 50);
//	mc->setPercentageExtendGrid(0.0f);
//	
//
//	//设置搜索方法
//	mc->setInputCloud(cloud);
//	//mc->setInputCloud(cloud_with_normals);
//    mc->UpdataCornerValues();
//	//执行重构，结果保存在mesh中
//	//mc->reconstruct(mesh);
//
//	
//
//	//保存网格图
//	//pcl::io::savePLYFile("result_mesh.ply", mesh);
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr pshowcloud(new pcl::PointCloud<pcl::PointXYZ>);
//
//	for (int i = 0; i != mc->m_vCornerCloud.points.size(); ++i){
//	
//		pshowcloud->points.push_back(mc->m_vCornerCloud.points[i]);
//	}
//
//	//WritePointCLoudTxt("result_corner.txt", mc->m_vCornerCloud, mc->m_vCornerValue);
//
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
//	HpdDisplay hpdisplay;
//	//viewer = hpdisplay.Showfeatureresult(pshowcloud, mc->m_vCornerValue, "redgreen");
//	//cloud->points.push_back(oViewPoint);
//	//viewer->addPolygonMesh(mesh, "polyline");
//
//
//	while (!viewer->wasStopped()){
//
//		viewer->spinOnce();
//
//	}
//
//	return (0);
//}

////
/////*
////三维重构之移动立方体算法
////*/
////#include <pcl/point_types.h>//点的类型的头文件
////#include <pcl/io/pcd_io.h>//点云文件IO（pcd文件和ply文件）
////#include <pcl/io/ply_io.h>
////#include <pcl/kdtree/kdtree_flann.h>//kd树 快速搜索
////#include <pcl/features/normal_3d.h>//　法线特征
////#include <pcl/surface/marching_cubes_hoppe.h>// 移动立方体算法
////#include <pcl/surface/marching_cubes_rbf.h>
////#include <pcl/surface/gp3.h>// 贪婪投影三角化算法
////#include <pcl/visualization/pcl_visualizer.h>//可视化
////#include <boost/thread/thread.hpp>//多线程
////#include <fstream>// std
////#include <iostream>
////#include <stdio.h>
////#include <string.h>
////#include <string>
////#include"GHPR.h"
////#include"HpdPointCloudDisplay.h"
////
////int main(int argc, char** argv)
////{
////	//	// Read in the cloud data
////	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
////	//
////	std::vector<Point3D> point3d;//Point3D目标的特征计算点云
////	HPDpointclouddataread("bunny.las",cloud,point3d,1);
////
////	// 估计法向量 x,y,x + 法向量　+　曲率
////	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
////	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
////	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
////	tree->setInputCloud(cloud);
////	n.setInputCloud(cloud);
////	n.setSearchMethod(tree);
////	n.setKSearch(10);//20个邻居
////	n.compute(*normals); //计算法线，结果存储在normals中
////	std::cout << "normals Fields: " << pcl::getFieldsList(*normals) << std::endl;
////	//* normals 不能同时包含点的法向量和表面的曲率
////
////	//将点云和法线放到一起
////	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
////	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
////	//* cloud_with_normals = cloud + normals
////
////
////	//创建搜索树
////	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
////	tree2->setInputCloud(cloud_with_normals);
////
////	//初始化 移动立方体算法 MarchingCubes对象，并设置参数
////	pcl::MarchingCubes<pcl::PointNormal> *mc;
////	mc = new pcl::MarchingCubesHoppe<pcl::PointNormal>();
////
////	//创建多变形网格，用于存储结果
////	pcl::PolygonMesh mesh;
////
////	//设置MarchingCubes对象的参数
////	mc->setIsoLevel(0.0f);
////	mc->setGridResolution(50, 50, 50);
////	mc->setPercentageExtendGrid(0.0f);
////
////	//设置搜索方法
////	mc->setInputCloud(cloud_with_normals);
////
////	//执行重构，结果保存在mesh中
////	mc->reconstruct(mesh);
////
////	//保存网格图
////	pcl::io::savePLYFile("result2.ply", mesh);
////
////	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
////	HpdDisplay hpdisplay;
////	viewer=hpdisplay.Showsimplecolor(cloud,"grey");
////	//cloud->points.push_back(oViewPoint);
////	viewer->addPolygonMesh(mesh, "polyline");
////
////	while (!viewer->wasStopped()){
////	
////			viewer->spinOnce ();
////		
////	}
////
////	return (0);
////}
//
//
//
////
////*************************************convex hull********************************************************
////
////
////
////#include <pcl/ModelCoefficients.h>
////#include <pcl/point_types.h>
////#include <pcl/io/pcd_io.h>
////#include <pcl/surface/convex_hull.h>
////#include <pcl\visualization\pcl_visualizer.h>
////#include"HPR.h"
////#include"HpdPointCloudDisplay.h"
////using namespace pcl::console;
////
////
////int
////main(int argc, char** argv)
////{
////	// Read in the cloud data
////	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
////
////	std::vector<Point3D> point3d;//Point3D目标的特征计算点云
////	HPDpointclouddataread("bunny.las",cloud,point3d,1);
////
////	std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl; //*
////
////	pcl::ConvexHull<pcl::PointXYZ> hull;
////	hull.setInputCloud(cloud);
////	hull.setDimension(3);
////
////	std::vector<pcl::Vertices> polygons;
////	pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);
////	hull.reconstruct(*surface_hull, polygons);
////
////	// ---------------------- Visualizer -------------------------------------------
////	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
////	viewer->setBackgroundColor(255, 255, 255);
////
////	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(cloud, 255, 255, 0);
////	viewer->addPointCloud(cloud, color_handler, "sample cloud");
////	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "sample cloud");
////
////	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handlerK(surface_hull, 255, 0, 0);
////	viewer->addPointCloud(surface_hull, color_handlerK, "point");
////	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "point");
////
////	//viewer->addPolygon<pcl::PointXYZ>(surface_hull, 0, 0, 255, "polyline");
////	viewer->addPolygonMesh<pcl::PointXYZ>(surface_hull, polygons, "polyline");
////
////	while (!viewer->wasStopped())
////	{
////		viewer->spinOnce(100);
////	}
////
////	return (0);
////}
////
////
////
////
//*****possion********************

/*
三维重构之泊松重构
pcl::Poisson<pcl::PointNormal> pn ;
通过本教程，我们将会学会：
如果通过泊松算法进行三维点云重构。
程序支持两种文件格式：*.pcd和*.ply
程序先读取点云文件，然后计算法向量，
接着使用泊松算法进行重构，最后显示结果。
*/

////点的类型的头文件
//#include <pcl/point_types.h>
////点云文件IO（pcd文件和ply文件）
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
////kd树
//#include <pcl/kdtree/kdtree_flann.h>
////特征提取
//#include <pcl/features/normal_3d_omp.h>//　法线特征
//#include <pcl/features/normal_3d.h>
////重构
////#include <pcl/surface/gp3.h> // 贪婪投影三角化算法
//#include <pcl/surface/poisson.h>// 泊松算法进行重构
////可视化
//#include <pcl/visualization/pcl_visualizer.h>
//
//#include "GHPR.h"
//#include "HpdPointCloudDisplay.h"
//#include "ConvexHullOperation.h"
//#include "readtxt.h"
////多线程
//#include <boost/thread/thread.hpp>
//// std
//#include <fstream>
//#include <iostream>
//#include <stdio.h>
//#include <string.h>
//#include <string>
//
//int main(int argc, char** argv)
//{
//
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//		//
//	std::vector<Point3D> point3d;//Point3D目标的特征计算点云
//	HPDpointclouddataread("bunny.las", cloud, point3d, 1);
//
//	// 计算法向量 x,y,x + 法向量　+　曲率
//	pcl::PointCloud<pcl::PointNormal>::Ptr  cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>); //法向量点云对象指针
//	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//法线估计对象
//	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//存储估计的法线的指针
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//	tree->setInputCloud(cloud);
//	n.setInputCloud(cloud);
//	n.setSearchMethod(tree);
//	n.setKSearch(20);//20个邻居
//	n.compute(*normals);//计算法线，结果存储在normals中
//	//　将点云和法线放到一起
//	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
//	//创建搜索树
//	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
//	tree2->setInputCloud(cloud_with_normals);
//	//创建Poisson对象，并设置参数
//	pcl::Poisson<pcl::PointNormal> pn;
//	pn.setConfidence(false); //是否使用法向量的大小作为置信信息。如果false，所有法向量均归一化。
//	pn.setDegree(2); //设置参数degree[1,5],值越大越精细，耗时越久。
//	pn.setDepth(8);
//	//树的最大深度，求解2^d x 2^d x 2^d立方体元。
//	// 由于八叉树自适应采样密度，指定值仅为最大深度。
//
//	pn.setIsoDivide(8); //用于提取ISO等值面的算法的深度
//	pn.setManifold(false); //是否添加多边形的重心，当多边形三角化时。 
//	// 设置流行标志，如果设置为true，则对多边形进行细分三角话时添加重心，设置false则不添加
//	pn.setOutputPolygons(false); //是否输出多边形网格（而不是三角化移动立方体的结果）
//	pn.setSamplesPerNode(3.0); //设置落入一个八叉树结点中的样本点的最小数量。无噪声，[1.0-5.0],有噪声[15.-20.]平滑
//	pn.setScale(1); //设置用于重构的立方体直径和样本边界立方体直径的比率。
//	pn.setSolverDivide(8); //设置求解线性方程组的Gauss-Seidel迭代方法的深度
//	//pn.setIndices();
//
//	//设置搜索方法和输入点云
//	pn.setSearchMethod(tree2);
//	pn.setInputCloud(cloud_with_normals);
//	//创建多变形网格，用于存储结果
//	pcl::PolygonMesh mesh;
//	//执行重构
//	pn.performReconstruction(mesh);
//
//	//保存网格图
//	pcl::io::savePLYFile("result.ply", mesh);
//
//	// 显示结果图
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer"));
//	viewer->setBackgroundColor(0, 0, 0);//背景　黑色
//	viewer->addPolygonMesh(mesh, "my");//mesh
//	//viewer->addCoordinateSystem (0.10);//坐标系
//	viewer->initCameraParameters();//相机参数
//	while (!viewer->wasStopped()){
//		viewer->spinOnce(100);
//		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//	}
//
//	return 0;
//}

/*
投影
*/


//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/ModelCoefficients.h>
//#include <pcl/filters/project_inliers.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include "readtxt.h"
//#include "Cell.h"
//#include "Fusion.h"
//#include "CIsoSurface.h"
//#include "SignedDistance.h"
//#include "HpdPointCloudDisplay.h"
//
//
//int main(int argc, char** argv)
//{
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//
//	std::vector<Point3D> point3d;
//	//read data
//	HPDpointclouddataread("bunny.las", cloud, point3d, 1);
//	point3d.clear();
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
//
//	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
//	coefficients->values.push_back(0.0);
//	coefficients->values.push_back(0.0);
//	coefficients->values.push_back(1.0);
//	coefficients->values.push_back(0.0);
//
//	pcl::ProjectInliers<pcl::PointXYZ> proj;
//	proj.setInputCloud(cloud);
//	proj.setModelCoefficients(coefficients);
//	proj.setModelType(pcl::SACMODEL_PLANE);
//	proj.filter(*cloud_projected);
//
//	pcl::visualization::PCLVisualizer my_viewer1;
//	pcl::visualization::PCLVisualizer my_viewer2;
//	my_viewer1.addPointCloud(cloud);
//	my_viewer2.addPointCloud(cloud_projected);
//
//	while (!my_viewer1.wasStopped())
//	{
//		my_viewer1.spinOnce(100);
//		my_viewer2.spinOnce(100);
//
//	}
//	system("pause");
//	return (0);
//}

/*
抽样
*/

//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/io/vtk_lib_io.h>
//#include <pcl/common/transforms.h>
//#include <vtkVersion.h>
//#include <vtkPLYReader.h>
//#include <vtkOBJReader.h>
//#include <vtkTriangle.h>
//#include <vtkTriangleFilter.h>
//#include <vtkPolyDataMapper.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/console/print.h>
//#include <pcl/console/parse.h>
//#include <boost/filesystem/operations.hpp>
//#include <boost/filesystem/path.hpp>
////#include <string>     // std::string, std::to_string
//
//inline double uniform_deviate(int seed)
//{
//	double ran = seed * (1.0 / (RAND_MAX + 1.0));
//	return ran;
//}
//
//inline void randomPointTriangle(float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3, Eigen::Vector4f& p)
//{
//	float r1 = static_cast<float> (uniform_deviate(rand()));
//	float r2 = static_cast<float> (uniform_deviate(rand()));
//	float r1sqr = std::sqrt(r1);
//	float OneMinR1Sqr = (1 - r1sqr);
//	float OneMinR2 = (1 - r2);
//	a1 *= OneMinR1Sqr;
//	a2 *= OneMinR1Sqr;
//	a3 *= OneMinR1Sqr;
//	b1 *= OneMinR2;
//	b2 *= OneMinR2;
//	b3 *= OneMinR2;
//	c1 = r1sqr * (r2 * c1 + b1) + a1;
//	c2 = r1sqr * (r2 * c2 + b2) + a2;
//	c3 = r1sqr * (r2 * c3 + b3) + a3;
//	p[0] = c1;
//	p[1] = c2;
//	p[2] = c3;
//	p[3] = 0;
//}
//
//inline void randPSurface(vtkPolyData * polydata, std::vector<double> * cumulativeAreas, double totalArea, Eigen::Vector4f& p, bool calcNormal, Eigen::Vector3f& n)
//{
//	float r = static_cast<float> (uniform_deviate(rand()) * totalArea);
//
//	std::vector<double>::iterator low = std::lower_bound(cumulativeAreas->begin(), cumulativeAreas->end(), r);
//	vtkIdType el = vtkIdType(low - cumulativeAreas->begin());
//
//	double A[3], B[3], C[3];
//	vtkIdType npts = 0;
//	vtkIdType *ptIds = NULL;
//	polydata->GetCellPoints(el, npts, ptIds);
//	polydata->GetPoint(ptIds[0], A);
//	polydata->GetPoint(ptIds[1], B);
//	polydata->GetPoint(ptIds[2], C);
//	if (calcNormal)
//	{
//		// OBJ: Vertices are stored in a counter-clockwise order by default
//		Eigen::Vector3f v1 = Eigen::Vector3f(A[0], A[1], A[2]) - Eigen::Vector3f(C[0], C[1], C[2]);
//		Eigen::Vector3f v2 = Eigen::Vector3f(B[0], B[1], B[2]) - Eigen::Vector3f(C[0], C[1], C[2]);
//		n = v1.cross(v2);
//		n.normalize();
//	}
//	randomPointTriangle(float(A[0]), float(A[1]), float(A[2]),
//		float(B[0]), float(B[1]), float(B[2]),
//		float(C[0]), float(C[1]), float(C[2]), p);
//}
//
//void uniform_sampling(vtkSmartPointer<vtkPolyData> polydata, size_t n_samples, bool calc_normal, pcl::PointCloud<pcl::PointNormal> & cloud_out)
//{
//	polydata->BuildCells();
//	vtkSmartPointer<vtkCellArray> cells = polydata->GetPolys();
//
//	double p1[3], p2[3], p3[3], totalArea = 0;
//	std::vector<double> cumulativeAreas(cells->GetNumberOfCells(), 0);
//	size_t i = 0;
//	vtkIdType npts = 0, *ptIds = NULL;
//	for (cells->InitTraversal(); cells->GetNextCell(npts, ptIds); i++)
//	{
//		polydata->GetPoint(ptIds[0], p1);
//		polydata->GetPoint(ptIds[1], p2);
//		polydata->GetPoint(ptIds[2], p3);
//		totalArea += vtkTriangle::TriangleArea(p1, p2, p3);
//		cumulativeAreas[i] = totalArea;
//	}
//
//	cloud_out.points.resize(n_samples);
//	cloud_out.width = static_cast<pcl::uint32_t> (n_samples);
//	cloud_out.height = 1;
//
//	for (i = 0; i < n_samples; i++)
//	{
//		Eigen::Vector4f p;
//		Eigen::Vector3f n;
//		randPSurface(polydata, &cumulativeAreas, totalArea, p, calc_normal, n);
//		cloud_out.points[i].x = p[0];
//		cloud_out.points[i].y = p[1];
//		cloud_out.points[i].z = p[2];
//		if (calc_normal)
//		{
//			cloud_out.points[i].normal_x = n[0];
//			cloud_out.points[i].normal_y = n[1];
//			cloud_out.points[i].normal_z = n[2];
//		}
//	}
//}
//
//using namespace std;
//using namespace pcl;
//using namespace pcl::io;
//using namespace pcl::console;
//
///* ---[ */
//int main(int argc, char **argv)
//{
//	
//	//get the vtk type data
//	vtkSmartPointer<vtkPolyData> polydata1 = vtkSmartPointer<vtkPolyData>::New();
//	
//	pcl::PolygonMesh mesh;
//	pcl::io::loadPolygonFilePLY("mesh_res.ply", mesh);
//	pcl::io::mesh2vtk(mesh, polydata1);
//
//
//	//make sure that the polygons are triangles!
//
//	vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
//    #if VTK_MAJOR_VERSION < 6
//		triangleFilter->SetInput(polydata1);
//    #else
//		triangleFilter->SetInputData(polydata1);
//    #endif
//	
//	triangleFilter->Update();
//
//	vtkSmartPointer<vtkPolyDataMapper> triangleMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
//	triangleMapper->SetInputConnection(triangleFilter->GetOutputPort());
//	triangleMapper->Update();
//	polydata1 = triangleMapper->GetInput();
//
//	const int SAMPLE_POINTS_ = 100000;
//	bool write_normals = true;
//	float leaf_size = 0.001f;
//
//	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_1(new pcl::PointCloud<pcl::PointNormal>);
//	uniform_sampling(polydata1, SAMPLE_POINTS_, write_normals, *cloud_1);
//
//	// Voxelgrid
//	VoxelGrid<PointNormal> grid_;
//	grid_.setInputCloud(cloud_1);
//	grid_.setLeafSize(leaf_size, leaf_size, leaf_size);
//
//	pcl::PointCloud<pcl::PointNormal>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointNormal>);
//	grid_.filter(*voxel_cloud);
//
//
//	if (!write_normals){
//
//		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
//		// Strip uninitialized normals from cloud:
//		pcl::copyPointCloud(*voxel_cloud, *cloud_xyz);
//		savePLYFileASCII("hhhh.ply", *cloud_xyz);
//	
//	}else{
//
//		savePLYFileASCII("hhhh.ply", *voxel_cloud);
//
//	}
//
//}

