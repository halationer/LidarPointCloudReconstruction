//*************for using TSDF as signed distance*********************

#include "readtxt.h"
#include "Cell.h"
#include "Fusion.h"
#include "MeshSample.h"
#include "CIsoSurface.h"
#include "SignedDistance.h"
#include "HpdPointCloudDisplay.h"


int main(){

	//raw point clouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr pFileCloud (new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<Point3D> point3d;
	//read data
	HPDpointclouddataread("bunny.las", pFileCloud, point3d, 1);
	point3d.clear();
	//clear raw data and smapling
	pcl::PointCloud<pcl::PointXYZ>::Ptr pRawCloud(new pcl::PointCloud<pcl::PointXYZ>);
	SamplePoints(*pFileCloud, *pRawCloud, 1);
	pFileCloud->clear();

	//******voxelization********
	Voxelization oVoxeler(*pRawCloud);
	//set the number of voxels
	oVoxeler.GetIntervalNum(60,60,60);
	//voxelize the space
	oVoxeler.VoxelizeSpace();

	//******set fusion model********
	Fusion oFusion;
	//signed distance of each node
	//this map is constantly updated
	oFusion.SetAccDisSize(oVoxeler.m_pCornerCloud->points.size(), oVoxeler.m_fDefault);


	//******viewpoints********
	pcl::PointCloud<pcl::PointXYZ>::Ptr pViewPoints(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZ oViewPoint;
	//x 0.535947 y  0.62239 z 0.535947 bunny
	//x 0.457275 y  0.500000 z 1.814216 Cassette.las
	//x 0.0 -y 0.0 z 0.0 scene1oneframe.las
	oViewPoint.x = 0.535947;
	oViewPoint.y = 0.62239;
	oViewPoint.z = 0.535947;
	pViewPoints->points.push_back(oViewPoint);
	oViewPoint.x = 0.535947;
	oViewPoint.y = -0.62239;
	oViewPoint.z = 0.535947;
	pViewPoints->points.push_back(oViewPoint);
	oViewPoint.x = -0.535947;
	oViewPoint.y = 0.62239;
	oViewPoint.z = -0.535947;
	pViewPoints->points.push_back(oViewPoint);
	oViewPoint.x = 0.335947;
	oViewPoint.y = 0.12239;
	oViewPoint.z = 0.535947;
	pViewPoints->points.push_back(oViewPoint);
	oViewPoint.x = 0.235947;
	oViewPoint.y = 0.92239;
	oViewPoint.z = -0.835947;
	pViewPoints->points.push_back(oViewPoint);
	

	//******compute signed distance********
	for (int i = 1; i != 2; ++i){

		//***compute signed distance of a glance***
		//compute signed distance of a query nodes(voxels)
		SignedDistance oSDer;
		std::vector<float> vCornerSignedDis = oSDer.PointBasedGlance(pRawCloud, pViewPoints->points[i], oVoxeler);
		std::vector<float> vCornerWeigt(vCornerSignedDis.size(),1.0f);

		//***fusion a glance result to global nodes (map)***
		oFusion.UnionMinimalFusion(vCornerSignedDis);
		//oFusion.CorrosionFusion(vCornerSignedDis, vSDMap);
		std::cout << "Finish the " << i << "th viewpoint" << std::endl;
	}


	//******construction********
	//marching cuber
	CIsoSurface<float> oMarchingCuber;
	oMarchingCuber.GenerateSurface(oFusion.m_vAccDis, 0,
		                           oVoxeler.m_iFinalVoxelNum.ixnum - 1, oVoxeler.m_iFinalVoxelNum.iynum - 1, oVoxeler.m_iFinalVoxelNum.iznum - 1, 
		                           oVoxeler.m_oVoxelLength.x, oVoxeler.m_oVoxelLength.y, oVoxeler.m_oVoxelLength.z);


	//******output********
	pcl::PolygonMesh oCBModel;
	oMarchingCuber.OutputMesh(oVoxeler.m_oOriCorner, oCBModel);

	pcl::io::savePLYFileBinary("cb_res.ply", oCBModel);
	std::cout << "the number of final faces after reconstruction: " << oCBModel.polygons.size() << std::endl;
	//*****display part*****
	WritePointCloudTxt("result_corner.txt", *oVoxeler.m_pCornerCloud, oFusion.m_vAccDis);

	pcl::PointCloud<pcl::PointXYZ>::Ptr pInnerClouds(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<float> vInnerValue;
	for (int i = 0; i != oVoxeler.m_pCornerCloud->size(); ++i){
		if (oFusion.m_vAccDis[i]<0.0){
			pInnerClouds->push_back(oVoxeler.m_pCornerCloud->points[i]);
			vInnerValue.push_back(oFusion.m_vAccDis[i]);
		}
	}
	WritePointCloudTxt("inner_corner.txt", *pInnerClouds, vInnerValue);

	pcl::PointCloud<pcl::PointXYZ>::Ptr pOutlierClouds(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<float> vOutlierValue;
	for (int i = 0; i != oVoxeler.m_pCornerCloud->size(); ++i){
		if (oFusion.m_vAccDis[i] >= 0.0){
			pOutlierClouds->push_back(oVoxeler.m_pCornerCloud->points[i]);
			vOutlierValue.push_back(oFusion.m_vAccDis[i]);
		}
	}
	WritePointCloudTxt("Outlier_corner.txt", *pOutlierClouds, vOutlierValue);

	pcl::PointCloud<pcl::PointXYZ>::Ptr pOutDisClouds(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<float> vFinalCornerValue;
	for (int i = 0; i != oVoxeler.m_pCornerCloud->size(); ++i){
		if (oFusion.m_vAccDis[i] != 0.0){
			pOutDisClouds->push_back(oVoxeler.m_pCornerCloud->points[i]);
			vFinalCornerValue.push_back(oFusion.m_vAccDis[i]);
		}
	}
	WritePointCloudTxt("final_corner_value.txt", *pOutDisClouds, vFinalCornerValue);
	
	//pcl::PolygonMesh MeshModel;
	//pcl::toPCLPointCloud2(*pRawCloud,MeshModel.cloud);
	//MeshModel.polygons = oSDer.m_vSurfaceIdxs;
	//pcl::io::savePLYFileBinary("reconstruction_res.ply", MeshModel);

	//******display********
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    HpdDisplay hpdisplay;
	//viewer = hpdisplay.Showclassification(mc->m_vCornerCloud, vInner, "assign");
	viewer = hpdisplay.Showfeatureresult(*oVoxeler.m_pCornerCloud, oFusion.m_vAccDis, "redgreen");
	//viewer = hpdisplay.Showclassification(pVoxelCorners, "assign");
	viewer->addSphere(oViewPoint, 0.02, 0.0, 0.0, 1.0, "viewpointer");	
	//viewer->addPolygonMesh<pcl::PointXYZ>(pRawCloud, oSDer.m_vSurfaceIdxs, "polyline");


	while (!viewer->wasStopped()){

		viewer->spinOnce ();
	
	}
	
	return 0;


}


////*************for using convex hull distance as signed distance*********************
//
//#include "readtxt.h"
//#include "Cell.h"
//#include "Fusion.h"
//#include "CIsoSurface.h"
//#include "SignedDistance.h"
//#include "HpdPointCloudDisplay.h"
//
//
//int main(){
//
//	//raw point clouds
//	pcl::PointCloud<pcl::PointXYZ>::Ptr pFileCloud (new pcl::PointCloud<pcl::PointXYZ>);
//	std::vector<Point3D> point3d;
//	//read data
//	HPDpointclouddataread("bunny.las", pFileCloud, point3d, 1);
//	point3d.clear();
//	//clear raw data and smapling
//	pcl::PointCloud<pcl::PointXYZ>::Ptr pRawCloud(new pcl::PointCloud<pcl::PointXYZ>);
//	SamplePoints(*pFileCloud, *pRawCloud, 1);
//	pFileCloud->clear();
//
//	//******voxelization********
//	Voxelization oVoxeler(*pRawCloud);
//	//set the number of voxels
//	oVoxeler.GetIntervalNum(60,60,60);
//	//voxelize the space
//	oVoxeler.VoxelizeData();
//
//	//signed distance of each node
//	//this map is constantly updated
//	std::vector<float> vSDMap(oVoxeler.m_pCornerCloud->points.size(), -1.0);
//	std::vector<float> vMinDis = SignedDistance::MinKDDis(pRawCloud, oVoxeler.m_pCornerCloud);
//	std::cout << "finished the kd" << std::endl;
//
//	Fusion oFusion;
//	//******viewpoints********
//	pcl::PointCloud<pcl::PointXYZ>::Ptr pViewPoints(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointXYZ oViewPoint;
//	//x 0.535947 y  0.62239 z 0.535947 bunny
//	//x 0.457275 y  0.500000 z 1.814216 Cassette.las
//	//x 0.0 -y 0.0 z 0.0 scene1oneframe.las
//	oViewPoint.x = 0.535947;
//	oViewPoint.y = -0.62239;
//	oViewPoint.z = 0.535947;
//	pViewPoints->points.push_back(oViewPoint);
//	oViewPoint.x = 0.535947;
//	oViewPoint.y = 0.62239;
//	oViewPoint.z = 0.535947;
//	pViewPoints->points.push_back(oViewPoint);
//	oViewPoint.x = -0.535947;
//	oViewPoint.y = 0.62239;
//	oViewPoint.z = -0.535947;
//	pViewPoints->points.push_back(oViewPoint);
//	oViewPoint.x = 0.335947;
//	oViewPoint.y = 0.12239;
//	oViewPoint.z = 0.535947;
//	pViewPoints->points.push_back(oViewPoint);
//	oViewPoint.x = 0.235947;
//	oViewPoint.y = 0.92239;
//	oViewPoint.z = -0.835947;
//	pViewPoints->points.push_back(oViewPoint);
//	
//	for (int i = 0; i != 1; ++i){
//		//compute signed distance of a query nodes(voxels)
//		SignedDistance oSDer;
//		std::vector<float> vPlanDis = oSDer.RaycastBasedGlance(pRawCloud, pViewPoints->points[i], oVoxeler.m_pCornerCloud);
//		std::vector<float> vCornerSignedDis = oSDer.MinKDDSignedDis(vPlanDis, vMinDis);
//		oFusion.CorrosionFusion(vCornerSignedDis, vSDMap);
//		std::cout << "Finish the " << i << "th viewpoint" << std::endl;
//	}
//
//	//marching cuber
//	CIsoSurface<float> oMarchingCuber;
//	oMarchingCuber.GenerateSurface(vSDMap, 0, 
//		                           oVoxeler.m_iFinalVoxelNum.ixnum - 1, oVoxeler.m_iFinalVoxelNum.iynum - 1, oVoxeler.m_iFinalVoxelNum.iznum - 1, 
//		                           oVoxeler.m_oVoxelLength.x, oVoxeler.m_oVoxelLength.y, oVoxeler.m_oVoxelLength.z);
//
//	//
//	pcl::PolygonMesh oCBModel;
//	oMarchingCuber.OutputMesh(oVoxeler.m_oOriCorner, oCBModel);
//
//	pcl::io::savePLYFileBinary("cb_res.ply", oCBModel);
//	std::cout << "the number of final faces after reconstruction: " << oCBModel.polygons.size() << std::endl;
//	//*****display part*****
//	WritePointCloudTxt("result_corner.txt", *oVoxeler.m_pCornerCloud, vSDMap);
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr pInnerClouds(new pcl::PointCloud<pcl::PointXYZ>);
//	std::vector<float> vInnerValue;
//	for (int i = 0; i != oVoxeler.m_pCornerCloud->size(); ++i){
//		if (vSDMap[i]<0.0){
//			pInnerClouds->push_back(oVoxeler.m_pCornerCloud->points[i]);
//			vInnerValue.push_back(vSDMap[i]);
//		}
//	}
//	WritePointCloudTxt("inner_corner.txt", *pInnerClouds, vInnerValue);
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr pOutlierClouds(new pcl::PointCloud<pcl::PointXYZ>);
//	std::vector<float> vOutlierValue;
//	for (int i = 0; i != oVoxeler.m_pCornerCloud->size(); ++i){
//		if (vSDMap[i] >= 0.0){
//			pOutlierClouds->push_back(oVoxeler.m_pCornerCloud->points[i]);
//			vOutlierValue.push_back(vSDMap[i]);
//		}
//	}
//	WritePointCloudTxt("Outlier_corner.txt", *pOutlierClouds, vOutlierValue);
//	
//	//pcl::PolygonMesh MeshModel;
//	//pcl::toPCLPointCloud2(*pRawCloud,MeshModel.cloud);
//	//MeshModel.polygons = oSDer.m_vSurfaceIdxs;
//	//pcl::io::savePLYFileBinary("reconstruction_res.ply", MeshModel);
//
//	//窗口开启
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
//    HpdDisplay hpdisplay;
//	//viewer = hpdisplay.Showclassification(mc->m_vCornerCloud, vInner, "assign");
//	viewer = hpdisplay.Showfeatureresult(*oVoxeler.m_pCornerCloud, vSDMap, "redgreen");
//	//viewer = hpdisplay.Showclassification(pVoxelCorners, "assign");
//	viewer->addSphere(oViewPoint, 0.02, 0.0, 0.0, 1.0, "viewpointer");	
//	//viewer->addPolygonMesh<pcl::PointXYZ>(pRawCloud, oSDer.m_vSurfaceIdxs, "polyline");
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
//
//}


//////*************for showing the connvex hull in transformation coordiante*********************
//
//#include "GHPR.h"
//#include "HpdPointCloudDisplay.h"
//#include "ConvexHullOperation.h"
//#include "MarchingCubesGHPR.hpp"
//#include "readtxt.h"
//
//
//int main(){
//
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr pRawCloud(new pcl::PointCloud<pcl::PointXYZ>);//原始pcd点云
//
//	std::vector<Point3D> point3d;//Point3D目标的特征计算点云
//	//读取点云数据
//	HPDpointclouddataread("bunny.las", pRawCloud, point3d, 1);
//	std::vector<Point3D> oripoint3d(point3d);
//	//设置视点;
//	pcl::PointXYZ oViewPoint;
//	//x 0.535947 y  0.62239 z 0.535947 bunny
//	//x 0.457275 y  0.500000 z 1.814216 Cassette.las
//	//x 0.0 -y 0.0 z 0.0 scene1oneframe.las
//	oViewPoint.x = 0.535947;
//	oViewPoint.y = 0.62239;
//	oViewPoint.z = 0.535947;
//	//初始化HPR类
//	GHPR oGHPRer(oViewPoint, 3.6);
//	//设置遮挡索引
//	std::vector<int> occindices;
//	oGHPRer.Compute(pRawCloud, false);
//
//	//compute parameters of each face
//	ConvexHullOperation oConvexHullOPer;
//	oConvexHullOPer.ComputeAllFaceParams(*oGHPRer.m_pHullVertices, oGHPRer.m_vHullPolygonIdxs,
//		oConvexHullOPer.m_oCenterPoint, oConvexHullOPer.m_oConvertMatN, oConvexHullOPer.m_vConvertD);
//
//
//	pcl::PointCloud<pcl::PointNormal>::Ptr pRawPN(new pcl::PointCloud<pcl::PointNormal>);
//	for (int i = 0; i != pRawCloud->points.size(); ++i){
//		pcl::PointNormal oOnePN;
//		oOnePN.x = pRawCloud->points[i].x;
//		oOnePN.y = pRawCloud->points[i].y;
//		oOnePN.z = pRawCloud->points[i].z;
//		oOnePN.normal_x = 0.0;
//		oOnePN.normal_y = 0.0;
//		oOnePN.normal_z = 1.0;
//		pRawPN->push_back(oOnePN);
//	}
//
//	//
//	//初始化 移动立方体算法 MarchingCubes对象，并设置参数
//	MarchingCubesGHPR<pcl::PointNormal>::Ptr mc(new MarchingCubesGHPR<pcl::PointNormal>());
//	//创建多变形网格，用于存储结果
//	pcl::PolygonMesh mesh;
//
//	//设置MarchingCubes对象的参数
//	mc->setIsoLevel(0.0f);
//	mc->setGridResolution(10, 10, 10);
//	mc->setPercentageExtendGrid(0.0f);
//
//
//	//设置搜索方法
//	mc->setInputCloud(pRawPN);
//	mc->Voxelization();
//	//mc->UpdataCornerValues();
//	//执行重构，结果保存在mesh中
//	//mc->reconstruct(mesh);
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr pVoxelCorners(new pcl::PointCloud<pcl::PointXYZ>);
//	oGHPRer.ConvertCloud(mc->m_vCornerCloud, *pVoxelCorners);
//
//	//compute the signed distance
//	std::vector<SignedDis> vSDis = oConvexHullOPer.ComputePointSignedDis(*pVoxelCorners);
//
//	std::cout << "is it a inner point: " << vSDis[0].bInner << std::endl;
//	std::cout << "the distance is: " << vSDis[0].fDis << std::endl;
//	std::cout << "the nearest face is: " << vSDis[0].iFaceNum << std::endl;
//
//	//*****display part*****
//	pcl::PointCloud<pcl::Normal>::Ptr pFaceNormal(new pcl::PointCloud<pcl::Normal>);
//	oConvexHullOPer.GetPCLNormal(pFaceNormal);
//	//center of each face
//	pcl::PointCloud<pcl::PointXYZ>::Ptr pCenterCloud(new pcl::PointCloud<pcl::PointXYZ>);//原始pcd点云
//	oConvexHullOPer.ComputeCenterPoint(*oGHPRer.m_pHullVertices, oGHPRer.m_vHullPolygonIdxs, *pCenterCloud);
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr pShowCloud(new pcl::PointCloud<pcl::PointXYZ>);
//	std::vector<int> vInner;
//	for (int i = 0; i != vSDis.size(); ++i){
//		vInner.push_back(vSDis[i].bInner);
//		//if (vSDis[i].bInner)
//		pShowCloud->points.push_back(pVoxelCorners->points[i]);
//	}
//		
//
//	WritePointCloudTxt("visibilitynode.txt", mc->m_vCornerCloud, vInner);
//
//	WritePointCloudTxt("result_corner.txt", mc->m_vCornerCloud, mc->m_vCornerValue);
//	oViewPoint.x = 0.0;
//	oViewPoint.y = 0.0;
//	oViewPoint.z = 0.0;
//	//窗口开启
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
//	HpdDisplay hpdisplay;
//	//viewer = hpdisplay.Showsimplecolor(oGHPRer.m_pHullVertices, "red");
//	viewer = hpdisplay.Showclassification(pShowCloud, vInner, "assign");
//	viewer->addSphere(oViewPoint, 200.0, 0.0, 0.0, 1.0, "viewpointer");
//	//cloud->points.push_back(oViewPoint);
//	//viewer->addPolygonMesh<pcl::PointXYZ>(oGHPRer.m_pHullVertices, oGHPRer.m_vHullPolygonIdxs, "polyline");
//	//viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(pCenterCloud, pFaceNormal, 1, 50.0f, "normal");
//	//viewer->addSphere(oConvexHullOPer.m_oCenterPoint, 5.0f, 0.0, 0.0, 1.0, "centerpointer");
//
//
//	while (!viewer->wasStopped()){
//
//		viewer->spinOnce();
//
//	}
//
//	return 0;
//
//}





/////*************for one scene*********************/
//#include "HpdPointCloudDisplay.h"
//#include "LasOperator.h"
//#include "SectorPartition.h"
//#include "GHPR.h"
//#include <iostream>
//#include <cmath>
//
//
//
//int main() {
//
//
//	std::vector<Point3D> vScenePoints;
//	pcl::PointCloud<pcl::PointXYZ>::Ptr vSceneCloud(new pcl::PointCloud<pcl::PointXYZ>);
//	HPDpointclouddataread("Cassette.las", vSceneCloud, vScenePoints);
//
//	pcl::PointXYZ oViewPoint;
//	//x 0.535947 y  0.62239 z 0.535947 bunny
//	//x 0.457275 y  0.500000 z 1.814216 Cassette.las
//	//x 0.0 -y 0.0 z 0.0 scene1oneframe.las
//	oViewPoint.x = 0.457275;
//	oViewPoint.y = 0.500000;
//	oViewPoint.z = 1.814216;
//
//	std::vector<std::vector<int>> oPointSecIdxs;
//	DivideSector oSectorDivider(20);
//	oSectorDivider.SetOriginPoint(oViewPoint);
//	oSectorDivider.ComputePointSectorIdxs(*vSceneCloud, oPointSecIdxs);
//
//	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vClouds;
//	std::vector<std::vector<pcl::Vertices>> vFaces;
//
//	for (int i = 0; i != oPointSecIdxs.size(); ++i) {
//		
//		pcl::PointCloud<pcl::PointXYZ>::Ptr vOneSectionCloud(new pcl::PointCloud<pcl::PointXYZ>);
//		//get a point clouds in one section
//		for (int j = 0; j != oPointSecIdxs[i].size(); ++j) {
//			int iSecPointInAllIdx = oPointSecIdxs[i][j];
//			vScenePoints[iSecPointInAllIdx].classification = i;
//			vOneSectionCloud->points.push_back(vSceneCloud->points[iSecPointInAllIdx]);
//		}
//
//		GHPR hpdhpr(oViewPoint, 3.6);
//
//		std::vector<int> occindices;
//		hpdhpr.Compute(vOneSectionCloud);
//		occindices = hpdhpr.GetOccludedIdx();
//		std::vector<pcl::Vertices> vOneFaces;
//		vOneFaces = hpdhpr.ConstructSurfaceIdx();
//
//		vClouds.push_back(vOneSectionCloud);
//		vFaces.push_back(vOneFaces);
//	}
//
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
//	HpdDisplay hpdisplay;
//	//viewer=hpdisplay.Showsimplecolor(occloud,"grey");
//	viewer = hpdisplay.Showclassification(vScenePoints, "random");
//	//viewer = hpdisplay.Showclassification(vScenePoints, "random");
//	viewer->addSphere(oViewPoint, 0.2, 0.0, 0.0, 1.0, "viewpointer");
//	//cloud->points.push_back(oViewPoint);
//	for (int i = 0; i != vClouds.size(); ++i){
//
//		std::stringstream sMeshStream;
//		sMeshStream << i << "_sec_mesh";
//		std::string sMeshName;
//		sMeshStream >> sMeshName;
//		viewer->addPolygonMesh<pcl::PointXYZ>(vClouds[i], vFaces[i], sMeshName);
//
//	}
//
//	while (!viewer->wasStopped())
//	{
//		viewer->spinOnce();
//	}
//
//	return 0;
//
//}





////*************for one object*********************
//
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
//	HPDpointclouddataread("Cassette.las",cloud,point3d,1);
//	std::vector<Point3D> oripoint3d(point3d);
//	//设置视点;
//	pcl::PointXYZ oViewPoint;
//	//x 0.535947 y  0.62239 z 0.535947 bunny
//	//x 0.457275 y  0.500000 z 1.814216 Cassette.las
//	//x 0.0 -y 0.0 z 0.0 scene1oneframe.las
//	oViewPoint.x = 0.457275;
//	oViewPoint.y = 0.500000;
//	oViewPoint.z = 1.814216;
//	//初始化HPR类
//	GHPR hpdhpr(oViewPoint, 3.6);
//	//设置遮挡索引
//	std::vector<int> occindices;
//	hpdhpr.Compute(cloud);
//	occindices = hpdhpr.GetOccludedIdx();
//	std::vector<pcl::Vertices> vFaces;
//	vFaces = hpdhpr.ConstructSurfaceIdx();
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


//*************for showing the connvex hull in transformation coordiante*********************

//#include "readtxt.h"
//#include "Cell.h"
//#include "Fusion.h"
//#include "CIsoSurface.h"
//#include "SignedDistance.h"
//#include "HpdPointCloudDisplay.h"
//
//
//int main(){
//
//	//raw point clouds
//	pcl::PointCloud<pcl::PointXYZ>::Ptr pFileCloud (new pcl::PointCloud<pcl::PointXYZ>);
//	std::vector<Point3D> point3d;
//	//read data
//	HPDpointclouddataread("bunny.las", pFileCloud, point3d, 1);
//	point3d.clear();
//	//clear raw data and smapling
//	pcl::PointCloud<pcl::PointXYZ>::Ptr pRawCloud(new pcl::PointCloud<pcl::PointXYZ>);
//	SamplePoints(*pFileCloud, *pRawCloud, 1);
//	pFileCloud->clear();
//
//	//******voxelization********
//	Voxelization oVoxeler(*pRawCloud);
//	//set the number of voxels
//	oVoxeler.GetIntervalNum(60,60,60);
//	//voxelize the space
//	oVoxeler.VoxelizeData();
//
//	//signed distance of each node
//	//this map is constantly updated
//	std::vector<float> vSDMap(oVoxeler.m_pCornerCloud->points.size(), -1.0);
//	std::vector<float> vMinDis = SignedDistance::MinKDDis(pRawCloud, oVoxeler.m_pCornerCloud);
//	std::cout << "finished the kd" << std::endl;
//
//	Fusion oFusion;
//	//******viewpoints********
//	pcl::PointCloud<pcl::PointXYZ>::Ptr pViewPoints(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointXYZ oViewPoint;
//	//x 0.535947 y  0.62239 z 0.535947 bunny
//	//x 0.457275 y  0.500000 z 1.814216 Cassette.las
//	//x 0.0 -y 0.0 z 0.0 scene1oneframe.las
//	oViewPoint.x = 0.535947;
//	oViewPoint.y = -0.62239;
//	oViewPoint.z = 0.535947;
//	pViewPoints->points.push_back(oViewPoint);
//	oViewPoint.x = 0.535947;
//	oViewPoint.y = 0.62239;
//	oViewPoint.z = 0.535947;
//	pViewPoints->points.push_back(oViewPoint);
//	oViewPoint.x = -0.535947;
//	oViewPoint.y = 0.62239;
//	oViewPoint.z = -0.535947;
//	pViewPoints->points.push_back(oViewPoint);
//	oViewPoint.x = 0.335947;
//	oViewPoint.y = 0.12239;
//	oViewPoint.z = 0.535947;
//	pViewPoints->points.push_back(oViewPoint);
//	oViewPoint.x = 0.235947;
//	oViewPoint.y = 0.92239;
//	oViewPoint.z = -0.835947;
//	pViewPoints->points.push_back(oViewPoint);
//	
//	for (int i = 0; i != 1; ++i){
//		//compute signed distance of a query nodes(voxels)
//		SignedDistance oSDer;
//		std::vector<float> vPlanDis = oSDer.ComputeOneView(pRawCloud, pViewPoints->points[i], oVoxeler.m_pCornerCloud);
//		std::vector<float> vCornerSignedDis = oSDer.MinKDDSignedDis(vPlanDis, vMinDis);
//		oFusion.CorrosionFusion(vCornerSignedDis, vSDMap);
//		std::cout << "Finish the " << i << "th viewpoint" << std::endl;
//	}
//
//	//marching cuber
//	CIsoSurface<float> oMarchingCuber;
//	oMarchingCuber.GenerateSurface(vSDMap, 0, 
//		                           oVoxeler.m_iFinalVoxelNum.ixnum - 1, oVoxeler.m_iFinalVoxelNum.iynum - 1, oVoxeler.m_iFinalVoxelNum.iznum - 1, 
//		                           oVoxeler.m_oVoxelLength.x, oVoxeler.m_oVoxelLength.y, oVoxeler.m_oVoxelLength.z);
//
//	//
//	pcl::PolygonMesh oCBModel;
//	oMarchingCuber.OutputMesh(oVoxeler.m_oOriCorner, oCBModel);
//
//	pcl::io::savePLYFileBinary("cb_res.ply", oCBModel);
//	std::cout << "the number of final faces after reconstruction: " << oCBModel.polygons.size() << std::endl;
//	//*****display part*****
//	WritePointCloudTxt("result_corner.txt", *oVoxeler.m_pCornerCloud, vSDMap);
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr pInnerClouds(new pcl::PointCloud<pcl::PointXYZ>);
//	std::vector<float> vInnerValue;
//	for (int i = 0; i != oVoxeler.m_pCornerCloud->size(); ++i){
//		if (vSDMap[i]<0.0){
//			pInnerClouds->push_back(oVoxeler.m_pCornerCloud->points[i]);
//			vInnerValue.push_back(vSDMap[i]);
//		}
//	}
//	WritePointCloudTxt("inner_corner.txt", *pInnerClouds, vInnerValue);
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr pOutlierClouds(new pcl::PointCloud<pcl::PointXYZ>);
//	std::vector<float> vOutlierValue;
//	for (int i = 0; i != oVoxeler.m_pCornerCloud->size(); ++i){
//		if (vSDMap[i] >= 0.0){
//			pOutlierClouds->push_back(oVoxeler.m_pCornerCloud->points[i]);
//			vOutlierValue.push_back(vSDMap[i]);
//		}
//	}
//	WritePointCloudTxt("Outlier_corner.txt", *pOutlierClouds, vOutlierValue);
//	
//	//pcl::PolygonMesh MeshModel;
//	//pcl::toPCLPointCloud2(*pRawCloud,MeshModel.cloud);
//	//MeshModel.polygons = oSDer.m_vSurfaceIdxs;
//	//pcl::io::savePLYFileBinary("reconstruction_res.ply", MeshModel);
//
//	//窗口开启
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
//    HpdDisplay hpdisplay;
//	//viewer = hpdisplay.Showclassification(mc->m_vCornerCloud, vInner, "assign");
//	viewer = hpdisplay.Showfeatureresult(*oVoxeler.m_pCornerCloud, vSDMap, "redgreen");
//	//viewer = hpdisplay.Showclassification(pVoxelCorners, "assign");
//	viewer->addSphere(oViewPoint, 0.02, 0.0, 0.0, 1.0, "viewpointer");	
//	//viewer->addPolygonMesh<pcl::PointXYZ>(pRawCloud, oSDer.m_vSurfaceIdxs, "polyline");
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
//
//}

//#include "readtxt.h"
//#include "Cell.h"
//#include "Fusion.h"
//#include "CIsoSurface.h"
//#include "SignedDistance.h"
//#include "HpdPointCloudDisplay.h"
//#include <pcl/range_image/range_image.h>    //深度图像的头文件
//
//int main(int argc, char** argv) {
//	pcl::PointCloud<pcl::PointXYZ> pointCloud;   //定义点云的对象
//
//	// 循环产生点云的数据
//	for (float y = -0.5f; y <= 0.5f; y += 0.01f) {
//		for (float z = -0.5f; z <= 0.5f; z += 0.01f) {
//			pcl::PointXYZ point;
//			point.x = 2.0f - y;
//			point.y = y;
//			point.z = z;
//			pointCloud.points.push_back(point); //循环添加点数据到点云对象
//		}
//	}
//	pointCloud.width = (uint32_t)pointCloud.points.size();
//	pointCloud.height = 1;                                        //设置点云对象的头信息
//	//实现一个呈矩形形状的点云
//	// We now want to create a range image from the above point cloud, with a 1deg angular resolution
//	//angular_resolution为模拟的深度传感器的角度分辨率，即深度图像中一个像素对应的角度大小
//	float angularResolution = (float)(1.0f * (M_PI / 180.0f));  //   1.0 degree in radians
//	//max_angle_width为模拟的深度传感器的水平最大采样角度，
//	float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f));  // 360.0 degree in radians
//	//max_angle_height为模拟传感器的垂直方向最大采样角度  都转为弧度
//	float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f));  // 180.0 degree in radians
//	//传感器的采集位置
//	Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
//	//深度图像遵循坐标系统
//	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
//	float noiseLevel = 0.00;    //noise_level获取深度图像深度时，近邻点对查询点距离值的影响水平
//	float minRange = 0.0f;     //min_range设置最小的获取距离，小于最小获取距离的位置为传感器的盲区
//	int borderSize = 1;        //border_size获得深度图像的边缘的宽度
//
//	pcl::RangeImage rangeImage;
//	rangeImage.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,
//		sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
//
//	std::cout << rangeImage << "\n";
//	std::cin.get();
//}