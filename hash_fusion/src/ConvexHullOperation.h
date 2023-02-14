#ifndef CONVEXHULLOPERATION_H
#define CONVEXHULLOPERATION_H
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>

#include "HashVoxeler.h"

//The plane general formula is Ax + By + Cz + D = 0, where (A, B, C)is the normal vector of the plane is
//and D is the offset from coordinate origin
//oNormal - the normal vector (A, B, C)
//fDparam - D offset
//FacePara is to store the parameters of a plane. in other words, to parameterize the plane.
struct FacePara{

	//N(A,B,C)
	Eigen::Vector3f oNormal;
	//D
	float fDparam;

};

//a signed distance information - the shortest distance from point to all face
//the positive or negative value of the distance depends on the whetherthe point is outside or inside convex hull
//iFaceNum records the nearest face index so that distance can be recalculated in world coordinates
//In term of this, the distance needs to be calculated in transfored coordinates and in world coordinates, respectively
struct SignedDis{

	SignedDis(){
		bInner = true;
	}

	//distance value -signed
	float fDis;
	//whether it is a inner point
	bool bInner;
	//the nearest triangular face
	int iFaceNum;

};


//The plane general formula is Ax + By + Cz + D = 0, where (A, B, C)is the normal vector of the plane is
//and D is the offset from coordinate origin
//oNormal - the normal vector (A, B, C)
//fDparam - D offset
//FacePara is to store the parameters of a plane. in other words, to parameterize the plane.
class ConvexHullOperation{

public:

	ConvexHullOperation();

	//transform pcl::pointxyz and faceparam to pcl::pointnormal
	void ChangeNormalType(const pcl::PointCloud<pcl::PointXYZ> & vClouds, const std::vector<FacePara> & vFaceParams, pcl::PointCloud<pcl::PointNormal> & vPNormals);

	//compute point-to-face distance along with the direction relative to the normal vector
	float PointToFaceDis(const Eigen::Vector3f & vVecP, const Eigen::Vector3f & vVecN,  const float & fD);
	//reload, compute point to each face distance along with the direction relative to each normal vector
	Eigen::VectorXf PointToFaceDis(const Eigen::Vector3f & vVecP, const Eigen::MatrixXf & oMatN, const Eigen::VectorXf & vfD);


	//*************CaculateTriangleNormal***********
	//calculate normal vectors of plane and triangular faces with pcl type output
	pcl::Normal CaculateTriangleNormal(pcl::PointXYZ & oPZero, pcl::PointXYZ & oPOne, pcl::PointXYZ & oPTwo);

	//reload, caculate triangle face normal with a reference point
	void CaculateTriangleNormal(const pcl::PointXYZ & oPZero, const pcl::PointXYZ & oPOne, const pcl::PointXYZ & oPTwo, 
		const pcl::PointXYZ & oInnerP, FacePara & oFacePara);

	//reload, caculate triangle face normal with a reference point
	void CaculateTriangleNormal(const pcl::PointCloud<pcl::PointXYZ> & vVertices,const pcl::PointXYZ & oInnerP, 
		pcl::Vertices & vOneMeshVertexIdxs, FacePara & oFacePara);

	//reload, caculate triangle face normal without a reference point
	void CaculateTriangleNormal(const pcl::PointXYZ & oPZero, const pcl::PointXYZ & oPOne, const pcl::PointXYZ & oPTwo,
		FacePara & oFacePara);

	//reload, caculate triangle face normal with a normal input and a point triangle in face, and also a reference point to calibration
	void CaculateTriangleNormal(const Eigen::Vector3f & oRefPoint, const pcl::PointNormal & oPNormal,
		pcl::PointXYZ & oPoint, FacePara & oFacePara);

	//reload, caculate triangle face normal with a normal input and a point in face, this function is to compute the D parameter of plan equation
	void CaculateTriangleNormal(const pcl::PointNormal & oPNormal, FacePara & oFacePara);

	//compute the center point based on A given triangle vertices
	void ComputeCenterPoint(const pcl::PointCloud<pcl::PointXYZ> & vVertices, const std::vector<pcl::Vertices> & vMeshVertexIdxs,
		                    pcl::PointCloud<pcl::PointXYZ> & vCenterPoints);
	//reload, compute the center point based on ALL given triangle vertices
	void ComputeCenterPoint(const pcl::PointCloud<pcl::PointXYZ> & vVertices, pcl::PointXYZ & oCenterPoint);


	//*************ComputeAllFaceParams***********
	//Calculate equation parameters for all faces with a given center point (reference)
	void ComputeAllFaceParams(const pcl::PointCloud<pcl::PointXYZ> & vVertices, const std::vector<pcl::Vertices> & vMeshVertexIdxs, Eigen::MatrixXf & oMatNormal, Eigen::VectorXf & vfDParam);

	//Calculate equation parameters for all faces without a given base point (reference)
	void ComputeAllFaceParams(const pcl::PointCloud<pcl::PointXYZ> & vVertices, const std::vector<pcl::Vertices> & vMeshVertexIdxs, pcl::PointXYZ & oCenterPoint,
		                      Eigen::MatrixXf & oMatNormal,Eigen::VectorXf & vfDParam);

	//Calculate equation parameters for all faces with a given viewpoint (reference)
	void ComputeAllFaceParams(const pcl::PointXYZ & oViewPoint, const pcl::PointCloud<pcl::PointXYZ> & vVertices, std::vector<pcl::Vertices> & vMeshVertexIdxs,
		                      Eigen::MatrixXf & oMatNormal, Eigen::VectorXf & vfDParam);

	//Calculate right direction normal for all faces correspoding to a given viewpoint (reference)
	void ComputeAllFaceParams(const pcl::PointXYZ & oViewPoint, const pcl::PointCloud<pcl::PointNormal> & vPNormals,
		                              pcl::PointCloud<pcl::PointXYZ> & vClouds, std::vector<FacePara> & vFaceParams);

	//Calculate equation parameters for all faces with a given base point (reference)
	void ComputeAllFaceParams(const pcl::PointCloud<pcl::PointXYZ> & vVertices, const std::vector<pcl::Vertices> & vMeshVertexIdxs, pcl::PointXYZ & oCenterPoint, 
		                     std::vector<FacePara> & vFaces);

	//Calculate equation parameters for all faces with a given base point (reference)
	void ComputeAllFaceParams(const pcl::PointCloud<pcl::PointNormal> & vPNormals, std::vector<FacePara> & vFaceParams);

	//Calculate equation parameters for all faces with a given base point (reference)
	void ComputeAllFaceParams(const HashVoxeler::HashVolume & vVolume, std::unordered_map<HashPos, FacePara, HashFunc> & vFaceParams);

	//compute face params in two space
	void FaceParamInTwoSpace(const pcl::PointCloud<pcl::PointXYZ> & vConvertVertices, const std::vector<pcl::Vertices> & vConvertMeshIdxs,
		                     const pcl::PointCloud<pcl::PointXYZ> & vEuclidVertices, const std::vector<pcl::Vertices> & vEuclidMeshIdxs);

	//organize the normal and d parameter to the face param
	void NDToFacePara(const Eigen::MatrixXf & oMatN, const Eigen::VectorXf & vDParam, std::vector<FacePara> & vFaces);

	//in TRANSFORMED COORDINATES, get shortest distance and interior point situation from all point-to-face distances 
	SignedDis JudgeSignedValue(const Eigen::VectorXf & vAllDis);

	//compute the signed distance of each query point based on the calculated face parameters
	std::vector<SignedDis> ComputePointSignedDis(pcl::PointCloud<pcl::PointXYZ> & vQueryCloud);
	//reload, compute the signed distance of each query point on the global Euclidean Coordinates with built-out face parameters
	std::vector<float> ComputePointSignedDis(const std::vector<SignedDis> & vConvexSD, const pcl::PointCloud<pcl::PointXYZ> & vQueryCloud,
		                                     const std::vector<FacePara> & vFaceParams);
	//reload, compute the signed distance of each query point on the global Euclidean Coordinates with built-in face parameters
	std::vector<float> ComputePointSignedDis(const std::vector<SignedDis> & vConvexSD, const pcl::PointCloud<pcl::PointXYZ> & vQueryCloud);
	//reload, compute the signed distance of each query point based on face Vertices
	std::vector<float> ComputePointSignedDis(const std::vector<SignedDis> & vConvexSD,const pcl::PointCloud<pcl::PointXYZ> & vQueryCloud,
		                                     const pcl::PointCloud<pcl::PointXYZ> & vCenterPoints);
	//reload, compute the signed distance of each query point based on normal
	std::vector<float> ComputePointSignedDis(const pcl::PointCloud<pcl::PointXYZ> & vQueryCloud, const Eigen::MatrixXf & m_oMatN,const Eigen::VectorXf & m_vD);

	//get the normal vector in pcl
	void GetPCLNormal(pcl::PointCloud<pcl::Normal>::Ptr & pFaceNormal);

	//exchange vertex order to render the face
	void WindingOrder(pcl::Vertices & vVertices);

	//a matrix where each row stores the normal vector of a face in converted space
	//convex hull after convertion
	Eigen::MatrixXf m_oConvertMatN;

	//a vector where each row stores the D parameters of a facet in converted space
	//convex hull after convertion
	Eigen::VectorXf m_vConvertD;

	//a matrix where each row stores the normal vector of a face in Euclid space 
	//(Convert convex hull to real-world coordinates, possibly non-convex)
	Eigen::MatrixXf m_oEuclidMatN;

	//a vector where each row stores the D parameters of a facet in Euclid space 
	//(Convert convex hull to real-world coordinates, possibly non-convex)
	Eigen::VectorXf m_vEuclidD;

	//the center point
	pcl::PointXYZ m_oCenterPoint;

private:

	//A flag indicating that the face parameter has been calculated
	bool bParamFlag;

};




#endif

//******************************one example*************************************
//#include "GHPR.h"
//#include "HpdPointCloudDisplay.h"
//#include "ConvexHullOperation.h"
//
//
//int main(){
//
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);//ԭʼpcd����
//	pcl::PointCloud<pcl::PointXYZ>::Ptr pQuerycloud(new pcl::PointCloud<pcl::PointXYZ>);
//	std::vector<Point3D> point3d;//Point3DĿ��������������
//	//��ȡ��������
//	HPDpointclouddataread("bunny.las", cloud, point3d, 1);
//	std::vector<Point3D> oripoint3d(point3d);
//	//�����ӵ�;
//	pcl::PointXYZ oViewPoint;
//	//x 0.535947 y  0.62239 z 0.535947 bunny
//	//x 0.457275 y  0.500000 z 1.814216 Cassette.las
//	//x 0.0 -y 0.0 z 0.0 scene1oneframe.las
//	oViewPoint.x = 0.535947;
//	oViewPoint.y = 0.62239;
//	oViewPoint.z = 0.535947;
//	//��ʼ��HPR��
//	GHPR hpdhpr(oViewPoint, 3.6);
//	//�����ڵ�����
//	std::vector<int> occindices;
//	hpdhpr.Compute(cloud, false);
//
//	//compute parameters of each face
//	ConvexHullOperation oConvexHullOPer;
//	oConvexHullOPer.ComputeAllFaceParams(*hpdhpr.m_pHullVertices, hpdhpr.m_vHullPolygonIdxs);
//
//	pQuerycloud->points.push_back(oConvexHullOPer.m_oCenterPoint);
//
//	//compute the signed distance
//	std::vector<SignedDis> vSDis = oConvexHullOPer.ComputePointSignedDis(*pQuerycloud);
//
//	std::cout << "is it a inner point: " << vSDis[0].bInner << std::endl;
//	std::cout << "the distance is: " << vSDis[0].fDis << std::endl;
//	std::cout << "the nearest face is: " << vSDis[0].iFaceNum << std::endl;
//
//	//*****display part*****
//	pcl::PointCloud<pcl::Normal>::Ptr pFaceNormal(new pcl::PointCloud<pcl::Normal>);
//	oConvexHullOPer.GetPCLNormal(pFaceNormal);
//	//center of each face
//	pcl::PointCloud<pcl::PointXYZ>::Ptr pCenterCloud(new pcl::PointCloud<pcl::PointXYZ>);//ԭʼpcd����
//	oConvexHullOPer.ComputeCenterPoint(*hpdhpr.m_pHullVertices, hpdhpr.m_vHullPolygonIdxs, *pCenterCloud);
//
//
//	//���ڿ���
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
//	HpdDisplay hpdisplay;
//	viewer = hpdisplay.Showsimplecolor(pCenterCloud, "red");
//	//viewer=hpdisplay.Showclassification(oripoint3d,"assign");
//	viewer->addSphere(oViewPoint, 0.002, 0.0, 0.0, 1.0, "viewpointer");
//	//cloud->points.push_back(oViewPoint);
//	viewer->addPolygonMesh<pcl::PointXYZ>(hpdhpr.m_pHullVertices, hpdhpr.m_vHullPolygonIdxs, "polyline");
//	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(pCenterCloud, pFaceNormal, 1, 50.0f, "normal");
//	viewer->addSphere(oConvexHullOPer.m_oCenterPoint, 5.0f, 0.0, 0.0, 1.0, "centerpointer");
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