#include"tools/PointCloudOperation.h"
#include"tools/OutputUtils.h"

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/copy_point.h>

PointCloudOperation PointCloudOperation::instance;

float PointCloudOperation::EuclideanDistance(const pcl::PointXYZ & oBasedP, const pcl::PointNormal & oTargetP){

    return (oBasedP.getVector3fMap()-oTargetP.getVector3fMap()).norm();
}

void PointCloudOperation::NearbyClouds(const pcl::PointCloud<pcl::PointNormal> & pRawCloud, const pcl::PointXYZ & oBasedP, pcl::PointCloud<pcl::PointNormal> & pNearCloud, float fLength){

	// pNearCloud.clear();
			
	for (int i = 0; i != pRawCloud.size(); ++i){
		
		if (EuclideanDistance(oBasedP, pRawCloud.at(i)) <= fLength) {
			pNearCloud.push_back(pRawCloud.at(i));
			pNearCloud.points.rbegin()->data_c[3] = i; //record the rawcloud index in data_c[3]
		}
	}
}

// void PointCloudOperation::NearbyClouds(CloudVector & pRawCloud, const pcl::PointXYZ & oBasedP, pcl::PointCloud<pcl::PointNormal> & pNearCloud, float fLength) {
	
// 	for (int i = 0; i != pRawCloud.size(); ++i){
		
// 		if (EuclideanDistance(oBasedP, pRawCloud.at(i)) <= fLength) {
// 			pNearCloud.push_back(pRawCloud.at(i));
// 			pNearCloud.points.rbegin()->data_c[3] = i; //record the rawcloud index in data_c[3]
// 		}

// 	}
// }

void PointCloudOperation::ExtractNearbyClouds(pcl::PointCloud<pcl::PointNormal> & pRawCloud, const pcl::PointXYZ & oBasedP, pcl::PointCloud<pcl::PointNormal> & pNearCloud, float fLength){

	// pNearCloud.clear();
	
	for (int i = 0; i < pRawCloud.points.size();){
		
		if (EuclideanDistance(oBasedP, pRawCloud.points[i]) <= fLength) {
			pNearCloud.push_back(pRawCloud.points[i]);
			pNearCloud.points.rbegin()->data_c[3] = i; //record the rawcloud index in data_c[3]

			//删除已经识别的顶点
			std::swap(pRawCloud.points[i], pRawCloud.points.back());
			pRawCloud.points.pop_back();
		}
		else {
			++i;
		}
	}

	pRawCloud.width = static_cast<uint32_t> (pRawCloud.points.size ()); // 模仿pointcloud中的erase函数

};

// void PointCloudOperation::ExtractNearbyClouds(CloudVector & pRawCloud, const pcl::PointXYZ & oBasedP, pcl::PointCloud<pcl::PointNormal> & pNearCloud, float fLength){

// 	for (int i = 0; i < pRawCloud.size();){
		
// 		if (EuclideanDistance(oBasedP, pRawCloud.at(i)) <= fLength) {
// 			pNearCloud.push_back(pRawCloud.at(i));
// 			pNearCloud.points.rbegin()->data_c[3] = i; //record the rawcloud index in data_c[3]

// 			//删除已经识别的顶点
// 			pRawCloud.erase(i);
// 		}
// 		else {
// 			++i;
// 		}
// 	}
// };

void PointCloudOperation::TranslatePointCloudToLocal(
	const pcl::PointNormal& oLidarPos, 
	const pcl::PointCloud<pcl::PointNormal>& vGlobalCloud,
	pcl::PointCloud<pcl::PointNormal>& vLocalCloud)
{


	Eigen::Vector3f translation = -(oLidarPos.getArray3fMap());

	// average 0.5 ms
	// Eigen::MatrixXf vEigenCloud = vGlobalCloud.getMatrixXfMap();
	// vEigenCloud.colwise() += translation;
	// vLocalCloud.resize(vEigenCloud.cols());
	// vLocalCloud.getMatrixXfMap() = vEigenCloud;

	// average 0.17 ms, 使用 assign 拷贝比 = 拷贝快一丢丢
	auto& cloud_in = vGlobalCloud;
	auto& cloud_out = vLocalCloud;
	if (&cloud_in != &cloud_out)
	{
		cloud_out.header   = cloud_in.header;
		cloud_out.is_dense = cloud_in.is_dense;
		cloud_out.width    = cloud_in.width;
		cloud_out.height   = cloud_in.height;
		cloud_out.points.reserve (cloud_in.points.size ());
		cloud_out.points.assign (cloud_in.points.begin (), cloud_in.points.end ());
		cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
		cloud_out.sensor_origin_      = cloud_in.sensor_origin_;
	}
	vLocalCloud.getMatrixXfMap().colwise() += translation;

	// average 0.22 ms
	// pcl::transformPointCloud(vGlobalCloud, vLocalCloud, translation, Eigen::Quaternionf::Identity());

	// average 0.4 ms
	// vLocalCloud = vGlobalCloud;
	// for(pcl::PointNormal& oPoint : vLocalCloud){
	// 	oPoint.getVector3fMap() -= translation;
	// }
}