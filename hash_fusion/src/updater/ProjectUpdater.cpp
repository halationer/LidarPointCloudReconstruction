#include "updater/ProjectUpdater.h"
#include <vector>
#include <unordered_set>

// singleton instance
ProjectUpdater ProjectUpdater::instance;


/** 基于投影的点云融合
   @param 	oLidarPos - 雷达视点的位置
   @param 	vDepthMeasurementCloud - 新一帧的点云
   @param   oVolume - 要融合到的Volume对象
   @param   bKeepVoxel - 是否保留被删除的非占用体素
*/
void ProjectUpdater::SurfelFusionQuick(
    pcl::PointNormal oLidarPos, 
    pcl::PointCloud<pcl::PointNormal>& vDepthMeasurementCloud,
    VolumeBase& oVolume,
    bool bKeepVoxel)
{

	pcl::PointCloud<pcl::PointNormal> vPointCloudBuffer;

	SurfelFusionCore(oLidarPos, vDepthMeasurementCloud, vPointCloudBuffer, oVolume);

	// 更新volume
	oVolume.UpdateConflictResult(vPointCloudBuffer, bKeepVoxel);

}


/** 基于投影的点云融合
   @param 	oLidarPos - 雷达视点的位置
   @param 	vDepthMeasurementCloud - 新一帧的点云
   @param   oVolume - 要融合到的Volume对象
*/
void ProjectUpdater::SurfelFusionCore(
    pcl::PointNormal oLidarPos, 
    pcl::PointCloud<pcl::PointNormal>& vDepthMeasurementCloud, 
    pcl::PointCloud<pcl::PointNormal>& vPointCloudBuffer,
    VolumeBase& oVolume) 
{

	constexpr int pitch_dim_expand = 4;
	constexpr int yaw_dim_expand = 4;
	constexpr int pitch_dim = 180 * pitch_dim_expand;
	constexpr int yaw_dim = 360 * yaw_dim_expand;
	
	constexpr double support_factor = 0.3;
	constexpr double tight_support_factor = 0.1;
	constexpr double normal_support_factor = 0.8;

	//计算点云每个点的角度，将其置于相机像素中 4ms - 8ms
	std::vector<std::vector<double>> depth_image(pitch_dim, std::vector<double>(yaw_dim));
	std::vector<std::vector<int>> depth_index(pitch_dim, std::vector<int>(yaw_dim));
	// std::vector<bool> vCurrentFuseIndex(vDepthMeasurementCloud.size(), false); //因为只记录一个点，哪些点被覆盖需要记录

	// 记录测量点从属的voxel
	std::unordered_set<HashPos, HashFunc> pos_record;

	// conf - 0.01 - 0.5
	// double min_depth = __INT_MAX__; 
	// int min_id = 0, max_id = 0;
	double max_depth = 0;
	for(int i = 0; i < vDepthMeasurementCloud.size(); ++i) {


		const pcl::PointNormal& oCurrentPoint = vDepthMeasurementCloud.at(i);
		Eigen::Vector3f oRefPoint(oCurrentPoint.x - oLidarPos.x, oCurrentPoint.y - oLidarPos.y, oCurrentPoint.z - oLidarPos.z);

		// 记录测量点从属的voxel
		HashPos oPos;
		oVolume.PointBelongVoxelPos(oCurrentPoint, oPos);
		pos_record.emplace(oPos);


		// 计算投影位置和深度
		double depth = oRefPoint.norm();
		if(depth < 1e-5) continue;
		max_depth = max_depth < depth ? depth : max_depth;

		// min_depth = min_depth > depth ? depth : min_depth;
		// if(min_depth == depth) min_id = i;
		// if(max_depth == depth) max_id = i;

		oRefPoint.normalize();
		double yaw = std::atan2((double)oRefPoint.y(), (double)oRefPoint.x()) / M_PI * 180.0;
		double pitch = std::asin((double)oRefPoint.z()) / M_PI * 180.0;

		// 将投影结果存储
		double row = (pitch + 90) * pitch_dim_expand;
		double col = (yaw   + 180) * yaw_dim_expand;
		if(int(row) < 0 || int(row) >= pitch_dim || int(col) < 0 || int(col) >= yaw_dim) continue;
		if(depth_image[int(row)][int(col)] > 0 && depth < depth_image[int(row)][int(col)] || depth_image[int(row)][int(col)] == 0) {
			// vCurrentFuseIndex[depth_index[int(row)][int(col)]] = true;
			depth_image[int(row)][int(col)] = depth;
			depth_index[int(row)][int(col)] = i;
		}

		// 根据surfel在subpixel的位置，添加到邻近的像素中去
		double local_row = row - int(row);
		double local_col = col - int(col);

		// 如果只扩展左右，不扩展上下如何？好像效果还可以，但是仍然不能完全解决单向素拉长导致的错误complict判断问题
		// /* 双向扩展
		if(local_col > 0.6) {
			// 右
			int temp_col = col + 1;
			if(temp_col >= yaw_dim) temp_col = 0;

			if(depth_image[int(row)][temp_col] > 0 && depth < depth_image[int(row)][temp_col] || depth_image[int(row)][temp_col] == 0) {

				// vCurrentFuseIndex[depth_index[int(row)][temp_col]] = true;
				depth_image[int(row)][temp_col] = depth;
				depth_index[int(row)][temp_col] = i;
			}
		}
		else if(local_col < 0.4) {
			// 左
			int temp_col = col - 1;
			if(temp_col < 0) temp_col = yaw_dim - 1;

			if(depth_image[int(row)][temp_col] > 0 && depth < depth_image[int(row)][temp_col] || depth_image[int(row)][temp_col] == 0) {
				// vCurrentFuseIndex[depth_index[int(row)][temp_col]] = true;
				depth_image[int(row)][temp_col] = depth;
				depth_index[int(row)][temp_col] = i;
			}
		}
		// */
	}	
	
	// std::cout << output::format_red << "min depth: " << min_depth << " | conf: " << vDepthMeasurementCloud[min_id].data_n[3] << "\t";
	// std::cout << output::format_red << "max depth: " << max_depth << " | conf: " << vDepthMeasurementCloud[max_id].data_n[3] << output::format_white << std::endl;

	// 将多帧重建结果点云拷贝到Buffer中，并筛选范围内的点
	pcl::PointCloud<pcl::PointNormal> vVolumeCloud;
	oVolume.GetVolumeCloud(vVolumeCloud);
	pcl::PointXYZ oBase(oLidarPos.x, oLidarPos.y, oLidarPos.z);
	m_oPcOperation.NearbyClouds(vVolumeCloud, oBase, vPointCloudBuffer, max_depth);


	// /* 对于之前帧的所有点，对应位置建立匹配关系 60 - 150 ms
	std::vector<float> associated_feature(vPointCloudBuffer.size(), 0);

	for(int i = 0; i < vPointCloudBuffer.size(); ++i) {

		const pcl::PointNormal& oCurrentPoint = vPointCloudBuffer.at(i);
		Eigen::Vector3f oRefPoint(oCurrentPoint.x - oLidarPos.x, oCurrentPoint.y - oLidarPos.y, oCurrentPoint.z - oLidarPos.z);

		double depth = oRefPoint.norm();
		if(depth < 1e-5) continue;
		oRefPoint.normalize();
		double yaw = std::atan2((double)oRefPoint.y(), (double)oRefPoint.x()) / M_PI * 180.0;
		double pitch = std::asin((double)oRefPoint.z()) / M_PI * 180.0;
		
		int row = (pitch + 90) * pitch_dim_expand;
		int col = (yaw   + 180) * yaw_dim_expand;

		if(row < 0 || row >= pitch_dim || col < 0 || col >= yaw_dim) continue;

		int complict_count = 0;

		pcl::PointNormal* pSupportPoint = nullptr;
		pcl::PointNormal* pConflictPoint = nullptr;

		if(row < pitch_dim && col < yaw_dim && depth_image[row][col]) {

			pcl::PointNormal& oAssociatedPoint = vDepthMeasurementCloud.points[depth_index[row][col]];
			double dAssociatedDepth = depth_image[row][col];
			// 判断complict关系
			if(abs(depth - dAssociatedDepth) > support_factor && depth < dAssociatedDepth /*&& abs(depth - dAssociatedDepth) < 5*/) { // 旧点的depth < 新点的depth
				
				++complict_count;
				pConflictPoint = &oAssociatedPoint;
			}
		}

		if(complict_count > 0) {

			//进一步精细的遮挡检查
			
			pcl::PointNormal& oOldPoint = vPointCloudBuffer.at(i);
			float& fOldConfidence = oOldPoint.data_n[3];
			float& fConfidenceRecord = oOldPoint.data_c[0];
			pcl::PointNormal& oNewPoint = *pConflictPoint;
			float& fNewConfidence = oNewPoint.data_n[3];

			Eigen::Vector3f p1(oOldPoint.x - oLidarPos.x, oOldPoint.y - oLidarPos.y, oOldPoint.z - oLidarPos.z);
			Eigen::Vector3f n1(oOldPoint.normal_x, oOldPoint.normal_y, oOldPoint.normal_z);
			Eigen::Vector3f p2(oNewPoint.x - oLidarPos.x, oNewPoint.y - oLidarPos.y, oNewPoint.z - oLidarPos.z);
			Eigen::Vector3f n2(oNewPoint.normal_x, oNewPoint.normal_y, oNewPoint.normal_z);
			// 给测量点加一个bais，减少错误判断
			p2 += n2 * tight_support_factor;

			Eigen::Vector3f vp2 = p2 / p2.norm();
			Eigen::Vector3f v12 = p2 - p1;

			// dynamic surfel radius, according to confidence
			float r = abs(n2.dot(vp2)) * 2 * fNewConfidence;
			// float r = abs(n2.dot(vp2)) * support_factor;
			Eigen::Vector3f vr = v12.dot(vp2) * vp2 - v12;
			float a = p2.norm() - v12.dot(vp2), b = p2.norm();

			HashPos oPos;
			oVolume.PointBelongVoxelPos(oCurrentPoint, oPos);

			if( pos_record.count(oPos) ) { // 避免自穿透现象
				associated_feature[i] = -1.0f;
			}
			else if( vr.norm() < r * a / b && v12.dot(n2) < 0 //保守剔除（在斜圆锥内部）
			|| n1.dot(v12) < -0.2 && n1.dot(Eigen::Vector3f(0,0,1)) < 0.2 && depth < 30 //针对自遮挡加强剔除(法向与视线方向相近 && 法向不朝上 && 距离足够近)
			) { 
				associated_feature[i] = 0.0f;

				// 记录减少的置信度
				fConfidenceRecord = fOldConfidence;
				fOldConfidence = - 0.8 * fNewConfidence;
				// if(fOldConfidence < 0.f) fOldConfidence = 0.f;
			}
			else associated_feature[i] = -1.0f;
		}
		else {
			associated_feature[i] = -1.0f;
		}
	}	
	
	// output dynamic
	m_oRpManager.PublishPointCloud(vPointCloudBuffer, associated_feature, "/debug_associated_point");
}
