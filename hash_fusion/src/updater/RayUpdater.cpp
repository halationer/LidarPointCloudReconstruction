#include"RayUpdater.h"

#include <exception>
#include <unordered_set>
#include "tools/OutputUtils.h"

Eigen::Vector3f RayCaster::m_vLastStartPoint;
Eigen::Vector3f RayCaster::m_vLastEndPoint;
Eigen::Vector3f RayCaster::m_vLastBlockSize;

RayCaster::RayCaster(const Eigen::Vector3f& vStartPoint, const Eigen::Vector3f& vEndPoint, const Eigen::Vector3f& vBlockSize)
    :m_vStartPoint(vStartPoint), m_vEndPoint(vEndPoint), m_vBlockSize(vBlockSize) {
    
    m_vLastStartPoint = vStartPoint;
    m_vLastEndPoint = vEndPoint;
    m_vLastBlockSize = vBlockSize;
    
    InitCasterParams();
}


bool RayCaster::GetNextHashPos(HashPos& oBlockPos) {

    if(m_iCurrentStep++ > m_iStepNum) {
        return false;
    }

    oBlockPos.x = m_vCurrentPos.x();
    oBlockPos.y = m_vCurrentPos.y();
    oBlockPos.z = m_vCurrentPos.z();
    
    int iDimToGo;
    m_vCurrentToNearstCorner.minCoeff(&iDimToGo);
    m_vCurrentPos[iDimToGo] += m_vStepDirection[iDimToGo];
    m_vCurrentToNearstCorner[iDimToGo] += m_vStepDistance[iDimToGo];
    
    return true;
}


void RayCaster::InitCasterParams() {

    Eigen::Vector3i m_vEndPos = m_vEndPoint.cwiseProduct(m_vBlockSize.cwiseInverse()).array().floor().cast<int>();
    m_vCurrentPos = m_vStartPoint.cwiseProduct(m_vBlockSize.cwiseInverse()).array().floor().cast<int>();
    m_iStepNum = (m_vCurrentPos - m_vEndPos).cwiseAbs().sum();
    m_iCurrentStep = 0;

    Eigen::Vector3f vCurrentToEnd = m_vEndPoint - m_vStartPoint;
    m_vStepDirection = vCurrentToEnd.cwiseSign().cast<int>();

    Eigen::Vector3i vNextCornerDirection = m_vStepDirection.cwiseMax(Eigen::Vector3i::Zero());
    m_vCurrentToNearstCorner = m_vBlockSize.cwiseProduct((m_vCurrentPos + vNextCornerDirection).cast<float>()) - m_vStartPoint;
    m_vStepDistance = m_vStepDirection.cast<float>().cwiseProduct(m_vBlockSize);
    
    vCurrentToEnd = vCurrentToEnd.cwiseInverse();
    m_vCurrentToNearstCorner = m_vCurrentToNearstCorner.cwiseProduct(vCurrentToEnd);
    m_vStepDistance = m_vStepDistance.cwiseProduct(vCurrentToEnd);
}

void RayCaster::MakeRayDebugMarker(const RayCaster::HashPosSet& vBlockList, visualization_msgs::MarkerArray& oOutputVolume, int iIdOffset) {

    constexpr int index = 2e4;

    // make blocks
	visualization_msgs::Marker oVolumeMarker;
	oVolumeMarker.header.frame_id = "map";
	oVolumeMarker.header.stamp = ros::Time::now();
	oVolumeMarker.type = visualization_msgs::Marker::CUBE_LIST;
	oVolumeMarker.action = visualization_msgs::Marker::MODIFY;
	oVolumeMarker.id = index + 2 * iIdOffset; 

	oVolumeMarker.scale.x = m_vLastBlockSize.x();
	oVolumeMarker.scale.y = m_vLastBlockSize.y();
	oVolumeMarker.scale.z = m_vLastBlockSize.z();

	oVolumeMarker.pose.position.x = 0.0;
	oVolumeMarker.pose.position.y = 0.0;
	oVolumeMarker.pose.position.z = 0.0;

	oVolumeMarker.pose.orientation.x = 0.0;
	oVolumeMarker.pose.orientation.y = 0.0;
	oVolumeMarker.pose.orientation.z = 0.0;
	oVolumeMarker.pose.orientation.w = 1.0;

	oVolumeMarker.color.a = std::min(0.2 * (iIdOffset+1), 1.0);
	oVolumeMarker.color.r = 0.8;
	oVolumeMarker.color.g = 0.2;
	oVolumeMarker.color.b = 0.2;

	for(const HashPos& oPos : vBlockList) {

		Eigen::Vector3f oPoint(oPos.x, oPos.y, oPos.z);
        oPoint = oPoint.cwiseProduct(m_vLastBlockSize);
        oPoint += 0.5 * m_vLastBlockSize;

		geometry_msgs::Point point;
		point.x = oPoint.x();
		point.y = oPoint.y();
		point.z = oPoint.z();
		oVolumeMarker.points.push_back(point);
	}

	oOutputVolume.markers.push_back(oVolumeMarker);

    // make ray
    oVolumeMarker.id = index + 1 + 2 * iIdOffset;
    oVolumeMarker.type = visualization_msgs::Marker::LINE_STRIP;
    oVolumeMarker.scale.x = 0.1;
    oVolumeMarker.color.a = 1.0;
    oVolumeMarker.points.clear();
    geometry_msgs::Point point;
    point.x = m_vLastStartPoint.x();
    point.y = m_vLastStartPoint.y();
    point.z = m_vLastStartPoint.z();
    oVolumeMarker.points.push_back(point);
    point.x = m_vLastEndPoint.x();
    point.y = m_vLastEndPoint.y();
    point.z = m_vLastEndPoint.z();
    oVolumeMarker.points.push_back(point);
    
    oOutputVolume.markers.push_back(oVolumeMarker);
}

///////////////////////////////////////////////// Ray Updater /////////////////////////////////////////////////////
RayUpdater RayUpdater::instance;

/**
 * @brief ray_cast method to update volume
 * @param oLidarPos lidar center pos as ray start pos
 * @param vDepthMeasurementCloud one frame point cloud, and its center is oLidarPos
 * @param oVolume the volume to be updated
 * @param bKeepVoxel wether to keep the voxel that considered to be dynamic
*/
void RayUpdater::RayFusion(
    const pcl::PointNormal& oLidarPos, 
    pcl::PointCloud<pcl::PointNormal>& vDepthMeasurementCloud,
    VolumeBase& oVolume,
    bool bKeepVoxel)
{
	m_oFuseTimer.NewLine();

    if(vDepthMeasurementCloud.empty()) return;

    HashBlock* pHashBlockVolume = dynamic_cast<HashBlock*>(&oVolume);
    if(pHashBlockVolume == nullptr) {
        std::cout << output::format_red << "[updater/RayUpdater] The volume type is not HashBlock!" << output::format_white << std::endl;
        return;
    }

	m_oFuseTimer.DebugTime("1_convert_ptr");

    /**
     * 如果依然使用纯 voxel 的形式，则 voxel 数量的峰值为 300,000 左右。
     * 然而仅仅射线追踪时间大大超出预期(平均80，最大199），在峰值情况下，使用纯 voxel 追踪几乎是无法达到实时的
    */

    /**
     * 如果写成分 block 的形式，则需要 block_pos 和 point_cloud 同时传入方法,
     * 通过投影的形式，遍历 block 内部的所有 voxel，根据与投影点云的比对进行更新。
     * 通过测试，在 0.4 voxel size, 16x16x16=4096 的 block 数量峰值为 800 左右，此时射线追踪好是约20ms左右（平均6ms，峰值20ms）
     * 那么遍历所有的 voxel 需要 4,086,000 次操作，在不考虑多线程的情况下，允许每次操作的时间约为 2ms
    */
    HashPos oNextHashPos;
    visualization_msgs::MarkerArray oOutputBlocks;
    std::unordered_set<HashPos, HashFunc> vHashPosSet;
    m_vPosRecord.clear();
    for(int i = 0; i < vDepthMeasurementCloud.size(); ++i) {

        RayCaster oRayCaster(oLidarPos.getVector3fMap(), vDepthMeasurementCloud[i].getVector3fMap(), pHashBlockVolume->m_vBlockSize);
        while(oRayCaster.GetNextHashPos(oNextHashPos)) {
            vHashPosSet.emplace(oNextHashPos);
        }

        // 记录测量点从属的voxel
        HashPos oPos;
        oVolume.PointBelongVoxelPos(vDepthMeasurementCloud[i], oPos);
        m_vPosRecord.emplace(oPos);
    }
    
    m_oRpManager.PublishMarkerArray(oOutputBlocks, "/ray_cast_debug", [&](visualization_msgs::MarkerArray& oMarkerArray){
        std::cout << output::format_purple 
            << "Cloud Size: " << vDepthMeasurementCloud.size() 
            << " | Voxel Num: " << vHashPosSet.size() 
            << " | " << pHashBlockVolume->PrintVolumeStatus() << output::format_white << std::endl;
        RayCaster::MakeRayDebugMarker(vHashPosSet, oMarkerArray);
    });
	
	m_oFuseTimer.DebugTime("2_get_blocks");


    // 转换坐标系 0.2ms
    // 投影点云 10ms, m_vDepthImage, m_vDepthIndex
    pcl::PointCloud<pcl::PointNormal> vLocalCloud;
    m_oPcOperation.TranslatePointCloudToLocal(oLidarPos, vDepthMeasurementCloud, vLocalCloud);
    GetDepthAndIndexMap(vLocalCloud);

	m_oFuseTimer.DebugTime("3_convert_pc");

    for(const HashPos& oBlockPos : vHashPosSet) {
		Block oBlock;
		pHashBlockVolume->GetBlockCopy(oBlockPos, oBlock);
		m_oFuseTimer.DebugTime("4_1_BlockCopy");

        BlockFusion(oLidarPos, vLocalCloud, pHashBlockVolume, oBlock);
		m_oFuseTimer.DebugTime("4_2_BlockFusion"); // this is the bottleneck

		pHashBlockVolume->UpdateConflictResult(oBlock, bKeepVoxel);
		m_oFuseTimer.DebugTime("4_3_Update");
    }
	
	// m_oFuseTimer.DebugTime("4_fusion_all");
	m_oFuseTimer.GetCurrentLineTime();
	m_oFuseTimer.CoutCurrentLine();
}

constexpr int pitch_dim_expand = 4;
constexpr int yaw_dim_expand = 4;
constexpr int pitch_dim = 180 * pitch_dim_expand;
constexpr int yaw_dim = 360 * yaw_dim_expand;

constexpr double support_factor = 0.3;
constexpr double tight_support_factor = 0.1;
constexpr double normal_support_factor = 0.8;

/**
 * @brief projective method to update a block
 * @param oLidarPos lidar center pos as project ball center
 * @param vLocalCloud one frame point cloud, and its center is oLidarPos
 * @param oBlockPos hash pos of a block
 * @return oBlock return the result of conflict
*/
void RayUpdater::BlockFusion(
    const pcl::PointNormal& oLidarPos, 
    pcl::PointCloud<pcl::PointNormal>& vLocalCloud,
	HashBlock* pHashBlockVolume,
    Block& oBlock)
{
    Eigen::Vector3f vBlockBasePos = pHashBlockVolume->HashBlockPosTo3DPos(oBlock.pos);

	// /* 对于之前帧的所有点，对应位置建立匹配关系 60 - 150 ms
	// std::vector<float> associated_feature(oBlock.size(), 0);
    for(int i = 0; i < oBlock.size(); ++i) {
        
        VoxelBase& oVoxel = oBlock[i];

        Eigen::Vector3f vRefPoint;
        if(oVoxel.IsOccupied()) vRefPoint = oVoxel.point.getVector3fMap();
        else vRefPoint = pHashBlockVolume->HashVoxelIndexTo3DPos(i) + vBlockBasePos;

        // 避免自穿透现象，这几句也相当消耗性能
        HashPos oPos;
        pHashBlockVolume->PointBelongVoxelPos(vRefPoint, oPos);
        if( m_vPosRecord.count(oPos) ) { 
            // associated_feature[i] = -1.0f;
            continue;
        }

		// 此处消耗一半时间
		static const float DEG_RAD_FIX = 180.0f / M_PIf32;
        vRefPoint -= oLidarPos.getVector3fMap();
        double depth = vRefPoint.norm();
		if(depth < 1e-5) continue;
        Eigen::Vector3f oDirection = vRefPoint.normalized();
		float yaw = std::atan2(oDirection.y(), oDirection.x()) * DEG_RAD_FIX;
		float pitch = std::asin(oDirection.z()) * DEG_RAD_FIX;
		int row = (pitch + 90) * pitch_dim_expand;
		int col = (yaw   + 180) * yaw_dim_expand;

		if(row < 0 || row >= pitch_dim || col < 0 || col >= yaw_dim) continue;

		int complict_count = 0;

		pcl::PointNormal* pConflictPoint = nullptr;

		if(row < pitch_dim && col < yaw_dim && m_vDepthImage[row][col]) {

			pcl::PointNormal& oAssociatedPoint = vLocalCloud[m_vDepthIndex[row][col]];
			double dAssociatedDepth = m_vDepthImage[row][col];
			// 判断complict关系
			if(abs(depth - dAssociatedDepth) > support_factor && depth < dAssociatedDepth) { // 旧点的depth < 新点的depth
				
				++complict_count;
				pConflictPoint = &oAssociatedPoint;
			}
		}

		if(complict_count > 0) {

			//进一步精细的遮挡检查
			
			float& fOldConfidence = oVoxel.point.data_n[3];
			float& fConfidenceRecord = oVoxel.point.data_c[0];
			pcl::PointNormal& oNewPoint = *pConflictPoint;
			float& fNewConfidence = oNewPoint.data_n[3];

			Eigen::Vector3f n1 = oVoxel.point.getNormalVector3fMap();
			Eigen::Vector3f p2 = pConflictPoint->getVector3fMap();
			Eigen::Vector3f n2 = pConflictPoint->getNormalVector3fMap();
			// 给测量点加一个bais，减少错误判断
			p2 += n2 * tight_support_factor;

			Eigen::Vector3f vp2 = p2 / p2.norm();
			Eigen::Vector3f v12 = p2 - vRefPoint;

			// dynamic surfel radius, according to confidence
			float r = abs(n2.dot(vp2)) * 2 * fNewConfidence;
			// float r = abs(n2.dot(vp2)) * support_factor;
			Eigen::Vector3f vr = v12.dot(vp2) * vp2 - v12;
			float a = p2.norm() - v12.dot(vp2), b = p2.norm();


			if( vr.norm() < r * a / b && v12.dot(n2) < 0 //保守剔除（在斜圆锥内部）
			|| n1.dot(v12) < -0.2 && n1.dot(Eigen::Vector3f(0,0,1)) < 0.2 && depth < 30 //针对自遮挡加强剔除(法向与视线方向相近 && 法向不朝上 && 距离足够近)
			) { 
				// associated_feature[i] = 0.0f;

				// 记录减少的置信度
				fConfidenceRecord = fOldConfidence;
				fOldConfidence = - 0.8 * fNewConfidence;
				oVoxel.SetFreeSpace();
				// if(fOldConfidence < 0.f) fOldConfidence = 0.f;
			}
			// else associated_feature[i] = -1.0f;
		}
		// else {
		// 	associated_feature[i] = -1.0f;
		// }
    }
}

/**
 * @brief project the local cloud onto a sphere image
 * @param vLocalCloud the local cloud
 * @return vDepthImage the output image with depth |
 * @return vDepthIndex the output image with index of the cloud
*/
void RayUpdater::GetDepthAndIndexMap(pcl::PointCloud<pcl::PointNormal>& vLocalCloud)
{
	//计算点云每个点的角度，将其置于相机像素中 4ms - 8ms
	m_vDepthImage.resize(pitch_dim, std::vector<double>(yaw_dim));
	m_vDepthIndex.resize(pitch_dim, std::vector<size_t>(yaw_dim));
	// std::vector<bool> vCurrentFuseIndex(vDepthMeasurementCloud.size(), false); //因为只记录一个点，哪些点被覆盖需要记录

	for(int i = 0; i < vLocalCloud.size(); ++i) {

		Eigen::Vector3d vRefPoint = vLocalCloud[i].getVector3fMap().cast<double>();

		// 计算投影位置和深度
		double depth = vRefPoint.norm();
		if(depth < 1e-5) continue;

		vRefPoint.normalize();
		double yaw = std::atan2(vRefPoint.y(), vRefPoint.x()) / M_PI * 180.0;
		double pitch = std::asin(vRefPoint.z()) / M_PI * 180.0;

		// 将投影结果存储
		double row = (pitch + 90) * pitch_dim_expand;
		double col = (yaw   + 180) * yaw_dim_expand;
		if(int(row) < 0 || int(row) >= pitch_dim || int(col) < 0 || int(col) >= yaw_dim) continue;
		if(m_vDepthImage[int(row)][int(col)] > 0 && depth < m_vDepthImage[int(row)][int(col)] || m_vDepthImage[int(row)][int(col)] == 0) {
			// vCurrentFuseIndex[m_vDepthIndex[int(row)][int(col)]] = true;
			m_vDepthImage[int(row)][int(col)] = depth;
			m_vDepthIndex[int(row)][int(col)] = i;
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

			if(m_vDepthImage[int(row)][temp_col] > 0 && depth < m_vDepthImage[int(row)][temp_col] || m_vDepthImage[int(row)][temp_col] == 0) {

				// vCurrentFuseIndex[m_vDepthIndex[int(row)][temp_col]] = true;
				m_vDepthImage[int(row)][temp_col] = depth;
				m_vDepthIndex[int(row)][temp_col] = i;
			}
		}
		else if(local_col < 0.4) {
			// 左
			int temp_col = col - 1;
			if(temp_col < 0) temp_col = yaw_dim - 1;

			if(m_vDepthImage[int(row)][temp_col] > 0 && depth < m_vDepthImage[int(row)][temp_col] || m_vDepthImage[int(row)][temp_col] == 0) {
				// vCurrentFuseIndex[m_vDepthIndex[int(row)][temp_col]] = true;
				m_vDepthImage[int(row)][temp_col] = depth;
				m_vDepthIndex[int(row)][temp_col] = i;
			}
		}
		// */
	}	
}