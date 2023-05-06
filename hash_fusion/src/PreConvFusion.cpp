#include "PreConvFusion.h"


// voxelize the points and fuse them
void ExpandVoxeler::VoxelizePointsAndFusion(pcl::PointCloud<pcl::PointNormal> & vCloud) {

	std::unique_lock<std::shared_mutex> volume_write_lock(m_mVolumeLock);
	++m_iFrameCount;

	// put points into voxels
    std::unordered_map<HashPos, std::vector<int>, HashFunc> vPointContainer;
	HashPos oCurrentPos;
	for(int i = 0; i < vCloud.size(); ++i) {
		const pcl::PointNormal & oCurrentPoint = vCloud.at(i);
		PointBelongVoxelPos(oCurrentPoint, oCurrentPos);
		vPointContainer[oCurrentPos].emplace_back(i);
	}

	// fusion the points
	Fusion oVoxelFusion;
	for(auto&& [oCenterPos, vIndexList] : vPointContainer) {

        float& fConflictTime = m_vVolume[oCenterPos].data_c[1];
        m_pUpdateStrategy->Support(fConflictTime);

		for(int dz = -m_iConvHalfDim; dz <= m_iConvHalfDim; ++dz) {
		for(int dy = -m_iConvHalfDim; dy <= m_iConvHalfDim; ++dy) {
		for(int dx = -m_iConvHalfDim; dx <= m_iConvHalfDim; ++dx) {
        
            HashPos oPos(oCenterPos.x + dx, oCenterPos.y + dy, oCenterPos.z + dz);

            // init base point
            pcl::PointNormal oBasePoint;
            if(m_vVolume.count(oPos)) {
                oBasePoint = m_vVolume[oPos];
            }

            // fusion points
            pcl::PointNormal oFusedPoint = oVoxelFusion.NormalFusionWeighted(vIndexList, vCloud, oBasePoint);
            float& fTimeStamp = oFusedPoint.data_c[2];
            fTimeStamp = m_iFrameCount;
            m_vVolume[oPos] = oFusedPoint;

            // update recent volume
            m_vRecentVolume[oPos] = oFusedPoint;
        }}}

	}

	// delete not recent voxels
	std::vector<HashPos> vPosToDelete;
	for(auto && [oPos,oPoint] : m_vRecentVolume) {
		float& fTimeStamp = oPoint.data_c[2];
		if(m_iFrameCount - fTimeStamp >= m_iMaxRecentKeep)
			vPosToDelete.emplace_back(oPos);
	}
	for(auto && oPos : vPosToDelete)
		m_vRecentVolume.erase(oPos);
}

// decrease the confidence of dynamic point and record conflict
void ExpandVoxeler::UpdateConflictResult(const pcl::PointCloud<pcl::PointNormal> & vVolumeCloud) {
    
	std::unique_lock<std::shared_mutex> volume_write_lock(m_mVolumeLock);

	HashPos oCenterPos;
	for(auto && oVoxelPoint : vVolumeCloud) {

        // check if the voxel is confilcted
		float fDeconfidence = oVoxelPoint.data_n[3];
        if(fDeconfidence >= 0) continue;

        const float& fConfidenceRecord = oVoxelPoint.data_c[0];

		PointBelongVoxelPos(oVoxelPoint, oCenterPos);
        float& fConflictTime = m_vVolume[oCenterPos].data_c[1];
        bool bToDelete = m_pUpdateStrategy->Conflict(fConflictTime);
        if(bToDelete) fDeconfidence = -fConfidenceRecord;

		for(int dz = -m_iConvHalfDim; dz <= m_iConvHalfDim; ++dz) {
		for(int dy = -m_iConvHalfDim; dy <= m_iConvHalfDim; ++dy) {
		for(int dx = -m_iConvHalfDim; dx <= m_iConvHalfDim; ++dx) {
        
            HashPos oPos(oCenterPos.x + dx, oCenterPos.y + dy, oCenterPos.z + dz);
            pcl::PointNormal& oVoxel = m_vVolume[oPos];
            float& fConfidence = oVoxel.data_n[3];

            if(-fDeconfidence < fConfidence) {
                oVoxel.x = fConfidence * oVoxel.x + fDeconfidence * oVoxelPoint.x;
                oVoxel.y = fConfidence * oVoxel.y + fDeconfidence * oVoxelPoint.y;
                oVoxel.z = fConfidence * oVoxel.z + fDeconfidence * oVoxelPoint.z;
                oVoxel.normal_x = fConfidence * oVoxel.normal_x + fDeconfidence * oVoxelPoint.normal_x;
                oVoxel.normal_y = fConfidence * oVoxel.normal_y + fDeconfidence * oVoxelPoint.normal_y;
                oVoxel.normal_z = fConfidence * oVoxel.normal_z + fDeconfidence * oVoxelPoint.normal_z;              
                fConfidence -= fDeconfidence;
                oVoxel.x /= fConfidence;
                oVoxel.y /= fConfidence;
                oVoxel.z /= fConfidence;
                oVoxel.normal_x /= fConfidence;
                oVoxel.normal_y /= fConfidence;
                oVoxel.normal_z /= fConfidence;  
            }
            else {
                fConfidence = 0;
            }
        }}}
	}
}


std::unordered_map<HashPos, float, HashFunc> & PreConvSD::PreConvGlance(HashVoxeler & oVoxeler, ExpandVoxeler & oPreConvVoxeler) {
    
    HashVoxeler::HashVolume vTempVolumeCopy;
	oVoxeler.GetRecentVolume(vTempVolumeCopy, m_iKeepTime);

    HashVoxeler::HashVolume vConvedVolumeCopy;
    oPreConvVoxeler.GetRecentVolume(vConvedVolumeCopy, m_iKeepTime);

    m_vVolumeCopy = vTempVolumeCopy;

	// /* expand - not that good, only fill small holes, but runs too slow(if 3d expand)
	for(auto && [oPos,oPoint] : vTempVolumeCopy) {

		// judge normal main direction to speed up
		Eigen::Vector3f vNormal(oPoint.normal_x, oPoint.normal_y, oPoint.normal_z);
		vNormal = vNormal.cwiseAbs();
		int index;
		vNormal.maxCoeff(&index);
		int start_x = index == 0 ? 0 : -m_iConvHalfDim;
		int start_y = index == 1 ? 0 : -m_iConvHalfDim;
		int start_z = index == 2 ? 0 : -m_iConvHalfDim;
		int end_x   = index == 0 ? 0 : m_iConvHalfDim;
		int end_y   = index == 1 ? 0 : m_iConvHalfDim;
		int end_z   = index == 2 ? 0 : m_iConvHalfDim;
	 
		for(int dx = start_x; dx <= end_x; ++dx) {
		for(int dy = start_y; dy <= end_y; ++dy) {
		for(int dz = start_z; dz <= end_z; ++dz) {

			const HashPos oCurrentPos(oPos.x + dx, oPos.y + dy, oPos.z + dz);
			if(!vTempVolumeCopy.count(oCurrentPos)) {
				vTempVolumeCopy[oCurrentPos] = pcl::PointNormal();
				vTempVolumeCopy[oCurrentPos].data_c[2] = 0;
			}
		}}}
	}
	//*/

	// /*
	clock_t start_time, conv_time;

	// multi-resolution by convolution method
	start_time = clock();
	for(auto && [oPos,_] : vTempVolumeCopy) {

        pcl::PointNormal& oFusedNormal = vConvedVolumeCopy[oPos];
		float & normal_distribution_distance = oFusedNormal.data_c[3];
		float & all_weight = oFusedNormal.data_n[3];
        int point_count = 4; // TODO: record the point count

		if(all_weight > 0) {

			// use min level to update the normal
			const HashPos& oCurrentPos = oPos;
			if(normal_distribution_distance > m_fConvFusionDistanceRef1) {

				// copy fused points to octree neighbors
				if(point_count < 3) {
					m_vVolumeCopy.erase(oCurrentPos);
				}
				else if(m_vVolumeCopy.count(oCurrentPos) || point_count >= m_iConvAddPointNumRef) {
					
					uint32_t voxel_token = m_vVolumeCopy.count(oCurrentPos) ? 0x3fffffff /*means the point is fused*/: __INT_MAX__ /* means the point is added*/;

					pcl::PointNormal& oVoxelNormal = m_vVolumeCopy[oCurrentPos];
					oVoxelNormal.x = oFusedNormal.x;
					oVoxelNormal.y = oFusedNormal.y;
					oVoxelNormal.z = oFusedNormal.z;
					oVoxelNormal.data_c[1] = voxel_token;
					oVoxelNormal.data_c[3] = normal_distribution_distance;
					oVoxelNormal.normal_x = oFusedNormal.normal_x;
					oVoxelNormal.normal_y = oFusedNormal.normal_y;
					oVoxelNormal.normal_z = oFusedNormal.normal_z;
					oVoxelNormal.data_n[3] = all_weight;
				}
			}
			// add point whatever
			else if(!m_vVolumeCopy.count(oCurrentPos) && point_count > 0) {
					
					pcl::PointNormal& oVoxelNormal = m_vVolumeCopy[oCurrentPos];
					oVoxelNormal.x = oFusedNormal.x;
					oVoxelNormal.y = oFusedNormal.y;
					oVoxelNormal.z = oFusedNormal.z;
					oVoxelNormal.data_c[1] = __INT_MAX__;
					oVoxelNormal.data_c[3] = normal_distribution_distance;
					oVoxelNormal.normal_x = oFusedNormal.normal_x;
					oVoxelNormal.normal_y = oFusedNormal.normal_y;
					oVoxelNormal.normal_z = oFusedNormal.normal_z;
					oVoxelNormal.data_n[3] = all_weight;
			}
		}

	}
	conv_time = 1000.0 * (clock() - start_time) / CLOCKS_PER_SEC;
	// std::cout << "conv_time: " << conv_time << "ms" << std::endl;
	//*/

	// meshing
	const HashVoxeler::HashVolume & vVolumeToMeshing = m_vVolumeCopy;

	//compute the sampled point normal based on the face normal
	ConvexHullOperation oConvexHullOPer;

	std::unordered_map<HashPos, FacePara, HashFunc> vVoxelNormalPara;

	//compute the sampled point normal and divde it into point and normal
	oConvexHullOPer.ComputeAllFaceParams(vVolumeToMeshing, vVoxelNormalPara);

	//output
	return PlanDistance(vVolumeToMeshing, vVoxelNormalPara, oVoxeler.m_oVoxelLength);

}


void PreConvFusion::LazyLoading() {

    std::cout << "Load Pre Convolution Fusion..." << std::endl;

    m_oPreConvVoxeler.SetResolution(m_oVoxeler.m_oVoxelLength);
    m_oPreConvVoxeler.GetStrategy(m_oVoxeler);
    m_oPreConvVoxeler.m_iMaxRecentKeep = m_oVoxeler.m_iMaxRecentKeep;
}


bool PreConvFusion::ReadLaunchParams(ros::NodeHandle & nodeHandle) {

    m_oPreConvVoxeler.SetConvDim(m_iConvDim);

	return true;
}


void PreConvFusion::SlideModeling(pcl::PolygonMesh & oResultMesh, const int iFrameId) {

    // Super::SlideModeling(oResultMesh, iFrameId);
    
	if(m_bUseUnionSetConnection) {
		m_oVoxeler.RebuildUnionSet();
		m_oVoxeler.UpdateUnionConflict();
	}
    
	//******make mesh********
    //replace mesh of bottom
    // pcl::PolygonMesh oPreConvMesh;
    pcl::PolygonMesh& oPreConvMesh = oResultMesh;
    oPreConvMesh.cloud.data.clear();
    oPreConvMesh.polygons.clear();

	//using signed distance
	PreConvSD oSDer(m_iKeepTime, m_iConvDim, m_iConvAddPointNumRef, m_fConvFusionDistanceRef1);
	//compute signed distance based on centroids and its normals within voxels
	std::unordered_map<HashPos, float, HashFunc> vSignedDis = oSDer.PreConvGlance(m_oVoxeler, m_oPreConvVoxeler);

	//marching cuber
	CIsoSurface<float> oMarchingCuber;
	//get surface mesh based on voxel related algorithm
	oMarchingCuber.GenerateSurface(vSignedDis, oSDer.m_vVolumeCopy, 0, 
        m_oPreConvVoxeler.m_oVoxelLength.x, m_oPreConvVoxeler.m_oVoxelLength.y, m_oPreConvVoxeler.m_oVoxelLength.z);

	//move the mesh position to the actual starting point
	pcl::PointCloud<pcl::PointXYZ>::Ptr pMCResultCloud(new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::PointXYZ oOffset(0, 0, 0);
	oMarchingCuber.OutputMesh(pcl::PointXYZ(0,0,0), oPreConvMesh, pMCResultCloud);
	
	///* output result mesh
	if(m_bOutputFiles) {

		std::stringstream sOutputPath;
		sOutputPath << m_sFileHead << std::setw(4) << std::setfill('0') << iFrameId << "_down_mesh.ply";
		pcl::io::savePLYFileBinary(sOutputPath.str(), oPreConvMesh);
	}
	//*/
}

void PreConvFusion::SurfelFusionQuick(pcl::PointNormal oLidarPos, pcl::PointCloud<pcl::PointNormal>& vDepthMeasurementCloud) {

	pcl::PointCloud<pcl::PointNormal> vPointCloudBuffer;

	SurfelFusionCore(oLidarPos, vDepthMeasurementCloud, vPointCloudBuffer);

	// 更新volume
	m_oVoxeler.UpdateConflictResult(vPointCloudBuffer);

    // m_oPreConvVoxeler.UpdateConflictResult(vPointCloudBuffer);
}

void PreConvFusion::UpdateOneFrame(const pcl::PointNormal& oViewPoint, pcl::PointCloud<pcl::PointNormal>& vFilteredMeasurementCloud) {

    Super::UpdateOneFrame(oViewPoint, vFilteredMeasurementCloud);

    m_oPreConvVoxeler.VoxelizePointsAndFusion(vFilteredMeasurementCloud);
}