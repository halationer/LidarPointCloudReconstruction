#include"SignedDistance.h"


SignedDistance::SignedDistance(int iKeepTime, int iConvDim, int iConvAddPointNumRef, float fConvFusionDistanceRef1) :
	m_iKeepTime(iKeepTime), m_iConvDim(iConvDim), m_iConvAddPointNumRef(iConvAddPointNumRef), m_fConvFusionDistanceRef1(fConvFusionDistanceRef1) 
{ 
	m_iConvHalfDim = m_iConvDim / 2;
};

void SignedDistance::BuildUnionSet(VolumeBase & oVoxeler, UnionSet& oUnionSet) {

	constexpr int xyz_order[][4] = {{0, 1, 4, 5}, {0, 3, 4, 7}, {0, 1, 2, 3}};
	constexpr int xyz_delta[][3] = {{-1, 0, 0}, {0, -1, 0}, {0, 0, -1}};

	for(auto && [oPos, oPoint] : m_vVolumeCopy) {

		std::vector<HashPos> vCornerPoses;
		oVoxeler.GetCornerPoses(oPos, vCornerPoses);
		for(int dim = 0; dim < 3; ++dim) {
			for(int i = 0; i < 3; ++i) {
				if(m_vSignedDistance[vCornerPoses[xyz_order[dim][i]]] * m_vSignedDistance[vCornerPoses[xyz_order[dim][i+1]]] < 0) {
					HashPos oNearPos(oPos.x + xyz_delta[dim][0], oPos.y + xyz_delta[dim][1], oPos.z + xyz_delta[dim][2]);
					if(m_vVolumeCopy.count(oNearPos)) oUnionSet.Union(oPos, oNearPos);
					break;
				}
			}	
		}
	}
}

std::unordered_map<HashPos, float, HashFunc> & SignedDistance::DebugGlance(VolumeBase & oVoxeler) {

	oVoxeler.GetRecentAllVolume(m_vVolumeCopy, m_iKeepTime);
	
	// meshing
	const VolumeBase::HashVolume & vVolumeToMeshing = m_vVolumeCopy;

	//compute the sampled point normal based on the face normal
	ConvexHullOperation oConvexHullOPer;

	std::unordered_map<HashPos, FacePara, HashFunc> vVoxelNormalPara;

	//compute the sampled point normal and divde it into point and normal
	oConvexHullOPer.ComputeAllFaceParams(vVolumeToMeshing, vVoxelNormalPara);

	//output
	return PlanDistance(vVolumeToMeshing, vVoxelNormalPara, oVoxeler.GetVoxelLength());
}

std::unordered_map<HashPos, float, HashFunc> & SignedDistance::NormalBasedGlance(VolumeBase & oVoxeler) {

	//****get the nodes that are near the surface****
	VolumeBase::HashVolume vTempVolumeCopy;
	oVoxeler.GetRecentVolume(vTempVolumeCopy, m_iKeepTime);
	// oVoxeler.GetRecentNoneFlowVolume(vTempVolumeCopy, m_iKeepTime);
	// oVoxeler.GetRecentHighDistributionVolume(vTempVolumeCopy, m_iKeepTime);
	m_vVolumeCopy = vTempVolumeCopy;

	// meshing
	const VolumeBase::HashVolume & vVolumeToMeshing = m_vVolumeCopy;

	//compute the sampled point normal based on the face normal
	ConvexHullOperation oConvexHullOPer;

	std::unordered_map<HashPos, FacePara, HashFunc> vVoxelNormalPara;

	//compute the sampled point normal and divde it into point and normal
	oConvexHullOPer.ComputeAllFaceParams(vVolumeToMeshing, vVoxelNormalPara);

	//output
	return PlanDistance(vVolumeToMeshing, vVoxelNormalPara, oVoxeler.GetVoxelLength());
}

std::unordered_map<HashPos, float, HashFunc> & SignedDistance::ConvedGlance(VolumeBase & oVoxeler, visualization_msgs::MarkerArray* oDebugMarker) {
	
	VolumeBase::HashVolume vTempVolumeCopy;
	oVoxeler.GetRecentVolume(vTempVolumeCopy, m_iKeepTime);
	CopyAndExpandVolume(vTempVolumeCopy);

	if(oDebugMarker != nullptr) 
		oVoxeler.DrawVolume(vTempVolumeCopy, *oDebugMarker);

	return ConvedGlanceCore(vTempVolumeCopy, oVoxeler.GetVoxelLength());
}

std::unordered_map<HashPos, float, HashFunc> & SignedDistance::ConvedGlanceNoneFlow(VolumeBase & oVoxeler, visualization_msgs::MarkerArray* oDebugMarker) {

	VolumeBase::HashVolume vTempVolumeCopy;
	oVoxeler.GetRecentNoneFlowVolume(vTempVolumeCopy, m_iKeepTime);
	CopyAndExpandVolume(vTempVolumeCopy);

	if(oDebugMarker != nullptr) 
		oVoxeler.DrawVolume(vTempVolumeCopy, *oDebugMarker);

	return ConvedGlanceCore(vTempVolumeCopy, oVoxeler.GetVoxelLength());
}

std::unordered_map<HashPos, float, HashFunc> & SignedDistance::ConvedGlanceLargeUnion(VolumeBase & oVoxeler, const int iRemoveSizeRef, visualization_msgs::MarkerArray* oDebugMarker) {

	VolumeBase::HashVolume vTempVolumeCopy;
	oVoxeler.GetRecentConnectVolume(vTempVolumeCopy, m_iKeepTime, iRemoveSizeRef);
	CopyAndExpandVolume(vTempVolumeCopy);

	if(oDebugMarker != nullptr) 
		oVoxeler.DrawVolume(vTempVolumeCopy, *oDebugMarker);

	return ConvedGlanceCore(vTempVolumeCopy, oVoxeler.GetVoxelLength());
}

std::unordered_map<HashPos, float, HashFunc> & SignedDistance::ConvedGlanceAllUnion(VolumeBase & oVoxeler, const int iRemoveSizeRef) {

	VolumeBase::HashVolume vTempVolumeCopy;
	oVoxeler.GetAllConnectVolume(vTempVolumeCopy, iRemoveSizeRef);
	CopyAndExpandVolume(vTempVolumeCopy);
	return ConvedGlanceCore(vTempVolumeCopy, oVoxeler.GetVoxelLength());
}

std::unordered_map<HashPos, float, HashFunc> & SignedDistance::ConvedGlanceAll(VolumeBase & oVoxeler) {

	VolumeBase::HashVolume vTempVolumeCopy;
	oVoxeler.GetAllVolume(vTempVolumeCopy);
	m_vVolumeCopy = vTempVolumeCopy;
	return ConvedGlanceCore(vTempVolumeCopy, oVoxeler.GetVoxelLength());
}

std::unordered_map<HashPos, float, HashFunc> & SignedDistance::ConvedGlanceOnlyMaxUnion(VolumeBase & oVoxeler, visualization_msgs::MarkerArray* oDebugMarker) {
	
	VolumeBase::HashVolume vTempVolumeCopy;
	oVoxeler.GetRecentMaxConnectVolume(vTempVolumeCopy, m_iKeepTime);
	CopyAndExpandVolume(vTempVolumeCopy);

	if(oDebugMarker != nullptr) 
		oVoxeler.DrawVolume(vTempVolumeCopy, *oDebugMarker);

	return ConvedGlanceCore(vTempVolumeCopy, oVoxeler.GetVoxelLength());
}

std::unordered_map<HashPos, float, HashFunc> & SignedDistance::CenterBasedGlance(VolumeBase & oVoxeler, const Eigen::Vector3f vCenter, const float fRadius, const int iRemoveSizeRef, visualization_msgs::MarkerArray* oDebugMarker) {

	VolumeBase::HashVolume vTempVolumeCopy;
	oVoxeler.GetLocalConnectVolume(vTempVolumeCopy, vCenter, fRadius, iRemoveSizeRef);
	CopyAndExpandVolume(vTempVolumeCopy);

	if(oDebugMarker != nullptr) 
		oVoxeler.DrawVolume(vTempVolumeCopy, *oDebugMarker);

	return ConvedGlanceCore(vTempVolumeCopy, oVoxeler.GetVoxelLength());
}

void SignedDistance::CopyAndExpandVolume(VolumeBase::HashVolume& vTempVolumeCopy) {

	m_vVolumeCopy = vTempVolumeCopy;

	// /* expand - not that good, only fill small holes, but runs too slow(if 3d expand)
	for(auto && [oPos,oPoint] : m_vVolumeCopy) {

		// /* judge normal main direction to speed up
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
		//*/
	 
		for(int dx = start_x; dx <= end_x; ++dx) {
		for(int dy = start_y; dy <= end_y; ++dy) {
		for(int dz = start_z; dz <= end_z; ++dz) {
		// for(int dx = -m_iConvHalfDim; dx <= m_iConvHalfDim; ++dx) {
		// for(int dy = -m_iConvHalfDim; dy <= m_iConvHalfDim; ++dy) {
		// for(int dz = -m_iConvHalfDim; dz <= m_iConvHalfDim; ++dz) {

			const HashPos oCurrentPos(oPos.x + dx, oPos.y + dy, oPos.z + dz);
			if(!vTempVolumeCopy.count(oCurrentPos)) {
				vTempVolumeCopy[oCurrentPos] = pcl::PointNormal();
				vTempVolumeCopy[oCurrentPos].data_c[2] = 0;
			}
		}}}
	}
	//*/
}

std::unordered_map<HashPos, float, HashFunc> & SignedDistance::ConvedGlanceCore(VolumeBase::HashVolume& vTempVolumeCopy, const Eigen::Vector3f& oVoxelSize) {

	// /*
	clock_t start_time, conv_time;

	// multi-resolution by convolution method
	start_time = clock();
	for(auto && [oPos,_] : vTempVolumeCopy) {

		pcl::PointNormal oFusedNormal;
		float & normal_distribution_distance = oFusedNormal.data_c[3];
		float & all_weight = oFusedNormal.data_n[3];

		// start conv
		int point_count = 0;
		for(int dz = -m_iConvHalfDim; dz <= m_iConvHalfDim; ++dz) {
			for(int dy = -m_iConvHalfDim; dy <= m_iConvHalfDim; ++dy) {
				for(int dx = -m_iConvHalfDim; dx <= m_iConvHalfDim; ++dx) {

					const HashPos oCurrentPos(oPos.x + dx, oPos.y + dy, oPos.z + dz);

					if(vTempVolumeCopy.count(oCurrentPos) && vTempVolumeCopy[oCurrentPos].data_c[2] /*not added point*/) {

						const pcl::PointNormal& oVoxelNormal = vTempVolumeCopy[oCurrentPos];
						const float& fCurrentWeight = oVoxelNormal.data_n[3];
						if(fCurrentWeight == 0) continue;

						// oFusedNormal.x += oVoxelNormal.x;
						// oFusedNormal.y += oVoxelNormal.y;
						// oFusedNormal.z += oVoxelNormal.z;
						oFusedNormal.x += fCurrentWeight * oVoxelNormal.x;
						oFusedNormal.y += fCurrentWeight * oVoxelNormal.y;
						oFusedNormal.z += fCurrentWeight * oVoxelNormal.z;
						++point_count;

						oFusedNormal.normal_x += fCurrentWeight * oVoxelNormal.normal_x;
						oFusedNormal.normal_y += fCurrentWeight * oVoxelNormal.normal_y;
						oFusedNormal.normal_z += fCurrentWeight * oVoxelNormal.normal_z;
						all_weight += fCurrentWeight;
					}
				}
			}
		} // end conv

		//TODO: then
		if(all_weight > 0) {

			// oFusedNormal.x /= point_count;
			// oFusedNormal.y /= point_count;
			// oFusedNormal.z /= point_count;
			oFusedNormal.x /= all_weight;
			oFusedNormal.y /= all_weight;
			oFusedNormal.z /= all_weight;
			oFusedNormal.normal_x /= all_weight;
			oFusedNormal.normal_y /= all_weight;
			oFusedNormal.normal_z /= all_weight;

			normal_distribution_distance = sqrt(oFusedNormal.normal_x * oFusedNormal.normal_x + oFusedNormal.normal_y * oFusedNormal.normal_y + oFusedNormal.normal_z * oFusedNormal.normal_z);
			// std::cout << "count: " << point_count << "\tweight: " << all_weight << "\tdistance: " << normal_distribution_distance << std::endl;
			//经过观察发现 all_weight 和 normal_distribution_disance 之间的关系也能得出一些信息： 如果 all_weight 较小，normal_distribution_distance = 1 则说明该4x4的区域只有一个点，很有可能是噪点

			// use min level to update the normal
			const HashPos oCurrentPos(oPos.x, oPos.y, oPos.z);
			// add point 
			if(!m_vVolumeCopy.count(oCurrentPos)) {
				
				if(point_count < m_iConvAddPointNumRef) continue;

				pcl::PointNormal oVoxelNormal;
				oVoxelNormal.x = oFusedNormal.x;
				oVoxelNormal.y = oFusedNormal.y;
				oVoxelNormal.z = oFusedNormal.z;
				oVoxelNormal.data_c[1] = __INT_MAX__; /*means the point is added*/
				oVoxelNormal.data_c[3] = normal_distribution_distance;
				oVoxelNormal.normal_x = oFusedNormal.normal_x;
				oVoxelNormal.normal_y = oFusedNormal.normal_y;
				oVoxelNormal.normal_z = oFusedNormal.normal_z;
				oVoxelNormal.data_n[3] = all_weight; 

				Eigen::Vector3f oCorner;
				VolumeBase::HashPosTo3DPos(oCurrentPos, oVoxelSize, oCorner);
				oCorner += oVoxelSize * 0.5;
				Eigen::Vector3f vPoint = oVoxelNormal.getVector3fMap();
				Eigen::Vector3f vNormal = oVoxelNormal.getNormalVector3fMap();
				if(abs(vNormal.dot(oCorner - vPoint)) < oVoxelSize.norm() * 0.5)
					m_vVolumeCopy[oCurrentPos] = oVoxelNormal;
			}
			else if(normal_distribution_distance > m_fConvFusionDistanceRef1) {

				// copy fused points to octree neighbors
				if(point_count < 3) {
					m_vVolumeCopy.erase(oCurrentPos);
				}
				else {
					
					uint32_t voxel_token = 0x3fffffff; /*means the point is fused*/

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
		}

	}
	conv_time = 1000.0 * (clock() - start_time) / CLOCKS_PER_SEC;
	// std::cout << "conv_time: " << conv_time << "ms" << std::endl;
	//*/
	

/** XXX: Output codes
	std::vector<float> vPointLabels(pCloudNormals->points.size(), 0.0);
	for (int i = 0; i != oVoxeler.m_vVoxelPointIdx.size(); ++i){
		for (int j = 0; j != oVoxeler.m_vVoxelPointIdx[i].size(); ++j){
			int iPointId = oVoxeler.m_vVoxelPointIdx[i][j];
			vPointLabels[iPointId] = float(i);
		}
	}
	WritePointCloudTxt("PointLabels.txt", *pCloudNormals, vPointLabels);
//*/

	// meshing
	const VolumeBase::HashVolume & vVolumeToMeshing = m_vVolumeCopy;

	//compute the sampled point normal based on the face normal
	ConvexHullOperation oConvexHullOPer;

	std::unordered_map<HashPos, FacePara, HashFunc> vVoxelNormalPara;

	//compute the sampled point normal and divde it into point and normal
	oConvexHullOPer.ComputeAllFaceParams(vVolumeToMeshing, vVoxelNormalPara);

	//output
	return PlanDistance(vVolumeToMeshing, vVoxelNormalPara, oVoxelSize);

}

unordered_map<HashPos, float, HashFunc> & SignedDistance::PlanDistance(const VolumeBase::HashVolume & vVolume, const unordered_map<HashPos, FacePara, HashFunc> & vNormalPara, const Eigen::Vector3f& oVoxelSize) {

	m_vSignedDistance.clear();
	unordered_map<HashPos, int, HashFunc> vCornerHits;

	ConvexHullOperation oNormalOper;

	for (auto && [oPos,_] : vVolume) {
		
		std::vector<HashPos> vCornerPoses;
		VolumeBase::GetCornerPoses(oPos, vCornerPoses);

		for(auto && oCornerPos : vCornerPoses) {
			
			Eigen::Vector3f oOneCorner;
			VolumeBase::HashPosTo3DPos(oCornerPos, oVoxelSize, oOneCorner);
			float fSignValue = oNormalOper.PointToFaceDis(oOneCorner, vNormalPara.at(oPos).oNormal, vNormalPara.at(oPos).fDparam);
			m_vSignedDistance[oCornerPos] += fSignValue;
			++vCornerHits[oCornerPos];
		}
	}

	for (auto & [oPos, oSignedDis] : m_vSignedDistance) 
		if(vCornerHits.count(oPos))
			oSignedDis /= vCornerHits[oPos];

	return m_vSignedDistance;
}