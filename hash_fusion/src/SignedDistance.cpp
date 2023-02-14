#include"SignedDistance.h"

std::unordered_map<HashPos, float, HashFunc> & SignedDistance::NormalBasedGlance(pcl::PointCloud<pcl::PointNormal>::Ptr & pCloudNormals, HashVoxeler & oVoxeler){


	//****get the nodes that are near the surface****
	oVoxeler.VoxelizePointsAndFusion(*pCloudNormals); 
	m_vVolumeCopy = oVoxeler.m_vVolume;

	// /*
	clock_t start_time, conv_time;

	// multi-resolution by convolution method
	constexpr int conv_dim = 3;
	constexpr int conv_half_dim = conv_dim / 2;
	constexpr int conv_add_point_num_ref = 5;
	constexpr float conv_fusion_distance_ref1 = 0.95f;

	start_time = clock();
	for(auto && [oPos, _] : oVoxeler.m_vVolume) {

		pcl::PointNormal oFusedNormal;
		float & normal_distribution_distance = oFusedNormal.data_c[3];
		float & all_weight = oFusedNormal.data_n[3];

		// start conv
		int point_count = 0;
		for(int dz = -conv_half_dim; dz <= conv_half_dim; ++dz) {
			for(int dy = -conv_half_dim; dy <= conv_half_dim; ++dy) {
				for(int dx = -conv_half_dim; dx <= conv_half_dim; ++dx) {

					const HashPos oCurrentPos(oPos.x + dx, oPos.y + dy, oPos.z + dz);

					if(oVoxeler.m_vVolume.count(oCurrentPos)) {

						const pcl::PointNormal& oVoxelNormal = oVoxeler.m_vVolume[oCurrentPos];
						const float& fCurrentWeight = oVoxelNormal.data_n[3];

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
			if(normal_distribution_distance > conv_fusion_distance_ref1) {

				const HashPos oCurrentPos(oPos.x, oPos.y, oPos.z);

				// copy fused points to octree neighbors
				if(m_vVolumeCopy.count(oCurrentPos) || point_count >= conv_add_point_num_ref) {
					
					pcl::PointNormal& oVoxelNormal = m_vVolumeCopy[oCurrentPos];
					// pcl::PointNormal& oVoxelNormal = vCompareCopy.at(current_index);
					oVoxelNormal.x = oFusedNormal.x;
					oVoxelNormal.y = oFusedNormal.y;
					oVoxelNormal.z = oFusedNormal.z;
					oVoxelNormal.data_c[3] = normal_distribution_distance;
					oVoxelNormal.normal_x = oFusedNormal.normal_x;
					oVoxelNormal.normal_y = oFusedNormal.normal_y;
					oVoxelNormal.normal_z = oFusedNormal.normal_z;
					oVoxelNormal.data_n[3] = all_weight;
				}
				else if(point_count <= 3) {
					m_vVolumeCopy.erase(oCurrentPos);
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
	const HashVoxeler::HashVolume & vVolumeToMeshing = m_vVolumeCopy;

	//compute the sampled point normal based on the face normal
	ConvexHullOperation oConvexHullOPer;

	std::unordered_map<HashPos, FacePara, HashFunc> vVoxelNormalPara;

	//compute the sampled point normal and divde it into point and normal
	oConvexHullOPer.ComputeAllFaceParams(vVolumeToMeshing, vVoxelNormalPara);

	//output
	return PlanDistance(vVolumeToMeshing, vVoxelNormalPara, oVoxeler.m_oVoxelLength);

}

unordered_map<HashPos, float, HashFunc> & SignedDistance::PlanDistance(const HashVoxeler::HashVolume & vVolume, const unordered_map<HashPos, FacePara, HashFunc> & vNormalPara, const pcl::PointXYZ oVoxelSize) {

	unordered_map<HashPos, float, HashFunc> & vSignedDis = m_vSignedDistance;
	unordered_map<HashPos, int, HashFunc> vCornerHits;

	ConvexHullOperation oNormalOper;

	for (auto && [oPos,_] : vVolume) {
		
		std::vector<HashPos> vCornerPoses;
		HashVoxeler::GetCornerPoses(oPos, vCornerPoses);

		for(auto && oCornerPos : vCornerPoses) {
			
			Eigen::Vector3f oOneCorner;
			HashVoxeler::HashPosTo3DPos(oCornerPos, oVoxelSize, oOneCorner);
			float fSignValue = oNormalOper.PointToFaceDis(oOneCorner, vNormalPara.at(oPos).oNormal, vNormalPara.at(oPos).fDparam);
			vSignedDis[oCornerPos] += fSignValue;
			++vCornerHits[oCornerPos];
		}
	}

	for (auto & [oPos, oSignedDis] : vSignedDis) 
		if(vCornerHits.count(oPos))
			oSignedDis /= vCornerHits[oPos];

	return vSignedDis;
}