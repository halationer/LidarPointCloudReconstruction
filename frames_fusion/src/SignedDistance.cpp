#include"SignedDistance.h"
#include"Cell.h"

/*=======================================
SamplePoints
Input: vCloud - the raw point clouds
    vNewCloud - the sampled point clouds
	iSampleNum - interval number or the maximum number
	bIntervalSamp - interval number model (true) or the maximum number model (false)
Output: vNewCloud - the sampled point clouds
Function: Sample a point clouds. Note that tthe given maximum total number guarantes the number of sampled point is smaller than it
However, it is not guaranteed that the number of sampled point could be equal to the given maximum total number
========================================*/
void SamplePoints(const pcl::PointCloud<pcl::PointXYZ> & vCloud, pcl::PointCloud<pcl::PointXYZ> & vNewCloud, int iSampleNum, bool bIntervalSamp){

	vNewCloud.clear();

	//sample by interval number
	if (bIntervalSamp){

		for (int i = 0; i < vCloud.points.size(); i = i + iSampleNum)
			vNewCloud.push_back(vCloud.points[i]);
		
	//over the function and output	
		return ;
	
	}//end if

	//Sampling according to the given maximum total number
	//get the interval point number - how muny points should be skipped
	int iInterval = ceil(float(vCloud.points.size()) / float(iSampleNum));
	//sample
	for (int i = 0; i < vCloud.points.size(); i = i + iInterval)
		vNewCloud.push_back(vCloud.points[i]);

	//output
	return ;

}


/*=======================================
ConvertCloud
Input: vSCloud - source point clouds
Output: vTCloud - target point clouds,also the transformed point cloud
Function: it converts points from world into local coordiante based on viewpoint
========================================*/
std::vector<float> SignedDistance::ConvexBasedGlance(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pRawCloud,
	                                const pcl::PointXYZ & oViewPoint,
	                                const pcl::PointCloud<pcl::PointXYZ>::Ptr & m_pVoxelCorner){

	GHPR oGHPRer(oViewPoint, m_GHPRParam);

	//compute the occlusion
	oGHPRer.Compute(pRawCloud);

	//get the face index
	m_vGlanceFaceIdxs = oGHPRer.GetConvexHullWorldIdx();

	//get the surface
	m_vSurfaceIdxs = oGHPRer.ConstructSurfaceIdx();

	pcl::PointCloud<pcl::PointXYZ>::Ptr pHullWorldVert(new pcl::PointCloud<pcl::PointXYZ>);//Ô­Ê¼pcdµãÔÆ
	for (int i = 0; i != pRawCloud->points.size(); ++i)
		pHullWorldVert->points.push_back(pRawCloud->points[i]);
	pHullWorldVert->points.push_back(oViewPoint);

	//compute parameters of each face
	ConvexHullOperation oConvexHullOPer;

	//compute face in two different space, respectively 
	oConvexHullOPer.FaceParamInTwoSpace(*oGHPRer.m_pHullVertices, oGHPRer.m_vHullPolygonIdxs,
		*pHullWorldVert, m_vGlanceFaceIdxs);
	//oConvexHullOPer.ComputeAllFaceParams(oViewPoint, *pHullWorldVert, m_vGlanceFaceIdxs,
	//	oConvexHullOPer.m_oEuclidMatN, oConvexHullOPer.m_vEuclidD);

	std::cout << "the number of faces after GHPR: " << m_vGlanceFaceIdxs.size() << std::endl;

	//compute the face parameters
	std::vector<FacePara> vEuclidFaces;
	oConvexHullOPer.NDToFacePara(oConvexHullOPer.m_oEuclidMatN, oConvexHullOPer.m_vEuclidD, vEuclidFaces);

	//convert the voxel corners
	pcl::PointCloud<pcl::PointXYZ>::Ptr pConvertedVoxelCs(new pcl::PointCloud<pcl::PointXYZ>);
	oGHPRer.ConvertCloud(*m_pVoxelCorner, *pConvertedVoxelCs);

	//compute indicator function notation
	std::vector<SignedDis> vTransedSDis = oConvexHullOPer.ComputePointSignedDis(*pConvertedVoxelCs);

	//compute the precise signed distance
	std::vector<float> vCornerSignedDis = oConvexHullOPer.ComputePointSignedDis(vTransedSDis, *m_pVoxelCorner, vEuclidFaces);
	//std::vector<float> vCornerSignedDis = oConvexHullOPer.ComputePointSignedDis(*m_pVoxelCorner, oConvexHullOPer.m_oEuclidMatN, oConvexHullOPer.m_vEuclidD);

	//test for 
	//std::vector<float> vCornerSignedDis;
	//for (int i = 0; i != vTransedSDis.size(); ++i){
	//	if (vTransedSDis[i].bInner)
	//		vCornerSignedDis.push_back(-1);
	//	else
	//		vCornerSignedDis.push_back(1);
	//}

	return vCornerSignedDis;

}


/*=======================================
ConvertCloud
Input: vSCloud - source point clouds
Output: vTCloud - target point clouds,also the transformed point cloud
Function: it converts points from world into local coordiante based on viewpoint
========================================*/
std::vector<float> SignedDistance::RaycastBasedGlance(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pRawCloud,
	const pcl::PointXYZ & oViewPoint,
	const pcl::PointCloud<pcl::PointXYZ>::Ptr & m_pVoxelCorner){

	GHPR oGHPRer(oViewPoint, m_GHPRParam);

	//compute the occlusion
	oGHPRer.Compute(pRawCloud);

	//get the face index
	m_vGlanceFaceIdxs = oGHPRer.GetConvexHullWorldIdx();

	//get the surface
	m_vSurfaceIdxs = oGHPRer.ConstructSurfaceIdx();

	//**data output
	pcl::PolygonMesh MeshModel;
	pcl::toPCLPointCloud2(*pRawCloud, MeshModel.cloud);
	MeshModel.polygons = m_vSurfaceIdxs;
	pcl::io::savePLYFileBinary("mesh_res.ply", MeshModel);

	//sample the mesh to points
	pcl::PointCloud<pcl::PointXYZ>::Ptr pZeroCrossCloud(new pcl::PointCloud<pcl::PointXYZ>);
	MeshSample oMeshSampler;
	oMeshSampler.SetSamplePointNum(100000);
	oMeshSampler.SampleMeshToPoints(MeshModel);
	oMeshSampler.OutputSampledClouds(*pZeroCrossCloud);

	WritePointCloudTxt("SampledClouds.txt", *pZeroCrossCloud);

	//compute the sectors
	Sector oSectorGrids;
	oSectorGrids.SetResolution(PI / 3600.0f, PI / 1800.0f);
	oSectorGrids.SetViewPoint(oViewPoint);
	oSectorGrids.AssignPoints(*pZeroCrossCloud);

	std::vector<int> vCloudGridLabels;
	oSectorGrids.CloudGridIdx(vCloudGridLabels);
	WritePointCloudTxt("PointSectorLabel.txt", *pZeroCrossCloud, vCloudGridLabels);



	std::vector<float> vCloudGridDis;
	oSectorGrids.CloudGridDis(vCloudGridDis);
	WritePointCloudTxt("PointtoViewDis.txt", *pZeroCrossCloud, vCloudGridDis);

	std::vector<float> vCornerSignedDis = oSectorGrids.Raycast(*pZeroCrossCloud, *m_pVoxelCorner);

	WritePointCloudTxt("NodeSectorLabel.txt", *m_pVoxelCorner, oSectorGrids.m_vNodeGridIdx);

	WritePointCloudTxt("NodeSignedDistance.txt", *m_pVoxelCorner, vCornerSignedDis);

	//output
	return vCornerSignedDis;

}



//compute point based signed distance of one glance
//"point based" means the method is based on point operation 
std::vector<float> SignedDistance::PointBasedGlance(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pRawCloud,
	                                                const pcl::PointXYZ & oViewPoint,
													Voxelization & oVoxeler){


	//****compute the visiable mesh****
	GHPR oGHPRer(oViewPoint, m_GHPRParam);

	//compute the occlusion
	oGHPRer.Compute(pRawCloud);

	//get the face index
	m_vGlanceFaceIdxs = oGHPRer.GetConvexHullWorldIdx();

	//get the surface
	m_vSurfaceIdxs = oGHPRer.ConstructSurfaceIdx();

	//save visiable mesh data in ply file 
	pcl::PolygonMesh MeshModel;
	pcl::toPCLPointCloud2(*pRawCloud, MeshModel.cloud);
	MeshModel.polygons = m_vSurfaceIdxs;
	pcl::io::savePLYFileBinary("mesh_res.ply", MeshModel);

	//****samples the visiavle mesh to dense point clouds****
	//sample the mesh to points
	MeshSample oMeshSampler;
	oMeshSampler.SetSamplePointNum(100000);
	oMeshSampler.SampleMeshToPoints(MeshModel);	
	//oMeshSampler.m_pSampedCloud();

	//compute the sampled point normal based on the face normal
	ConvexHullOperation oConvexHullOPer;
	
	//data
	pcl::PointCloud<pcl::PointXYZ>::Ptr pZeroCrossCloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<FacePara> vNormalPara;
	//compute the sampled point normal and divde it into point and normal
	oConvexHullOPer.ComputeAllFaceParams(oViewPoint, *oMeshSampler.m_pSampedCloud, *pZeroCrossCloud, vNormalPara);
	WritePointCloudTxt("SampledClouds.txt", *pZeroCrossCloud);

	//****get the nodes that are near the surface****
	oVoxeler.VoxelizePoints(*pZeroCrossCloud, *pRawCloud);

	//****computes the signed distance for query nodes****
	//output mesh
	pcl::PointCloud<pcl::PointNormal>::Ptr pPNormals(new pcl::PointCloud<pcl::PointNormal>);
	oConvexHullOPer.ChangeNormalType(*pZeroCrossCloud, vNormalPara, *pPNormals);
	pcl::io::savePLYFileASCII("sampled_clouds.ply", *pPNormals);

	//compute distance
	std::vector<float> vCornerSignedDis = MinKDDSignedDis(pZeroCrossCloud, oVoxeler, vNormalPara);
	WritePointCloudTxt("NodeSignedDistance.txt", *oVoxeler.m_pCornerCloud, vCornerSignedDis);

	//prepare for next calculation
	oVoxeler.ClearMiddleData();

	//output
	return vCornerSignedDis;

}




std::vector<float> SignedDistance::NormalBasedGlance(pcl::PointCloud<pcl::PointNormal>::Ptr & pCloudNormals,
	                                                 Voxelization & oVoxeler){
	
	//
	

	Fusion oVoxelFusion;

	//****get the nodes that are near the surface****
	oVoxeler.VoxelizePoints(*pCloudNormals); // 在 oVoxeler 中 m_vVoxelPointIdx 记录了在每个 voxel 中的点的索引

	oVoxeler.m_pVoxelNormals->clear();
	oVoxeler.m_pVoxelNormals->resize(oVoxeler.m_vVoxelPointIdx.size());

	for (int i = 0; i != oVoxeler.m_vVoxelPointIdx.size(); ++i){
		
		// pcl::PointNormal oOneVoxelN = oVoxelFusion.NormalFusion(oVoxeler.m_vVoxelPointIdx[i], *pCloudNormals);
		pcl::PointNormal oOneVoxelN = oVoxelFusion.NormalFusionWeighted(oVoxeler.m_vVoxelPointIdx[i], *pCloudNormals);
		oVoxeler.m_pVoxelNormals->points[i] = oOneVoxelN; // 在 oVoxeler 中的 m_pVoxelNormals 记录了每个 voxel 中融合后的向量
	
	}

	clock_t start_time, conv_time;

	/*TODO: 类八叉树写法
	constexpr int min_level = -2;
	constexpr int max_level = 0;
	constexpr int min_level_len = 1 << (max_level - min_level);
	constexpr int add_point_num_ref = 5;
	constexpr float fusion_distance_ref1 = 0.95f;

	// for per min level voxel
	for (int x = 0; x < oVoxeler.m_iFinalVoxelNum.ixnum; x += min_level_len) {
		for (int y = 0; y < oVoxeler.m_iFinalVoxelNum.iynum; y += min_level_len) {
			for (int z = 0; z < oVoxeler.m_iFinalVoxelNum.iznum; z += min_level_len) {
				
				pcl::PointNormal oFusedNormal;
				float& normal_distribution_distance = oFusedNormal.data_c[3];
				float& all_weight = oFusedNormal.data_n[3];

				int point_count = 0;

				// for each octree brothers of current pos
				for(int dx = 0; dx < min_level_len; ++dx) {
					for(int dy = 0; dy < min_level_len; ++dy) {
						for(int dz = 0; dz < min_level_len; ++dz) {

							const IndexinAxis current_pos = {x + dx, y + dy, z + dz};
							const int current_index = oVoxeler.Tran3DIdxTo1D(current_pos);

							if(!oVoxeler.OutOfBorder(current_pos) && oVoxeler.m_vVoxelPointIdx[current_index].size()) {

								const pcl::PointNormal& oVoxelNormal = oVoxeler.m_pVoxelNormals->at(current_index);
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
				}

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
					if(normal_distribution_distance > fusion_distance_ref1) {

						for(int dx = 0; dx < min_level_len; ++dx) {
							for(int dy = 0; dy < min_level_len; ++dy) {
								for(int dz = 0; dz < min_level_len; ++dz) {

									const IndexinAxis current_pos = {x + dx, y + dy, z + dz};

									if(!oVoxeler.OutOfBorder(current_pos)) {
										
										const int current_index = oVoxeler.Tran3DIdxTo1D(current_pos);

										// copy fused points to octree neighbors
										pcl::PointNormal& oVoxelNormal = oVoxeler.m_pVoxelNormals->at(current_index);
										oVoxelNormal.x = oFusedNormal.x;
										oVoxelNormal.y = oFusedNormal.y;
										oVoxelNormal.z = oFusedNormal.z;
										oVoxelNormal.data_c[3] = normal_distribution_distance;
										oVoxelNormal.normal_x = oFusedNormal.normal_x;
										oVoxelNormal.normal_y = oFusedNormal.normal_y;
										oVoxelNormal.normal_z = oFusedNormal.normal_z;
										oVoxelNormal.data_n[3] = all_weight;

										if(point_count >= add_point_num_ref && oVoxeler.m_vVoxelPointIdx[current_index].size() == 0) {
											// std::cout << "add index in no index voxel!!" << std::endl;
											oVoxeler.m_vVoxelPointIdx[current_index].push_back(-1);
										}
										else if(point_count == 1) {
											oVoxeler.m_vVoxelPointIdx[current_index].clear();
										}
									}
								}
							}
						}

					}
				}
			}
		}
	}
	//*/

	// /*TODO: 卷积写法,需要先复制一份空间用于查询，否则会出现叠加卷积错误
	// 因为要和简化版本对比，目前改成对比版本，需要对Compare数组进行去除才能真正起作用
	// pcl::PointCloud<pcl::PointNormal> vCompareCopy(*oVoxeler.m_pVoxelNormals);

	const pcl::PointCloud<pcl::PointNormal> vVoxelCopy(*oVoxeler.m_pVoxelNormals);
	constexpr int conv_dim = 3;
	constexpr int conv_half_dim = conv_dim / 2;
	constexpr int conv_add_point_num_ref = 5;
	constexpr float conv_fusion_distance_ref1 = 0.95f;

	// 噪点剔除（前置处理步骤）
	// start_time = clock();
	// constexpr int conv_expand_check_dim = conv_dim;
	// for (int z = 0; z < oVoxeler.m_iFinalVoxelNum.iznum; ++z) {
	// 	for (int y = 0; y < oVoxeler.m_iFinalVoxelNum.iynum; ++y) {
	// 		for (int x = 0; x < oVoxeler.m_iFinalVoxelNum.ixnum; ++x) {
				
	// 			pcl::PointNormal oFusedNormal;
	// 			float& normal_distribution_distance = oFusedNormal.data_c[3];
	// 			float& all_weight = oFusedNormal.data_n[3];

	// 			int point_count = 0;

	// 			// for each octree brothers of current pos
	// 			for(int dz = -conv_expand_check_dim; dz <= conv_expand_check_dim; ++dz) {
	// 				for(int dy = -conv_expand_check_dim; dy <= conv_expand_check_dim; ++dy) {
	// 					for(int dx = -conv_expand_check_dim; dx <= conv_expand_check_dim; ++dx) {

	// 						const IndexinAxis current_pos = {x + dx, y + dy, z + dz};
	// 						const int current_index = oVoxeler.Tran3DIdxTo1D(current_pos);

	// 						if(!oVoxeler.OutOfBorder(current_pos) && oVoxeler.m_vVoxelPointIdx[current_index].size() && oVoxeler.m_vVoxelPointIdx[current_index][0] >= 0) {

	// 							++point_count;
	// 						}
	// 					}
	// 				}
	// 			}
	// 			const IndexinAxis current_pos = {x, y, z};
	// 			const int current_index = oVoxeler.Tran3DIdxTo1D(current_pos);
	// 			if(point_count == 1) oVoxeler.m_vVoxelPointIdx[current_index].clear();
	// 		}
	// 	}
	// }
	// conv_time = 1000.0 * (clock() - start_time) / CLOCKS_PER_SEC;
	// std::cout << "pre_process_time: " << conv_time << "ms" << std::endl;


	start_time = clock();
	// for per min level voxel
	for (int z = 0; z < oVoxeler.m_iFinalVoxelNum.iznum; ++z) {
		for (int y = 0; y < oVoxeler.m_iFinalVoxelNum.iynum; ++y) {
			for (int x = 0; x < oVoxeler.m_iFinalVoxelNum.ixnum; ++x) {
				
				pcl::PointNormal oFusedNormal;
				float& normal_distribution_distance = oFusedNormal.data_c[3];
				float& all_weight = oFusedNormal.data_n[3];

				int point_count = 0;

				// for each octree brothers of current pos
				for(int dz = -conv_half_dim; dz <= conv_half_dim; ++dz) {
					for(int dy = -conv_half_dim; dy <= conv_half_dim; ++dy) {
						for(int dx = -conv_half_dim; dx <= conv_half_dim; ++dx) {

							const IndexinAxis current_pos = {x + dx, y + dy, z + dz};
							const int current_index = oVoxeler.Tran3DIdxTo1D(current_pos);

							if(!oVoxeler.OutOfBorder(current_pos) && oVoxeler.m_vVoxelPointIdx[current_index].size() && oVoxeler.m_vVoxelPointIdx[current_index][0] >= 0) {

								// 这里访问拷贝区的值，防止重叠卷积，在前面还要检查vVoxelPointIdx是否被修改过(这里是否考虑被视为噪点的剔除点呢？是否可以采用一个两遍的策略先剔除噪点，再进行卷积？)
								const pcl::PointNormal& oVoxelNormal = vVoxelCopy.at(current_index);
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
				}

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

						const IndexinAxis current_pos = {x, y, z};
						const int current_index = oVoxeler.Tran3DIdxTo1D(current_pos);

						// copy fused points to octree neighbors
						pcl::PointNormal& oVoxelNormal = oVoxeler.m_pVoxelNormals->at(current_index);
						// pcl::PointNormal& oVoxelNormal = vCompareCopy.at(current_index);
						oVoxelNormal.x = oFusedNormal.x;
						oVoxelNormal.y = oFusedNormal.y;
						oVoxelNormal.z = oFusedNormal.z;
						oVoxelNormal.data_c[3] = normal_distribution_distance;
						oVoxelNormal.normal_x = oFusedNormal.normal_x;
						oVoxelNormal.normal_y = oFusedNormal.normal_y;
						oVoxelNormal.normal_z = oFusedNormal.normal_z;
						oVoxelNormal.data_n[3] = all_weight;

						// add additional voxel or remove outlier voxel
						if(point_count >= conv_add_point_num_ref && oVoxeler.m_vVoxelPointIdx[current_index].size() == 0) {
							// std::cout << "add index in no index voxel!!" << std::endl;
							oVoxeler.m_vVoxelPointIdx[current_index].push_back(-1);
						}
						else if(point_count == 1) {
							oVoxeler.m_vVoxelPointIdx[current_index].clear();
						}
					}
				}
			}
		}
	}

	conv_time = 1000.0 * (clock() - start_time) / CLOCKS_PER_SEC;
	// std::cout << "conv_time: " << conv_time << "ms" << std::endl;
	//*/
	

	/* TODO：卷积优化(可加快约30%，但有bug)
	start_time = clock();

	// const pcl::PointCloud<pcl::PointNormal> vVoxelCopy(*oVoxeler.m_pVoxelNormals);

	constexpr int center_dim = 2;
	constexpr int expand_half_dim = conv_half_dim + 1;
	// constexpr int conv_dim = 3;
	// constexpr int conv_half_dim = conv_dim / 2;
	// constexpr int conv_add_point_num_ref = 5;
	// constexpr float conv_fusion_distance_ref1 = 0.95f;

	auto FusePoint = [&vVoxelCopy](pcl::PointNormal& point, const int current_index, int& point_count, float& all_weight) {
		const pcl::PointNormal& oVoxelNormal = vVoxelCopy.at(current_index);
		const float& fCurrentWeight = oVoxelNormal.data_n[3];

		point.x += fCurrentWeight * oVoxelNormal.x;
		point.y += fCurrentWeight * oVoxelNormal.y;
		point.z += fCurrentWeight * oVoxelNormal.z;
		++point_count;

		point.normal_x += fCurrentWeight * oVoxelNormal.normal_x;
		point.normal_y += fCurrentWeight * oVoxelNormal.normal_y;
		point.normal_z += fCurrentWeight * oVoxelNormal.normal_z;
		all_weight += fCurrentWeight;
	};

	// for per min level voxel
	for (int z = 0; z < oVoxeler.m_iFinalVoxelNum.iznum; z+=center_dim) {
		for (int y = 0; y < oVoxeler.m_iFinalVoxelNum.iynum; y+=center_dim) {
			for (int x = 0; x < oVoxeler.m_iFinalVoxelNum.ixnum; x+=center_dim) {

				pcl::PointNormal oCenterFuse;
				int center_count = 0;
				float center_weight = .0f;

				std::vector<pcl::PointNormal> vEdgeFuse(12);
				std::vector<int> edge_count(12);
				std::vector<float> edge_weight(12);

				std::vector<pcl::PointNormal> vFaceFuse(6);
				std::vector<int> face_count(6);
				std::vector<float> face_weight(6);

				// quick conv fuse
				for(int dz = -conv_half_dim; dz <= expand_half_dim; ++dz) {
					for(int dy = -conv_half_dim; dy <= expand_half_dim; ++dy) {
						for(int dx = -conv_half_dim; dx <= expand_half_dim; ++dx) {

							const IndexinAxis current_pos = {x + dx, y + dy, z + dz};
							const int current_index = oVoxeler.Tran3DIdxTo1D(current_pos);

							if(!oVoxeler.OutOfBorder(current_pos) && oVoxeler.m_vVoxelPointIdx[current_index].size() && oVoxeler.m_vVoxelPointIdx[current_index][0] >= 0) {

								// center
								if(dx > -conv_half_dim && dx < expand_half_dim && dy > -conv_half_dim && dy < expand_half_dim && dz > -conv_half_dim && dz < expand_half_dim) {
									
									FusePoint(oCenterFuse, current_index, center_count, center_weight);
								}
								// 4 edges and 4 faces
								else if(dz > -conv_half_dim && dz < expand_half_dim){

									// 4 edges
									if(dx == -conv_half_dim && dy == -conv_half_dim) FusePoint(vEdgeFuse[0], current_index, edge_count[0], edge_weight[0]);
									else if(dx == -conv_half_dim && dy == expand_half_dim) FusePoint(vEdgeFuse[6], current_index, edge_count[6], edge_weight[6]);
									else if(dx == expand_half_dim && dy == -conv_half_dim) FusePoint(vEdgeFuse[8], current_index, edge_count[8], edge_weight[8]);
									else if(dx == expand_half_dim && dy == expand_half_dim) FusePoint(vEdgeFuse[11], current_index, edge_count[11], edge_weight[11]);

									// 4 faces
									else if(dx == -conv_half_dim && dy > -conv_half_dim && dy < expand_half_dim) FusePoint(vFaceFuse[0], current_index, face_count[0], face_weight[0]);
									else if(dx == expand_half_dim && dy > -conv_half_dim && dy < expand_half_dim) FusePoint(vFaceFuse[2], current_index, face_count[2], face_weight[2]);
									else if(dy == -conv_half_dim && dx > -conv_half_dim && dx < expand_half_dim) FusePoint(vFaceFuse[1], current_index, face_count[1], face_weight[1]);
									else if(dy == expand_half_dim && dx > -conv_half_dim && dx < expand_half_dim) FusePoint(vFaceFuse[4], current_index, face_count[4], face_weight[4]);
								}
								// 4 edges and 2 faces
								else if(dx > -conv_half_dim && dx < expand_half_dim) {

									// 4 edges
									if(dz == -conv_half_dim && dy == -conv_half_dim) FusePoint(vEdgeFuse[1], current_index, edge_count[1], edge_weight[1]);
									else if(dz == -conv_half_dim && dy == expand_half_dim) FusePoint(vEdgeFuse[5], current_index, edge_count[5], edge_weight[5]);
									else if(dz == expand_half_dim && dy == -conv_half_dim) FusePoint(vEdgeFuse[4], current_index, edge_count[4], edge_weight[4]);
									else if(dz == expand_half_dim && dy == expand_half_dim) FusePoint(vEdgeFuse[7], current_index, edge_count[7], edge_weight[7]);

									// 2 faces
									else if(dz == -conv_half_dim && dy > -conv_half_dim && dy < expand_half_dim) FusePoint(vFaceFuse[3], current_index, face_count[3], face_weight[3]);
									else if(dz == expand_half_dim && dy > -conv_half_dim && dy < expand_half_dim) FusePoint(vFaceFuse[5], current_index, face_count[5], face_weight[5]);
								}
								// 4 edges
								else if(dy > -conv_half_dim && dy < expand_half_dim) {

									if(dz == -conv_half_dim && dx == -conv_half_dim) FusePoint(vEdgeFuse[2], current_index, edge_count[2], edge_weight[2]);
									else if(dz == -conv_half_dim && dx == expand_half_dim) FusePoint(vEdgeFuse[9], current_index, edge_count[9], edge_weight[9]);
									else if(dz == expand_half_dim && dx == -conv_half_dim) FusePoint(vEdgeFuse[3], current_index, edge_count[3], edge_weight[3]);
									else if(dz == expand_half_dim && dx == expand_half_dim) FusePoint(vEdgeFuse[10], current_index, edge_count[10], edge_weight[10]);
								}
							}
						}
					}
				}

				std::vector<pcl::PointNormal> vFusedPoints(8);
				std::vector<int> fused_point_count(8);
				auto FuseBlocks = [&](
					pcl::PointNormal& point, int& point_count,
					const int edge_index_1, const int edge_index_2, const int edge_index_3, 
					const int face_index_1, const int face_index_2, const int face_index_3, 
					const int cdx, const int cdy, const int cdz) 
				{
					float& all_weight = point.data_n[3];

					// fuse center
					point.x += oCenterFuse.x;
					point.y += oCenterFuse.y;
					point.z += oCenterFuse.z;
					point_count += center_count;

					point.normal_x += oCenterFuse.normal_x;
					point.normal_y += oCenterFuse.normal_y;
					point.normal_z += oCenterFuse.normal_z;
					all_weight += center_weight;

					// fuse edge
					point.x += vEdgeFuse[edge_index_1].x + vEdgeFuse[edge_index_2].x + vEdgeFuse[edge_index_3].x;
					point.y += vEdgeFuse[edge_index_1].y + vEdgeFuse[edge_index_2].y + vEdgeFuse[edge_index_3].y;
					point.z += vEdgeFuse[edge_index_1].z + vEdgeFuse[edge_index_2].z + vEdgeFuse[edge_index_3].z;
					point_count += edge_count[edge_index_1] + edge_count[edge_index_2] + edge_count[edge_index_3];

					point.normal_x += vEdgeFuse[edge_index_1].normal_x + vEdgeFuse[edge_index_2].normal_x + vEdgeFuse[edge_index_3].normal_x;
					point.normal_y += vEdgeFuse[edge_index_1].normal_y + vEdgeFuse[edge_index_2].normal_y + vEdgeFuse[edge_index_3].normal_y;
					point.normal_z += vEdgeFuse[edge_index_1].normal_z + vEdgeFuse[edge_index_2].normal_z + vEdgeFuse[edge_index_3].normal_z;
					all_weight += edge_weight[edge_index_1] + edge_weight[edge_index_2] + edge_weight[edge_index_3];

					// fuse face
					point.x += vFaceFuse[face_index_1].x + vFaceFuse[face_index_2].x + vFaceFuse[face_index_3].x;
					point.y += vFaceFuse[face_index_1].y + vFaceFuse[face_index_2].y + vFaceFuse[face_index_3].y;
					point.z += vFaceFuse[face_index_1].z + vFaceFuse[face_index_2].z + vFaceFuse[face_index_3].z;
					point_count += face_count[face_index_1] + face_count[face_index_2] + face_count[face_index_3];

					point.normal_x += vFaceFuse[face_index_1].normal_x + vFaceFuse[face_index_2].normal_x + vFaceFuse[face_index_3].normal_x;
					point.normal_y += vFaceFuse[face_index_1].normal_y + vFaceFuse[face_index_2].normal_y + vFaceFuse[face_index_3].normal_y;
					point.normal_z += vFaceFuse[face_index_1].normal_z + vFaceFuse[face_index_2].normal_z + vFaceFuse[face_index_3].normal_z;
					all_weight += face_weight[face_index_1] + face_weight[face_index_2] + face_weight[face_index_3];

					// fuse corner
					const IndexinAxis current_pos = {x + cdx, y + cdy, z + cdz};
					const int current_index = oVoxeler.Tran3DIdxTo1D(current_pos);

					if(!oVoxeler.OutOfBorder(current_pos) && oVoxeler.m_vVoxelPointIdx[current_index].size() && oVoxeler.m_vVoxelPointIdx[current_index][0] >= 0) {
				
						const pcl::PointNormal& oVoxelNormal = vVoxelCopy.at(current_index);
						const float& fCurrentWeight = oVoxelNormal.data_n[3];

						point.x += fCurrentWeight * oVoxelNormal.x;
						point.y += fCurrentWeight * oVoxelNormal.y;
						point.z += fCurrentWeight * oVoxelNormal.z;
						++point_count;

						point.normal_x += fCurrentWeight * oVoxelNormal.normal_x;
						point.normal_y += fCurrentWeight * oVoxelNormal.normal_y;
						point.normal_z += fCurrentWeight * oVoxelNormal.normal_z;
						all_weight += fCurrentWeight;
					}
				};

				FuseBlocks(vFusedPoints[0], fused_point_count[0], 0,  1,  2, 0, 1, 3, -conv_half_dim,   -conv_half_dim,   -conv_half_dim  );
				FuseBlocks(vFusedPoints[1], fused_point_count[1], 1,  8,  9, 1, 2, 3,  expand_half_dim, -conv_half_dim,   -conv_half_dim  );
				FuseBlocks(vFusedPoints[2], fused_point_count[2], 2,  5,  6, 0, 1, 5, -conv_half_dim,    expand_half_dim, -conv_half_dim  );
				FuseBlocks(vFusedPoints[3], fused_point_count[3], 5,  9, 11, 1, 2, 5,  expand_half_dim,  expand_half_dim, -conv_half_dim  );
				FuseBlocks(vFusedPoints[4], fused_point_count[4], 0,  3,  4, 0, 3, 4, -conv_half_dim,   -conv_half_dim,    expand_half_dim);
				FuseBlocks(vFusedPoints[5], fused_point_count[5], 4,  8, 10, 2, 3, 4,  expand_half_dim, -conv_half_dim,    expand_half_dim);
				FuseBlocks(vFusedPoints[6], fused_point_count[6], 3,  7,  6, 0, 4, 5, -conv_half_dim,    expand_half_dim,  expand_half_dim);
				FuseBlocks(vFusedPoints[7], fused_point_count[7], 7, 10, 11, 2, 4, 5,  expand_half_dim,  expand_half_dim,  expand_half_dim);

				for(int i = 0; i < 8; ++i) {
					
					auto& oFusedNormal = vFusedPoints[i];
					int& point_count = fused_point_count[i];
					float& normal_distribution_distance = oFusedNormal.data_c[3];
					float& all_weight = oFusedNormal.data_n[3];

					if(all_weight > 0) {

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

							const int dx = i & 1, dy = (i >> 1) & 1, dz = (i >> 2) & 1;
							const IndexinAxis current_pos = {x + dx, y + dy, z + dz};
							const int current_index = oVoxeler.Tran3DIdxTo1D(current_pos);

							if(!oVoxeler.OutOfBorder(current_pos) && oVoxeler.m_vVoxelPointIdx[current_index].size() && oVoxeler.m_vVoxelPointIdx[current_index][0] >= 0) {
								// copy fused points to octree neighbors
								pcl::PointNormal& oVoxelNormal = oVoxeler.m_pVoxelNormals->at(current_index);
								oVoxelNormal.x = oFusedNormal.x;
								oVoxelNormal.y = oFusedNormal.y;
								oVoxelNormal.z = oFusedNormal.z;
								oVoxelNormal.data_c[3] = normal_distribution_distance;
								oVoxelNormal.normal_x = oFusedNormal.normal_x;
								oVoxelNormal.normal_y = oFusedNormal.normal_y;
								oVoxelNormal.normal_z = oFusedNormal.normal_z;
								oVoxelNormal.data_n[3] = all_weight;

								if(point_count >= conv_add_point_num_ref && oVoxeler.m_vVoxelPointIdx[current_index].size() == 0) {
									// std::cout << "add index in no index voxel!!" << std::endl;
									oVoxeler.m_vVoxelPointIdx[current_index].push_back(-1);
								}
								// else if(point_count == 1) {
								// 	oVoxeler.m_vVoxelPointIdx[current_index].clear();
								// }
							}
						}
					}
				}
				
			}
		}
	}

	conv_time = 1000.0 * (clock() - start_time) / CLOCKS_PER_SEC;
	std::cout << "advanced_conv_time: " << conv_time << "ms" << std::endl;

	//检验快速算法的正确性
	int error[8] = {0};
	int sum[8] = {0};
	for(int i = 0; i < vCompareCopy.size(); ++i) {

		if(oVoxeler.m_vVoxelPointIdx[i].size() && oVoxeler.m_vVoxelPointIdx[i][0] != -1) {
			
			auto coord = oVoxeler.Tran1DIdxTo3D(i);
			if((coord.iznum % 2) == 0 && (coord.iynum % 2) == 0 && (coord.ixnum % 2) == 0) ++sum[0];
			if((coord.iznum % 2) == 0 && (coord.iynum % 2) == 0 && (coord.ixnum % 2) == 1) ++sum[1];
			if((coord.iznum % 2) == 0 && (coord.iynum % 2) == 1 && (coord.ixnum % 2) == 0) ++sum[2];
			if((coord.iznum % 2) == 0 && (coord.iynum % 2) == 1 && (coord.ixnum % 2) == 1) ++sum[3];
			if((coord.iznum % 2) == 1 && (coord.iynum % 2) == 0 && (coord.ixnum % 2) == 0) ++sum[4];
			if((coord.iznum % 2) == 1 && (coord.iynum % 2) == 0 && (coord.ixnum % 2) == 1) ++sum[5];
			if((coord.iznum % 2) == 1 && (coord.iynum % 2) == 1 && (coord.ixnum % 2) == 0) ++sum[6];
			if((coord.iznum % 2) == 1 && (coord.iynum % 2) == 1 && (coord.ixnum % 2) == 1) ++sum[7];

			if( abs(vCompareCopy.at(i).x - oVoxeler.m_pVoxelNormals->at(i).x) < 1e-4 &&
				abs(vCompareCopy.at(i).y - oVoxeler.m_pVoxelNormals->at(i).y) < 1e-4 &&
				abs(vCompareCopy.at(i).z - oVoxeler.m_pVoxelNormals->at(i).z) < 1e-4)
			{
				// std::cout << vCompareCopy.at(i).x << " " << oVoxeler.m_pVoxelNormals->at(i).x << std::endl;
				continue;
			}
			
			// ++error;
			// std::cout << coord.ixnum << ",\t" << coord.iynum << ",\t" << coord.iznum << "\t" << vCompareCopy.at(i).x << " " << oVoxeler.m_pVoxelNormals->at(i).x << std::endl;
			if((coord.iznum % 2) == 0 && (coord.iynum % 2) == 0 && (coord.ixnum % 2) == 0) ++error[0];
			if((coord.iznum % 2) == 0 && (coord.iynum % 2) == 0 && (coord.ixnum % 2) == 1) ++error[1];
			if((coord.iznum % 2) == 0 && (coord.iynum % 2) == 1 && (coord.ixnum % 2) == 0) ++error[2];
			if((coord.iznum % 2) == 0 && (coord.iynum % 2) == 1 && (coord.ixnum % 2) == 1) ++error[3];
			if((coord.iznum % 2) == 1 && (coord.iynum % 2) == 0 && (coord.ixnum % 2) == 0) ++error[4];
			if((coord.iznum % 2) == 1 && (coord.iynum % 2) == 0 && (coord.ixnum % 2) == 1) ++error[5];
			if((coord.iznum % 2) == 1 && (coord.iynum % 2) == 1 && (coord.ixnum % 2) == 0) ++error[6];
			if((coord.iznum % 2) == 1 && (coord.iynum % 2) == 1 && (coord.ixnum % 2) == 1) ++error[7];
		}

	}
	std::cout << "error0: " << error[0] << "/" << sum[0] << " " << (double)error[0]/sum[0]*100 << "%" << std::endl;
	std::cout << "error1: " << error[1] << "/" << sum[1] << " " << (double)error[1]/sum[1]*100 << "%" << std::endl;
	std::cout << "error2: " << error[2] << "/" << sum[2] << " " << (double)error[2]/sum[2]*100 << "%" << std::endl;
	std::cout << "error3: " << error[3] << "/" << sum[3] << " " << (double)error[3]/sum[3]*100 << "%" << std::endl;
	std::cout << "error4: " << error[4] << "/" << sum[4] << " " << (double)error[4]/sum[4]*100 << "%" << std::endl;
	std::cout << "error5: " << error[5] << "/" << sum[5] << " " << (double)error[5]/sum[5]*100 << "%" << std::endl;
	std::cout << "error6: " << error[6] << "/" << sum[6] << " " << (double)error[6]/sum[6]*100 << "%" << std::endl;
	std::cout << "error7: " << error[7] << "/" << sum[7] << " " << (double)error[7]/sum[7]*100 << "%" << std::endl;
	std::cout << "total: " << vCompareCopy.size() << std::endl;

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

	//compute the sampled point normal based on the face normal
	ConvexHullOperation oConvexHullOPer;

	std::vector<FacePara> vVoxelNormalPara;


	//compute the sampled point normal and divde it into point and normal
	oConvexHullOPer.ComputeAllFaceParams(*oVoxeler.m_pVoxelNormals, vVoxelNormalPara);

	//from corner to plan distance
	std::vector<float> vCornerSignedDis = PlanDistance(oVoxeler, vVoxelNormalPara);

	// WritePointCloudTxt("NodeSignedDistance.txt", *oVoxeler.m_pCornerCloud, vCornerSignedDis);

	//output
	return vCornerSignedDis;

}



std::vector<float> SignedDistance::MinKDDis(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pCloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr & pQueryCloud){

	std::vector<float> vMinDis;

	//get the raw index using the shortest distance approach
	pcl::KdTreeFLANN<pcl::PointXYZ> oLocalKdtree;
	oLocalKdtree.setInputCloud(pCloud);

	//searching based on kdtree
	for (size_t i = 0; i != pQueryCloud->points.size(); i++){
		std::vector<int> kdindices;
		std::vector<float> kdistances;
		//find the first point as itself
		oLocalKdtree.nearestKSearch(pQueryCloud->points[i], 1, kdindices, kdistances);
		//at least one point can be found
		vMinDis.push_back(sqrt(kdistances[0]));
	}

	return vMinDis;

}



std::vector<float> SignedDistance::MinKDDSignedDis(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pCloud, Voxelization & oVoxeler,
	                                               const std::vector<FacePara> & vNormalPara){

	float fDefault = oVoxeler.m_fDefault;
	std::vector<float> vSignedDis(oVoxeler.m_pCornerCloud->points.size(), 1.0);

	//get the raw index using the shortest distance approach
	//generate a clouds for kdtree
	pcl::KdTreeFLANN<pcl::PointXYZ> oPKdtree;
	oPKdtree.setInputCloud(pCloud);

	ConvexHullOperation oNormalOper;
	
	//searching based on kdtree
	for (size_t i = 0; i != oVoxeler.m_pCornerCloud->points.size(); i++){
		//if the node is near the potential surface
		if (oVoxeler.m_vNearStatus[i]){
			//define eigen type point
			Eigen::Vector3f oRefPoint(oVoxeler.m_pCornerCloud->points[i].x, oVoxeler.m_pCornerCloud->points[i].y, oVoxeler.m_pCornerCloud->points[i].z);
			//get the nearest mesh sampled point for query node point
			std::vector<int> kdindices;
			std::vector<float> kdistances;
			//find the first point as itself
			oPKdtree.nearestKSearch(oVoxeler.m_pCornerCloud->points[i], 1, kdindices, kdistances);
			//get target point id
			int iTargetId = kdindices[0];
			//get the unsigned distance
			float fMinDis = sqrt(kdistances[0]);
			//get the sign value
			float fSignValue = oNormalOper.PointToFaceDis(oRefPoint, vNormalPara[iTargetId].oNormal, vNormalPara[iTargetId].fDparam);
			//get the result
			vSignedDis[i] = fSignValue*fMinDis;
		}

	}

	return vSignedDis;

}




std::vector<float> SignedDistance::PlanDistance(Voxelization & oVoxeler, const std::vector<FacePara> & vNormalPara){


	float fDefault = oVoxeler.m_fDefault;

	std::vector<float> vSignedDis(oVoxeler.m_pCornerCloud->points.size(), 0.0);
	std::vector<int> vCornerHits(oVoxeler.m_pCornerCloud->points.size(),0);

	ConvexHullOperation oNormalOper;

	for (int i = 0; i != oVoxeler.m_vVoxelPointIdx.size(); ++i){

		if (oVoxeler.m_vVoxelPointIdx[i].size()){

			//get the corners
			IndexinAxis oP3DIndex = oVoxeler.Tran1DIdxTo3D(i);

			std::vector<int> vCornerIdxs;
			oVoxeler.CornerIdxs(oP3DIndex, vCornerIdxs);

			//to each corner
			for (int j = 0; j != vCornerIdxs.size(); ++j){

				int iOneCornerIdx = vCornerIdxs[j];

				Eigen::Vector3f oOneCorner(oVoxeler.m_pCornerCloud->points[iOneCornerIdx].x,
					                       oVoxeler.m_pCornerCloud->points[iOneCornerIdx].y,
										   oVoxeler.m_pCornerCloud->points[iOneCornerIdx].z);

				float fSignValue = oNormalOper.PointToFaceDis(oOneCorner, vNormalPara[i].oNormal, vNormalPara[i].fDparam);

				vSignedDis[iOneCornerIdx] = vSignedDis[iOneCornerIdx] + fSignValue;

				vCornerHits[iOneCornerIdx] = vCornerHits[iOneCornerIdx] + 1;


			}//end  j != vCornerIdxs.size();

		}//end if

	}//end i != oVoxeler.m_vVoxelPointIdx.size()

	for (int i = 0; i != vSignedDis.size(); ++i){
		if (vCornerHits[i])
			vSignedDis[i] = vSignedDis[i] / float(vCornerHits[i]);
	}

	return vSignedDis;

}