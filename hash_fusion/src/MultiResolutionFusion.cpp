#include "MultiResolutionFusion.h"

void MultiResolutionFusion::LazyLoading() {

    std::cout << "Load Multi Resolution Fusion..." << std::endl;

    pcl::PointXYZ &oVoxelBaseSize = m_oVoxeler.m_oVoxelLength;
    pcl::PointXYZ oDownResolutionVoxelSize(oVoxelBaseSize.x * 4, oVoxelBaseSize.y * 4, oVoxelBaseSize.z * 4);
    m_oDownResolutionVoxeler.SetResolution(oDownResolutionVoxelSize);
    m_oDownResolutionVoxeler.GetStrategy(m_oVoxeler);
    m_oDownResolutionVoxeler.m_iMaxRecentKeep = m_oVoxeler.m_iMaxRecentKeep;
}


bool MultiResolutionFusion::ReadLaunchParams(ros::NodeHandle & nodeHandle) {

	nodeHandle.param("expand_distribution_ref", m_fExpandDistributionRef, 0.99f);
	m_oDownResolutionVoxeler.m_fExpandDistributionRef = m_fExpandDistributionRef;

	return true;
}


void MultiResolutionFusion::SlideModeling(pcl::PolygonMesh & oResultMesh, const int iFrameId) {

    Super::SlideModeling(oResultMesh, iFrameId);


	// TODO : according the flow to do something
    HashVoxeler::HashVolume vNoneFlowVolume;
    m_oDownResolutionVoxeler.GetRecentNoneFlowVolume(vNoneFlowVolume, m_iKeepTime);
    // m_oDownResolutionVoxeler.ClearFlow();
    
	//******make mesh********
    //replace mesh of bottom
    // pcl::PolygonMesh oDownResolutionMesh;
    pcl::PolygonMesh& oDownResolutionMesh = oResultMesh;
    oDownResolutionMesh.cloud.data.clear();
    oDownResolutionMesh.polygons.clear();

	//using signed distance
	SignedDistance oSDer(m_iKeepTime, m_iConvDim, m_iConvAddPointNumRef, m_fConvFusionDistanceRef1);
	//compute signed distance based on centroids and its normals within voxels
	std::unordered_map<HashPos, float, HashFunc> vSignedDis;
	vSignedDis = oSDer.NormalBasedGlance(m_oDownResolutionVoxeler);

	//marching cuber
	CIsoSurface<float> oMarchingCuber;
	oMarchingCuber.m_pGetAttrFunc = [=](const pcl::PointNormal& oPoint) { 
		return oPoint.data_c[3] >= this->m_fExpandDistributionRef && oPoint.data_c[0] < 0.3 ? 1.0f : 0.0f; 
	};
	//get surface mesh based on voxel related algorithm
	oMarchingCuber.GenerateSurface(vSignedDis, oSDer.m_vVolumeCopy, 0, 
        m_oDownResolutionVoxeler.m_oVoxelLength.x, m_oDownResolutionVoxeler.m_oVoxelLength.y, m_oDownResolutionVoxeler.m_oVoxelLength.z);

	//move the mesh position to the actual starting point
	pcl::PointCloud<pcl::PointXYZ>::Ptr pMCResultCloud(new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::PointXYZ oOffset(0, 0, 0);
	oMarchingCuber.OutputMesh(pcl::PointXYZ(0,0,0), oDownResolutionMesh, pMCResultCloud);
	
	///* output result mesh
	if(m_bOutputFiles) {

		std::stringstream sOutputPath;
		sOutputPath << m_sFileHead << std::setw(4) << std::setfill('0') << iFrameId << "_down_mesh.ply";
		pcl::io::savePLYFileBinary(sOutputPath.str(), oDownResolutionMesh);
	}
	//*/
}

void MultiResolutionFusion::SurfelFusionQuick(pcl::PointNormal oLidarPos, pcl::PointCloud<pcl::PointNormal>& vDepthMeasurementCloud) {

    Super::SurfelFusionQuick(oLidarPos, vDepthMeasurementCloud);
}

void MultiResolutionFusion::UpdateOneFrame(const pcl::PointNormal& oViewPoint, pcl::PointCloud<pcl::PointNormal>& vFilteredMeasurementCloud) {

    Super::UpdateOneFrame(oViewPoint, vFilteredMeasurementCloud);

    m_oDownResolutionVoxeler.VoxelizePointsAndFusion(vFilteredMeasurementCloud);
}