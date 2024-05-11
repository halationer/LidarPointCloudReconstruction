#include "Fusion.h"

#include "MeshOperation.h"

Fusion::Fusion(){}
	
Fusion::~Fusion(){}

/*=======================================
SetAccDisSize
Input: iNodeNum - node number (voxel corner number)
Output: m_vKinectWeight - a weight vector
Function:set the size of accumulated distance map 
========================================*/
void Fusion::SetAccDisSize(const int & iNodeNum, float fRawDis){

	//distances of each node in kinnect fusion mode
	m_vAccDis.clear();
	m_vAccDis.resize(iNodeNum, fRawDis);

}


/*=======================================
SetKinectWeight
Input: iNodeNum - node number (voxel corner number)
     fRawWeight - initial weight of each corner (node)
Output: m_vKinectWeight - a weight vector
Function:set node weight in kinect fusion mode
========================================*/
void Fusion::SetKinectMode(const int & iNodeNum, float fRawWeight){

	//weights of each node in kinnect fusion mode
	m_vKinectWeight.clear();
	m_vKinectWeight.resize(iNodeNum, fRawWeight);


}

/*=======================================
KinectFusion
Input: vCurrentDis - the newest signed distance value
          vWeights - the newest weight of each corner (the angle between the face and the ray as usual)
           vDisMap - the accumulated signed distance value (k times)
Output: m_vKinectWeight - a weight vector
               vDisMap - the accumulated signed distance value (k+1 times)
Function:fusion two frame or two glance of mesh using kinect fusion method
========================================*/
void Fusion::KinectFusion(const std::vector<float> & vCurrentDis, std::vector<float> & vCurrentW){

	//update the distance
	for (int i = 0; i != vCurrentDis.size(); ++i){

		//distance
		//d_c = (w_c*d_c+w_new*d_new)/(w_c+w_new)
		m_vAccDis[i] = (m_vAccDis[i] * m_vKinectWeight[i] + vCurrentDis[i] * vCurrentW[i]) / (vCurrentW[i] + m_vKinectWeight[i]);

		//weight
		//w_c = (w_c+w_new)/2.0
		//if vCurrentW[i] is constant 1, the m_vKinectWeight[i] would not change
		m_vKinectWeight[i] = (vCurrentW[i] + m_vKinectWeight[i]) / 2.0f;

	}//end for i

};

/*=======================================
ConvexBasedFusion
Input: f_viewpoint - a given viewpoint
Output: none
Function: set the given viewpoint to class as m_oViewPInWorld
========================================*/
void Fusion::ConvexBasedFusion(const std::vector<float> & vCurrentDis, std::vector<float> & vDisMap){

	//update the distance
	for (int i = 0; i != vCurrentDis.size(); ++i){

		//the case of a visible node
		if (vCurrentDis[i] > 0.0){
			//the case of finding new visiable map
			if (vDisMap[i] < 0.0){
				vDisMap[i] = vCurrentDis[i];
			}else{//end else
				if (vDisMap[i] > vCurrentDis[i])
					vDisMap[i] = vCurrentDis[i];
			}//end else

		//the case of a occluded node
		}else{
		
			if (vDisMap[i] < 0.0){
				if (vDisMap[i] < vCurrentDis[i])
					vDisMap[i] = vCurrentDis[i];
			}//end if vDisMap[i] >= 0.0
		
		}//end big else

	}//end for i


}


/*=======================================
ConvexBasedFusion
Input: f_viewpoint - a given viewpoint
Output: none
Function: set the given viewpoint to class as m_oViewPInWorld
========================================*/
void Fusion::UnionMinimalFusion(const std::vector<float> & vCurrentDis){

	//update the distance
	for (int i = 0; i != vCurrentDis.size(); ++i){

		//the case of a visible node
		if (vCurrentDis[i]){
			//
			if (m_vAccDis[i]){
				//
				if (abs(m_vAccDis[i]) > abs(vCurrentDis[i]))
					m_vAccDis[i] = vCurrentDis[i];
			
			
			}else{
				//
				m_vAccDis[i] = vCurrentDis[i];
			
			}//end else

		}//end big if (vCurrentDis[i])

	}//end for i


}


pcl::PointNormal Fusion::NormalFusion(const std::vector<int> & vPointIdx, const pcl::PointCloud<pcl::PointNormal> & vCloudNormal){

	//Initialize output
	pcl::PointNormal oOnePN;
	oOnePN.x = 0.0f;
	oOnePN.y = 0.0f;
	oOnePN.z = 0.0f;
	oOnePN.normal_x = 0.0f;
	oOnePN.normal_y = 0.0f;
	oOnePN.normal_z = 0.0f;

	//linear increase
	for (int i = 0; i != vPointIdx.size(); ++i){

		int iVoxelPIdx = vPointIdx[i];
	
		oOnePN.x = oOnePN.x + vCloudNormal.points[iVoxelPIdx].x;
		oOnePN.y = oOnePN.y + vCloudNormal.points[iVoxelPIdx].y;
		oOnePN.z = oOnePN.z + vCloudNormal.points[iVoxelPIdx].z;
		oOnePN.normal_x = oOnePN.normal_x + vCloudNormal.points[iVoxelPIdx].normal_x;
		oOnePN.normal_y = oOnePN.normal_y + vCloudNormal.points[iVoxelPIdx].normal_y;
		oOnePN.normal_z = oOnePN.normal_z + vCloudNormal.points[iVoxelPIdx].normal_z;
	
	}

	
	float fNum = float(vPointIdx.size());

	if (vPointIdx.size()){
		//take the mean
		oOnePN.x = oOnePN.x / fNum;
		oOnePN.y = oOnePN.y / fNum;
		oOnePN.z = oOnePN.z / fNum;
		oOnePN.normal_x = oOnePN.normal_x / fNum;
		oOnePN.normal_y = oOnePN.normal_y / fNum;
		oOnePN.normal_z = oOnePN.normal_z / fNum;
	}

	return oOnePN;


}

pcl::PointNormal Fusion::NormalFusionWeighted(
	const std::vector<int> & vPointIdx, 
	pcl::PointCloud<pcl::PointNormal> & vCloudNormal, 
	const pcl::PointNormal & oBase)
{
	//Initialize output
	pcl::PointNormal oOnePN = oBase;
	float& normal_distribution_distance = oOnePN.data_c[3];
	float& all_weight = oOnePN.data_n[3];
	oOnePN.x *= all_weight;
	oOnePN.y *= all_weight;
	oOnePN.z *= all_weight;
	oOnePN.normal_x *= all_weight;
	oOnePN.normal_y *= all_weight;
	oOnePN.normal_z *= all_weight;
	// float fNum = all_weight > 0 ? 1 : 0;

	//linear increase
	for (int i = 0; i != vPointIdx.size(); ++i){

		int iVoxelPIdx = vPointIdx[i];
		float point_weight = vCloudNormal.points[iVoxelPIdx].data_n[3];
	
		oOnePN.x = oOnePN.x + point_weight * vCloudNormal.points[iVoxelPIdx].x;
		oOnePN.y = oOnePN.y + point_weight * vCloudNormal.points[iVoxelPIdx].y;
		oOnePN.z = oOnePN.z + point_weight * vCloudNormal.points[iVoxelPIdx].z;
		oOnePN.normal_x = oOnePN.normal_x + point_weight * vCloudNormal.points[iVoxelPIdx].normal_x;
		oOnePN.normal_y = oOnePN.normal_y + point_weight * vCloudNormal.points[iVoxelPIdx].normal_y;
		oOnePN.normal_z = oOnePN.normal_z + point_weight * vCloudNormal.points[iVoxelPIdx].normal_z;
		all_weight = all_weight + point_weight;
	}

	if(all_weight == 0.f) return oOnePN;
	
	// fNum += vPointIdx.size();

	if (vPointIdx.size()){
		//take the mean
		oOnePN.x = oOnePN.x / all_weight;
		oOnePN.y = oOnePN.y / all_weight;
		oOnePN.z = oOnePN.z / all_weight;
		oOnePN.normal_x = oOnePN.normal_x / all_weight;
		oOnePN.normal_y = oOnePN.normal_y / all_weight;
		oOnePN.normal_z = oOnePN.normal_z / all_weight;
		normal_distribution_distance = sqrt(oOnePN.normal_x * oOnePN.normal_x + oOnePN.normal_y * oOnePN.normal_y + oOnePN.normal_z * oOnePN.normal_z);
	}

	// spread back to the cloud normal
	// for (int i = 0; i != vPointIdx.size(); ++i){

	// 	int iVoxelPIdx = vPointIdx[i];
	// 	float point_weight = vCloudNormal.points[iVoxelPIdx].data_n[3];
	// 	vCloudNormal.points[iVoxelPIdx].normal_x = all_weight * oOnePN.normal_x + point_weight * vCloudNormal.points[iVoxelPIdx].normal_x;
	// 	vCloudNormal.points[iVoxelPIdx].normal_y = all_weight * oOnePN.normal_y + point_weight * vCloudNormal.points[iVoxelPIdx].normal_y;
	// 	vCloudNormal.points[iVoxelPIdx].normal_z = all_weight * oOnePN.normal_z + point_weight * vCloudNormal.points[iVoxelPIdx].normal_z;

	// 	vCloudNormal.points[iVoxelPIdx].normal_x /= (all_weight + point_weight);
	// 	vCloudNormal.points[iVoxelPIdx].normal_y /= (all_weight + point_weight);
	// 	vCloudNormal.points[iVoxelPIdx].normal_z /= (all_weight + point_weight);

	// 	MeshOperation m;
	// 	m.VectorNormalization(vCloudNormal.points[iVoxelPIdx].normal_x, vCloudNormal.points[iVoxelPIdx].normal_y, vCloudNormal.points[iVoxelPIdx].normal_z);
	// }

	// constexpr float max_weight = 5.0f;
	// all_weight = all_weight > max_weight ? max_weight : all_weight;

	return oOnePN;
}