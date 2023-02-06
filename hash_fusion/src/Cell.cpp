#include "Cell.h"


Voxelization::Voxelization(const pcl::PointCloud<pcl::PointXYZ> & vCloud)
	                  :m_fMinX(FLT_MAX),m_fMinY(FLT_MAX),m_fMinZ(FLT_MAX),
                    m_fMaxX(-FLT_MAX),m_fMaxY(-FLT_MAX),m_fMaxZ(-FLT_MAX),
					m_fDefault(0.0), m_bParamFlag(false), m_fExpandNum(1.0),
					m_pCornerCloud(new pcl::PointCloud<pcl::PointXYZ>),
					m_pVoxelNormals(new pcl::PointCloud<pcl::PointNormal>){

	BoundingBoxValue(vCloud);

	m_vVoxelPointIdx.clear();

}


Voxelization::Voxelization(const pcl::PointCloud<pcl::PointNormal> & vCloud)
	:m_fMinX(FLT_MAX), m_fMinY(FLT_MAX), m_fMinZ(FLT_MAX),
	m_fMaxX(-FLT_MAX), m_fMaxY(-FLT_MAX), m_fMaxZ(-FLT_MAX),
	m_fDefault(0.0), m_bParamFlag(false), m_fExpandNum(2.0),
	m_pCornerCloud(new pcl::PointCloud<pcl::PointXYZ>),
	m_pVoxelNormals(new pcl::PointCloud<pcl::PointNormal>){

	BoundingBoxValue(vCloud);

	m_vVoxelPointIdx.clear();

}

/*=======================================
BoundingBoxValue
Input: vCloud - a given point clouds
Output: m_fMinX,m_fMinY,m_fMinZ,m_fMaxX,m_fMaxY,m_fMaxZ - the maximum and minimum values of the point cloud on the three axes
Function: Compute extrema of bounding box of input point clouds
========================================*/
Voxelization::~Voxelization(){



}

/*=======================================
SetInitialNodeValue
Input: fDefault - the default of node value at the beginning
Output: set value of m_fDefault
========================================*/
void Voxelization::SetInitialNodeValue(float fDefault){

	m_fDefault = fDefault;

}

/*=======================================
BoundingBoxValue
Input: vCloud - a given point clouds
Output: m_fMinX,m_fMinY,m_fMinZ,m_fMaxX,m_fMaxY,m_fMaxZ - the maximum and minimum values of the point cloud on the three axes
Function: Compute extrema of bounding box of input point clouds
========================================*/
void Voxelization::GetResolution(pcl::PointXYZ oLength){

	if (!m_bParamFlag){

		m_oVoxelLength.x = oLength.x;
		m_oVoxelLength.y = oLength.y;
		m_oVoxelLength.z = oLength.z;

		//calculate the length of each side
		pcl::PointXYZ oBoundLength;
		oBoundLength.x = m_oMaxCorner.x - m_oMinCorner.x;
		oBoundLength.y = m_oMaxCorner.y - m_oMinCorner.y;
		oBoundLength.z = m_oMaxCorner.z - m_oMinCorner.z;

		//get the voxel number
		m_iNoExpandVoxelNum.ixnum = ceil(oBoundLength.x / m_oVoxelLength.x);
		m_iNoExpandVoxelNum.iynum = ceil(oBoundLength.y / m_oVoxelLength.y);
		m_iNoExpandVoxelNum.iznum = ceil(oBoundLength.z / m_oVoxelLength.z);

		m_bParamFlag = true;
	
	}else{

		m_oVoxelLength.x = oLength.x;
		m_oVoxelLength.y = oLength.y;
		m_oVoxelLength.z = oLength.z;
	
	}

}

/*=======================================
BoundingBoxValue
Input: vCloud - a given point clouds
Output: m_fMinX,m_fMinY,m_fMinZ,m_fMaxX,m_fMaxY,m_fMaxZ - the maximum and minimum values of the point cloud on the three axes
Function: Compute extrema of bounding box of input point clouds
========================================*/
void Voxelization::SetExpandNum(float fExpandNum){

	m_fExpandNum = fExpandNum;

}

/*=======================================
BoundingBoxValue
Input: vCloud - a given point clouds
Output: m_fMinX,m_fMinY,m_fMinZ,m_fMaxX,m_fMaxY,m_fMaxZ - the maximum and minimum values of the point cloud on the three axes
Function: Compute extrema of bounding box of input point clouds
========================================*/
int Voxelization::Tran3DIdxTo1D(const IndexinAxis & o3DIdx){

	return o3DIdx.iznum * (m_iFinalVoxelNum.iynum * m_iFinalVoxelNum.ixnum) + o3DIdx.iynum * m_iFinalVoxelNum.ixnum + o3DIdx.ixnum;


}

/*=======================================
BoundingBoxValue
Input: i1DIdx - a given point clouds
Output: m_fMinX,m_fMinY,m_fMinZ,m_fMaxX,m_fMaxY,m_fMaxZ - the maximum and minimum values of the point cloud on the three axes
Function: Compute extrema of bounding box of input point clouds
========================================*/
IndexinAxis Voxelization::Tran1DIdxTo3D(const int & i1DIdx){

	IndexinAxis o3DIdx;
	o3DIdx.iznum = i1DIdx / (m_iFinalVoxelNum.iynum * m_iFinalVoxelNum.ixnum);
	o3DIdx.iynum = (i1DIdx - o3DIdx.iznum * (m_iFinalVoxelNum.iynum * m_iFinalVoxelNum.ixnum)) / m_iFinalVoxelNum.ixnum;
	o3DIdx.ixnum = i1DIdx - o3DIdx.iznum * (m_iFinalVoxelNum.iynum * m_iFinalVoxelNum.ixnum) - o3DIdx.iynum * m_iFinalVoxelNum.ixnum;

	return o3DIdx;

}

/*=======================================
BoundingBoxValue
Input: vCloud - a given point clouds
Output: m_fMinX,m_fMinY,m_fMinZ,m_fMaxX,m_fMaxY,m_fMaxZ - the maximum and minimum values of the point cloud on the three axes
Function: Compute extrema of bounding box of input point clouds
========================================*/
void Voxelization::GetIntervalNum(int iVoxelNumX, int iVoxelNumY, int iVoxelNumZ){

	if (!m_bParamFlag){

		m_iNoExpandVoxelNum.ixnum = iVoxelNumX;
		m_iNoExpandVoxelNum.iynum = iVoxelNumY;
		m_iNoExpandVoxelNum.iznum = iVoxelNumZ;

		//calculate the length of each side
		pcl::PointXYZ oBoundLength;
		oBoundLength.x = m_oMaxCorner.x - m_oMinCorner.x;
		oBoundLength.y = m_oMaxCorner.y - m_oMinCorner.y;
		oBoundLength.z = m_oMaxCorner.z - m_oMinCorner.z;

		//
		m_oVoxelLength.x = oBoundLength.x / float(m_iNoExpandVoxelNum.ixnum);
		m_oVoxelLength.y = oBoundLength.y / float(m_iNoExpandVoxelNum.iynum);
		m_oVoxelLength.z = oBoundLength.z / float(m_iNoExpandVoxelNum.iznum);

		m_bParamFlag = true;

	}
	else{

		m_iNoExpandVoxelNum.ixnum = iVoxelNumX;
		m_iNoExpandVoxelNum.iynum = iVoxelNumY;
		m_iNoExpandVoxelNum.iznum = iVoxelNumZ;
	
	}

}

/*=======================================
BoundingBoxValue
Input: vCloud - a given point clouds
Output: m_fMinX,m_fMinY,m_fMinZ,m_fMaxX,m_fMaxY,m_fMaxZ - the maximum and minimum values of the point cloud on the three axes
Function: Compute extrema of bounding box of input point clouds
========================================*/
void SetVoxelParam(pcl::PointXYZ oOrignal, int iNumX, int iNumY, int iNumZ, float fLengthX, float fLengthY, float fLengthZ){





}


/*=======================================
BoundingBoxValue
Input: vCloud - a given point clouds
Output: m_fMinX,m_fMinY,m_fMinZ,m_fMaxX,m_fMaxY,m_fMaxZ - the maximum and minimum values of the point cloud on the three axes
Function: Compute extrema of bounding box of input point clouds
========================================*/
void Voxelization::BoundingBoxValue(const pcl::PointCloud<pcl::PointXYZ> & vCloud){

	//get the maximum and minimum value
	for (int i = 0; i != vCloud.points.size(); ++i){

		if (m_fMinX > vCloud.points[i].x)
			m_fMinX = vCloud.points[i].x;
		if (m_fMinY > vCloud.points[i].y)
			m_fMinY = vCloud.points[i].y;
		if (m_fMinZ > vCloud.points[i].z)
			m_fMinZ = vCloud.points[i].z;
		if (m_fMaxX < vCloud.points[i].x)
			m_fMaxX = vCloud.points[i].x;
		if (m_fMaxY < vCloud.points[i].y)
			m_fMaxY = vCloud.points[i].y;
		if (m_fMaxZ < vCloud.points[i].z)
			m_fMaxZ = vCloud.points[i].z;
	}
	//collect the minimum value as a original point
	m_oMinCorner.x = m_fMinX;
	m_oMinCorner.y = m_fMinY;
	m_oMinCorner.z = m_fMinZ;

	//get the maximum value as a reference point
	m_oMaxCorner.x = m_fMaxX;
	m_oMaxCorner.y = m_fMaxY;
	m_oMaxCorner.z = m_fMaxZ;

}

/*=======================================
BoundingBoxValue
Input: vCloud - a given point clouds
Output: m_fMinX,m_fMinY,m_fMinZ,m_fMaxX,m_fMaxY,m_fMaxZ - the maximum and minimum values of the point cloud on the three axes
Function: Compute extrema of bounding box of input point clouds
========================================*/
void Voxelization::BoundingBoxValue(const pcl::PointCloud<pcl::PointNormal> & vCloud){

	//get the maximum and minimum value
	for (int i = 0; i != vCloud.points.size(); ++i){

		if (m_fMinX > vCloud.points[i].x)
			m_fMinX = vCloud.points[i].x;
		if (m_fMinY > vCloud.points[i].y)
			m_fMinY = vCloud.points[i].y;
		if (m_fMinZ > vCloud.points[i].z)
			m_fMinZ = vCloud.points[i].z;
		if (m_fMaxX < vCloud.points[i].x)
			m_fMaxX = vCloud.points[i].x;
		if (m_fMaxY < vCloud.points[i].y)
			m_fMaxY = vCloud.points[i].y;
		if (m_fMaxZ < vCloud.points[i].z)
			m_fMaxZ = vCloud.points[i].z;
	}
	//collect the minimum value as a original point
	m_oMinCorner.x = m_fMinX;
	m_oMinCorner.y = m_fMinY;
	m_oMinCorner.z = m_fMinZ;

	//get the maximum value as a reference point
	m_oMaxCorner.x = m_fMaxX;
	m_oMaxCorner.y = m_fMaxY;
	m_oMaxCorner.z = m_fMaxZ;

}


/*=======================================
BoundingBoxValue
Input: vCloud - a given point clouds
Output: m_fMinX,m_fMinY,m_fMinZ,m_fMaxX,m_fMaxY,m_fMaxZ - the maximum and minimum values of the point cloud on the three axes
Function: Compute extrema of bounding box of input point clouds
========================================*/
void Voxelization::VoxelizeSpace(){


	//get the original point from min corner with a offset
	m_oOriCorner.x = m_oMinCorner.x - m_fExpandNum * m_oVoxelLength.x;
	m_oOriCorner.y = m_oMinCorner.y - m_fExpandNum * m_oVoxelLength.y;
	m_oOriCorner.z = m_oMinCorner.z - m_fExpandNum * m_oVoxelLength.z;
	
	//expand the boundary with adding two sides
	m_iFinalVoxelNum.ixnum = m_iNoExpandVoxelNum.ixnum + 2.0 * m_fExpandNum;
	m_iFinalVoxelNum.iynum = m_iNoExpandVoxelNum.iynum + 2.0 * m_fExpandNum;
	m_iFinalVoxelNum.iznum = m_iNoExpandVoxelNum.iznum + 2.0 * m_fExpandNum;

	//expand the potential space of three vectors representing voxel corner information
	m_pCornerCloud->reserve(m_iFinalVoxelNum.ixnum * m_iFinalVoxelNum.iynum * m_iFinalVoxelNum.iznum);//location
	//m_vCornerValue.reserve(m_iFinalVoxelNum.ixnum * m_iFinalVoxelNum.iynum * m_iFinalVoxelNum.iznum);//value
	//m_vCorToGridIdx.reserve(m_iFinalVoxelNum.ixnum * m_iFinalVoxelNum.iynum * m_iFinalVoxelNum.iznum);//index in 3d

	//voxelization
	//If you want to generate five nodes in the interval from 0 to 5 by 4 voxel
	//the interval is cut into 4 segments, and 0,1,2,3,4 node would be labeled
	//So you need to add one voxel to label 5 (it is why iz < m_fFinalVoxelNum instead of iz != m_fFinalVoxelNum)
	for (int iz = 0; iz < m_iFinalVoxelNum.iznum; ++iz){

		for (int iy = 0; iy < m_iFinalVoxelNum.iynum; ++iy){

			for (int ix = 0; ix <m_iFinalVoxelNum.ixnum; ++ix){

				pcl::PointXYZ oPoint;
				oPoint.x = m_oOriCorner.x + m_oVoxelLength.x * float(ix);
				oPoint.y = m_oOriCorner.y + m_oVoxelLength.y * float(iy);
				oPoint.z = m_oOriCorner.z + m_oVoxelLength.z * float(iz);

				//1.get the corner point for repeated calculation			
				m_pCornerCloud->points.push_back(oPoint);

				//2.get the index of each corner/node
				//int iGrididx = iz * m_iVoxelNum * m_iVoxelNum + iy * m_iVoxelNum + ix;
				//IndexinAxis oPoint3DIndex;
				//oPoint3DIndex.ixnum = ix;
				//oPoint3DIndex.iynum = iy;
				//oPoint3DIndex.iznum = iz;
				//get 3d index 
				//m_vCorToGridIdx.push_back(oPoint3DIndex);

				//3.get the value of each corner/node
				//m_vCornerValue.push_back(m_fDefault);

			}//end z

		}//end y

	}//end x

}


/*=======================================
PointBelongVoxel(still test)
Input: oPoint - a given point 
Output: the voxel index of a point
Function: calculate the index of voxel which the point belongs to
========================================*/
template<class PointType>
int Voxelization::PointBelongVoxel(const PointType & oPoint){

	IndexinAxis oP3DIndex;
	oP3DIndex.ixnum = floor((oPoint.x - m_oOriCorner.x) / m_oVoxelLength.x);
	oP3DIndex.iynum = floor((oPoint.y - m_oOriCorner.y) / m_oVoxelLength.y);
	oP3DIndex.iznum = floor((oPoint.z - m_oOriCorner.z) / m_oVoxelLength.z);

	return Tran3DIdxTo1D(oP3DIndex);
}
template int Voxelization::PointBelongVoxel(const pcl::PointXYZ & oPoint);
template int Voxelization::PointBelongVoxel(const pcl::PointNormal & oPoint);

/*=======================================
PointBelongVoxel
Input:oPoint - a given point 
Output: oP3DIndex - 3d index of the wrapped voxel
Tran3DIdxTo1D(oP3DIndex) - 1d index of the wrapped voxel
Function: calculate the 3d index of voxel which the point belongs to
========================================*/
template<class PointType>
int Voxelization::PointBelongVoxel(const PointType & oPoint, IndexinAxis & oP3DIndex){
	
	oP3DIndex.ixnum = floor((oPoint.x - m_oOriCorner.x) / m_oVoxelLength.x);
	oP3DIndex.iynum = floor((oPoint.y - m_oOriCorner.y) / m_oVoxelLength.y);
	oP3DIndex.iznum = floor((oPoint.z - m_oOriCorner.z) / m_oVoxelLength.z);

	return Tran3DIdxTo1D(oP3DIndex);
}
template int Voxelization::PointBelongVoxel(const pcl::PointXYZ & oPoint, IndexinAxis & oP3DIndex);
template int Voxelization::PointBelongVoxel(const pcl::PointNormal & oPoint, IndexinAxis & oP3DIndex);

template<>
int Voxelization::PointBelongVoxel(const Eigen::Vector3f & oPoint, IndexinAxis & oP3DIndex){

	oP3DIndex.ixnum = floor((oPoint.x() - m_oOriCorner.x) / m_oVoxelLength.x);
	oP3DIndex.iynum = floor((oPoint.y() - m_oOriCorner.y) / m_oVoxelLength.y);
	oP3DIndex.iznum = floor((oPoint.z() - m_oOriCorner.z) / m_oVoxelLength.z);

	return Tran3DIdxTo1D(oP3DIndex);
}


template<class PointType>
bool Voxelization::OutOfBorder(const PointType & oPoint) {
	
	return oPoint.x < 0 || oPoint.y < 0 || oPoint.z < 0 ||
		oPoint.x >= m_iFinalVoxelNum.ixnum || oPoint.y >= m_iFinalVoxelNum.iynum || oPoint.z >= m_iFinalVoxelNum.iznum;
}
template<>
bool Voxelization::OutOfBorder(const Eigen::Vector3f & oPoint) {
	return oPoint.x() < 0 || oPoint.y() < 0 || oPoint.z() < 0 ||
		oPoint.x() >= m_iFinalVoxelNum.ixnum || oPoint.y() >= m_iFinalVoxelNum.iynum || oPoint.z() >= m_iFinalVoxelNum.iznum;
}
template<>
bool Voxelization::OutOfBorder(const IndexinAxis & oPoint) {
	return oPoint.ixnum < 0 || oPoint.iynum < 0 || oPoint.iznum < 0 ||
		oPoint.ixnum >= m_iFinalVoxelNum.ixnum || oPoint.iynum >= m_iFinalVoxelNum.iynum || oPoint.iznum >= m_iFinalVoxelNum.iznum;
}

/*=======================================
VoxelizePoints
Input: vCloud - a given point clouds
Output: m_vNearStatus - a vector indicating whether the corner is a surface proximity point
        m_vVoxelPointIdx - point index within each voxel
Function: voxelize a given sampling point
========================================*/
//compute a given point clouds belong to the corresponding voxel

void Voxelization::VoxelizePoints(const pcl::PointCloud<pcl::PointXYZ> & vSampledCloud){

	//clear the voxel index
	m_vVoxelPointIdx.clear();
	m_vVoxelPointIdx.reserve(m_pCornerCloud->points.size());
	//construct new voxel index
	std::vector<int> vEmptyVec;
	for (int i = 0; i != m_pCornerCloud->points.size(); ++i)
		m_vVoxelPointIdx.push_back(vEmptyVec);

	//set the 
	m_vNearStatus.clear();
	m_vNearStatus.resize(m_pCornerCloud->points.size(), false);

	//get each point index
	for (int i = 0; i != vSampledCloud.points.size(); ++i){
		//3d index and 1d index of voxel in which it is 
		IndexinAxis oP3DIndex;
		int iVoxelIdx = PointBelongVoxel(vSampledCloud.points[i], oP3DIndex);
		//if this voxel is still a empty voxel
		//the label its corner as near node for calculation of signed distance
		if (!m_vVoxelPointIdx[iVoxelIdx].size()){
			//get the corners
			std::vector<int> vCornerIdxs;
			CornerIdxs(oP3DIndex, vCornerIdxs);
			for (int i = 0; i != vCornerIdxs.size(); ++i){
				m_vNearStatus[vCornerIdxs[i]] = true;
			}
		
		}
		//get data
		m_vVoxelPointIdx[iVoxelIdx].push_back(i);

	}//end for

}

/*=======================================
VoxelizePoints
Input: vCloud - a given point clouds
Output: m_vNearStatus - a vector indicating whether the corner is a surface proximity point
m_vVoxelPointIdx - point index within each voxel
Function: voxelize a given sampling point
========================================*/
//compute a given point clouds belong to the corresponding voxel

void Voxelization::VoxelizePoints(const pcl::PointCloud<pcl::PointNormal> & vSampledCloud){

	//clear the voxel index
	//voxel number is match with the corner number 
	m_vVoxelPointIdx.clear();
	m_vVoxelPointIdx.reserve(m_pCornerCloud->points.size());

	//construct new voxel index
	std::vector<int> vEmptyVec;
	for (int i = 0; i != m_pCornerCloud->points.size(); ++i)
		m_vVoxelPointIdx.push_back(vEmptyVec);

	//get each point index
	for (int i = 0; i != vSampledCloud.points.size(); ++i){

		int iVoxelIdx = PointBelongVoxel(vSampledCloud.points[i]);

		//get data
		m_vVoxelPointIdx[iVoxelIdx].push_back(i);

	}//end for

}

/*=======================================
VoxelizePoints
Input: vSampledCloud - a sampled point clouds from mesh
       vDenseCloud - a raw point clouds, "Dense" means that the point clouds must be dense
Output: m_vNearStatus - a vector indicating whether the corner is a surface proximity point
m_vVoxelPointIdx - point index within each voxel
Function: voxelize a given sampling point
========================================*/
//compute a given point clouds belong to the corresponding voxel

void Voxelization::VoxelizePoints(const pcl::PointCloud<pcl::PointXYZ> & vSampledCloud, const pcl::PointCloud<pcl::PointXYZ> & vDenseCloud){

	//clear the voxel index
	m_vVoxelPointIdx.clear();
	m_vVoxelPointIdx.reserve(m_pCornerCloud->points.size());
	//construct new voxel index
	std::vector<int> vEmptyVec;
	for (int i = 0; i != m_pCornerCloud->points.size(); ++i)
		m_vVoxelPointIdx.push_back(vEmptyVec);

	//clear the status of near node
	m_vNearStatus.clear();
	m_vNearStatus.resize(m_pCornerCloud->points.size(), false);

	//compute the non-empty voxel first caused by raw point clouds
	std::vector<bool> vRawVoxelStatus(m_vVoxelPointIdx.size(), false);
	//compute the raw data to know 
	for (int i = 0; i != vDenseCloud.points.size(); ++i){
		//3d index and 1d index of voxel in which it is 
		IndexinAxis oP3DIndex;
		int iVoxelIdx = PointBelongVoxel(vDenseCloud.points[i], oP3DIndex);
		vRawVoxelStatus[iVoxelIdx] = true;
	}

	//then compute the non-empty voxel caused by sampled point clouds
	for (int i = 0; i != vSampledCloud.points.size(); ++i){
		//3d index and 1d index of voxel in which it is 
		IndexinAxis oP3DIndex;
		int iVoxelIdx = PointBelongVoxel(vSampledCloud.points[i], oP3DIndex);
		//if this voxel is still a empty voxel
		//the label its corner as near node for calculation of signed distance
		//and also if this voxel has the raw data
		if (!m_vVoxelPointIdx[iVoxelIdx].size() && vRawVoxelStatus[iVoxelIdx]){
			//get the corners
			std::vector<int> vCornerIdxs;
			CornerIdxs(oP3DIndex, vCornerIdxs);
			for (int i = 0; i != vCornerIdxs.size(); ++i){
				m_vNearStatus[vCornerIdxs[i]] = true;
			}

		}
		//get data
		m_vVoxelPointIdx[iVoxelIdx].push_back(i);

	}//end for

}


void Voxelization::OutputNonEmptyVoxels(std::vector<bool> & vVoxelStatus){

	vVoxelStatus.clear();
	vVoxelStatus.resize(m_vVoxelPointIdx.size(), false);
	
	for (int i = 0; i != m_vVoxelPointIdx.size(); ++i){
	
		if (m_vVoxelPointIdx[i].size())
			vVoxelStatus[i] = true;
	
	}


}


/*=======================================
FindNearNodes
Input: vCloud - a given point clouds
Output: vNearNodes - non-empty voxel's conners 
Function: find the nodes near the surface (sampled points)
========================================*/
//compute a given point clouds belong to the corresponding voxel
//void Voxelization::FindNearNodes(const pcl::PointCloud<pcl::PointXYZ> & vCloud, pcl::PointCloud<pcl::PointXYZ> & vNearNodes){
//
//	//clear the voxel index
//	std::vector<bool> vNearStatus(m_pCornerCloud->points.size(), false);
//
//	//construct new voxel index
//	std::vector<int> vOneVoxelIdx;
//	for (int i = 0; i != m_vCornerValue.size(); ++i)
//		m_vVoxelPointIdx.push_back(vOneVoxelIdx);
//
//	//get each point index
//	for (int i = 0; i != vCloud.points.size(); ++i){
//		//point index in voxel
//		IndexinAxis oP3DIndex;
//		PointBelongVoxel(vCloud.points[i], oP3DIndex);
//		//get data
//		m_vVoxelPointIdx[iVoxelIdx].push_back(i);
//
//	}//end for
//
//}



/*=======================================
CornerIdxs
Input: IndexinAxis - a given voxel 1D index 
Output: vNeighborIdxs - eight corner 1D indexes of this voxel



                z_dir 4 ________ 5
			          /|       /|
			        /  |     /  |
			    7 /_______ /    |
			     |     |  |6    |
			     |    0|__|_____|1 y_dir
                 |    /   |    /
                 |  /     |  /
		    x_dir|/_______|/
				3          2


Function: return a voxel corner indexes
========================================*/
void Voxelization::CornerIdxs(const IndexinAxis & o3DIdx, std::vector<int> & vNeighborIdxs){

	vNeighborIdxs.clear();
	
	IndexinAxis oOneCorner3DIdx;
	int iCornerIdx;

	//0,0,0 -> corner 0
	oOneCorner3DIdx.ixnum = o3DIdx.ixnum;
	oOneCorner3DIdx.iynum = o3DIdx.iynum;
	oOneCorner3DIdx.iznum = o3DIdx.iznum;
	iCornerIdx = Tran3DIdxTo1D(oOneCorner3DIdx);
	vNeighborIdxs.push_back(iCornerIdx);
	//1,0,0 -> corner 3
	oOneCorner3DIdx.ixnum = o3DIdx.ixnum + 1;
	oOneCorner3DIdx.iynum = o3DIdx.iynum;
	oOneCorner3DIdx.iznum = o3DIdx.iznum;
	iCornerIdx = Tran3DIdxTo1D(oOneCorner3DIdx);
	vNeighborIdxs.push_back(iCornerIdx);
	//0, 1, 0 -> corner 1
	oOneCorner3DIdx.ixnum = o3DIdx.ixnum;
	oOneCorner3DIdx.iynum = o3DIdx.iynum + 1;
	oOneCorner3DIdx.iznum = o3DIdx.iznum;
	iCornerIdx = Tran3DIdxTo1D(oOneCorner3DIdx);
	vNeighborIdxs.push_back(iCornerIdx);
	//0, 0, 1-> corner 4
	oOneCorner3DIdx.ixnum = o3DIdx.ixnum;
	oOneCorner3DIdx.iynum = o3DIdx.iynum;
	oOneCorner3DIdx.iznum = o3DIdx.iznum + 1;
	iCornerIdx = Tran3DIdxTo1D(oOneCorner3DIdx);
	vNeighborIdxs.push_back(iCornerIdx);
	//1, 0 ,1-> corner 7
	oOneCorner3DIdx.ixnum = o3DIdx.ixnum + 1;
	oOneCorner3DIdx.iynum = o3DIdx.iynum;
	oOneCorner3DIdx.iznum = o3DIdx.iznum + 1;
	iCornerIdx = Tran3DIdxTo1D(oOneCorner3DIdx);
	vNeighborIdxs.push_back(iCornerIdx);
	//1, 1, 0-> corner 2
	oOneCorner3DIdx.ixnum = o3DIdx.ixnum + 1;
	oOneCorner3DIdx.iynum = o3DIdx.iynum + 1;
	oOneCorner3DIdx.iznum = o3DIdx.iznum;
	iCornerIdx = Tran3DIdxTo1D(oOneCorner3DIdx);
	vNeighborIdxs.push_back(iCornerIdx);
	//0, 1, 1-> corner 5
	oOneCorner3DIdx.ixnum = o3DIdx.ixnum;		//here before +1 which is a mistake
	oOneCorner3DIdx.iynum = o3DIdx.iynum + 1;
	oOneCorner3DIdx.iznum = o3DIdx.iznum + 1;
	iCornerIdx = Tran3DIdxTo1D(oOneCorner3DIdx);
	vNeighborIdxs.push_back(iCornerIdx);
	//1, 1, 1-> corner 6
	oOneCorner3DIdx.ixnum = o3DIdx.ixnum + 1;
	oOneCorner3DIdx.iynum = o3DIdx.iynum + 1;
	oOneCorner3DIdx.iznum = o3DIdx.iznum + 1;
	iCornerIdx = Tran3DIdxTo1D(oOneCorner3DIdx);
	vNeighborIdxs.push_back(iCornerIdx);

}

/*=======================================
ClearMiddleData
Input: IndexinAxis - a given voxel 1D index
Output: vNeighborIdxs - eight corner 1D indexes of this voxel
Function: clear some data not all
Middle data refers to the data produced during one observation
e.g.,
========================================*/
void Voxelization::ClearMiddleData(){


	//the index of point in each voxel
	 m_vVoxelPointIdx.clear();

	//a vector indicates whether the corner is adjacent to the surface
	m_vNearStatus.clear();


}