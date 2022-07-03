#include"GHPR.h"


/*=======================================
GHPR
Input: f_viewpoint - a given viewpoint
       f_param - a given fparam for point cloud geometric transformation
Output: none
Function: The constructor of class GHPR
========================================*/
GHPR::GHPR(pcl::PointXYZ f_viewpoint, double f_param) : 
                      m_pTransCloud(new pcl::PointCloud<pcl::PointXYZ>),
					  m_pHullVertices(new pcl::PointCloud<pcl::PointXYZ>){

    SetParam(f_param);

	Inputviewpoint(f_viewpoint);
	
	//the index has not been established 
	m_bToWorldIndex = false;

	//the radius has not been computed 
	m_bComputeRadius = false;

	//defaults
	m_fRadius = 0.0;

}


/*=======================================
SetParam
Input: f_param - a given param
Output: none
Function: set the parameter to param
========================================*/
void GHPR::SetParam(double f_param){
	param=f_param;
}


/*=======================================
Inputviewpoint
Input: f_viewpoint - a given viewpoint
Output: none
Function: set the given viewpoint to class as m_oViewPInWorld
========================================*/
void GHPR::Inputviewpoint(const pcl::PointXYZ & f_viewpoint){

	m_oViewPInWorld = f_viewpoint;

}


/*=======================================
NormVector
Input: oPoint - a given point
Output: fSum - modulo of the given vector
Function: calculate the modulo length of a vector
========================================*/
inline float GHPR::NormVector(const pcl::PointXYZ & oPoint){

	//new the output
	float fSum = oPoint.x*oPoint.x + oPoint.y*oPoint.y + oPoint.z*oPoint.z;

	//sqrt
	fSum = sqrt(fSum);

	return fSum;

}


/*=======================================
GetMaxValue
Input: vVec - a vector with float type
Output: fMax - the maximum value within the vector
Function: get the maximum value of vector members
========================================*/
inline float GHPR::GetMaxValue(std::vector<float> & vVec){

	//new the maximum value
	float fMax;
	fMax = vVec[0];

	//get the maximum value
	for (int i = 0; i != vVec.size(); ++i){
		if (vVec[i] > fMax)
			fMax = vVec[i];
	}

	return fMax;

}


/*=======================================
ConvertCloud
Input: pCloud - point clouds in world coordinate
       m_oViewPInWorld - viewpoint in world coordinate
Output: m_pTransCloud - geometrically transformed points including raw points and viewpoint
Function: geometric inversion transformation, it converts points from world into local coordiante
========================================*/
void GHPR::ConvertCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pCloud){

	m_pTransCloud->clear();

	//the modulus of each point
	std::vector<float> vNormEachPoint;

	//get local coordinates based on the viewpoint 
	for (int i = 0; i != pCloud->points.size(); ++i){

		pcl::PointXYZ oOnePoint;

		oOnePoint.x = pCloud->points[i].x - m_oViewPInWorld.x;
		oOnePoint.y = pCloud->points[i].y - m_oViewPInWorld.y;
		oOnePoint.z = pCloud->points[i].z - m_oViewPInWorld.z;
		m_pTransCloud->points.push_back(oOnePoint);

		//calculate the modulus length
		vNormEachPoint.push_back(NormVector(oOnePoint));
	}


	//get the radius of local coordiante system
	m_fRadius = pow(10.0, param)*GetMaxValue(vNormEachPoint);

	//
	for (size_t i = 0; i != m_pTransCloud->points.size(); i++){
		float numerator = 2 * (m_fRadius - vNormEachPoint[i]);
		m_pTransCloud->points[i].x = m_pTransCloud->points[i].x + numerator*m_pTransCloud->points[i].x / vNormEachPoint[i];
		m_pTransCloud->points[i].y = m_pTransCloud->points[i].y + numerator*m_pTransCloud->points[i].y / vNormEachPoint[i];
		m_pTransCloud->points[i].z = m_pTransCloud->points[i].z + numerator*m_pTransCloud->points[i].z / vNormEachPoint[i];
	}

	//set the viewpoint at the original value
	pcl::PointXYZ oLocalViewPoint;
	oLocalViewPoint.x = 0.0;
	oLocalViewPoint.y = 0.0;
	oLocalViewPoint.z = 0.0;
	m_pTransCloud->points.push_back(oLocalViewPoint);

	//Now, the transposed transformed radius has been obtained
	m_bComputeRadius = true;

}


/*=======================================
ConvertCloud
Input: vSCloud - source point clouds   
Output: vTCloud - target point clouds,also the transformed point cloud
Function: it converts points from world into local coordiante based on viewpoint
========================================*/
void GHPR::ConvertCloud(const pcl::PointCloud<pcl::PointXYZ> & vSCloud, pcl::PointCloud<pcl::PointXYZ> & vTCloud){

	//If the radius (m_fRadius) is still unknown
	if (!m_bComputeRadius){
		std::cout <<"The transpose radius of the point cloud has not been calculated!" << std::endl;
		return;
	}

	vTCloud.reserve(vSCloud.points.size());

	//the modulus of each point
	std::vector<float> vNormEachPoint;

	//get local coordinates based on the viewpoint 
	for (int i = 0; i != vSCloud.points.size(); ++i){

		pcl::PointXYZ oOnePoint;

		oOnePoint.x = vSCloud.points[i].x - m_oViewPInWorld.x;
		oOnePoint.y = vSCloud.points[i].y - m_oViewPInWorld.y;
		oOnePoint.z = vSCloud.points[i].z - m_oViewPInWorld.z;
		vTCloud.points.push_back(oOnePoint);
		
		//calculate the modulus length
		vNormEachPoint.push_back(NormVector(oOnePoint));
	}

	//transpose transformation based on radius 
	for (size_t i = 0; i != vTCloud.points.size(); i++){
		float numerator = 2 * (m_fRadius - vNormEachPoint[i]);
		vTCloud.points[i].x = vTCloud.points[i].x + numerator*vTCloud.points[i].x / vNormEachPoint[i];
		vTCloud.points[i].y = vTCloud.points[i].y + numerator*vTCloud.points[i].y / vNormEachPoint[i];
		vTCloud.points[i].z = vTCloud.points[i].z + numerator*vTCloud.points[i].z / vNormEachPoint[i];
	}

}


/*=======================================
IndexFromHulltoInput
Input: pHullVertices - vertices/points of convex hull
       m_pTransCloud - geometrically transformed points including raw points and viewpoint
Output: m_vHullInInputIdx - Correspondence relationship (index) of convex hull point set to transformed point set
Function: Establish the index from the convex hull point set to the transformed point set. 
Note that the order of the transformed point clouds is consistent with the input point clouds, 
so the index is intrinsically from the convex hull point set to the input point clouuds.
It means that the index value of the output is the the occluded point index in the input point cloud.
========================================*/
void GHPR::IndexFromHulltoInput(pcl::PointCloud<pcl::PointXYZ>::Ptr & pHullVertices){
	
	//get the raw index using the shortest distance approach
	pcl::KdTreeFLANN<pcl::PointXYZ> oLocalKdtree;
	oLocalKdtree.setInputCloud(m_pTransCloud);

	//searching based on kdtree
	for (size_t i = 0; i != pHullVertices->points.size(); i++){
		std::vector<int> kdindices;
		std::vector<float> kdistances;
		//find the first point as itself
		oLocalKdtree.nearestKSearch(pHullVertices->points[i], 1, kdindices, kdistances);
		//at least one point can be found
		m_vHullInInputIdx.push_back(kdindices[0]);
	}

}

/*=======================================
Compute
Input: pCloud - the point cloud to be proecssed / input point clouds
       bFindOccPoint - a flag indicating whether to output the occluded point index of the input point clouds
Output: m_pHullVertices - the vertices of convex hull. The convex hull is computed from transformed points (m_pTransCloud)
	    m_vHullPolygonIdxs - the face of convex hull (vertice indexes of a face)
		m_vHullInInputIdx (optional) - index of occluded point of input point clouds
Function: The main function of the class GHPR. It is to calculate the visibility of a viewpoint and a given point set.
Besides, it also obtains the rough model from the given perspective (viewpoint).
========================================*/
void GHPR::Compute(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pCloud, bool bIndexRelation){

	//convert point cloud
	ConvertCloud(pCloud);
	//get the viewpoint idx
	m_iViewWorldIdx = m_pTransCloud->points.size() - 1;

	//set the convex hull operater
	pcl::ConvexHull<pcl::PointXYZ> oConvexHull;
	
	//input point clouds of the convex hull
	oConvexHull.setInputCloud(m_pTransCloud);
	
	//set the dimension
	oConvexHull.setDimension(3);
	
	//pHullVertices - the vertices of convex hull
	//vPolygonIdxs - the face of convex hull (vertice indexes of a face)
	oConvexHull.reconstruct(*m_pHullVertices, m_vHullPolygonIdxs);	//10 - 50 ms

	//Check if a correspondence needs to be established from convex hull to input set with viewpoint
	//It is risk because the third - party (e.g.,PCL) implementation with differnet version would get a uniform point order 
	if (bIndexRelation){

		//construt a relationship between vertice and input point with viewpoint
		IndexFromHulltoInput(m_pHullVertices);

		//the index relationship has been established
		m_bToWorldIndex = true;

	}

}


void GHPR::Compute(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pCloud, std::mutex& reconstructLock, bool bIndexRelation){

	//convert point cloud
	ConvertCloud(pCloud);
	//get the viewpoint idx
	m_iViewWorldIdx = m_pTransCloud->points.size() - 1;

	//set the convex hull operater
	pcl::ConvexHull<pcl::PointXYZ> oConvexHull;
	
	//input point clouds of the convex hull
	oConvexHull.setInputCloud(m_pTransCloud);
	
	//set the dimension
	oConvexHull.setDimension(3);
	
	//pHullVertices - the vertices of convex hull
	//vPolygonIdxs - the face of convex hull (vertice indexes of a face)

	reconstructLock.lock();
	oConvexHull.reconstruct(*m_pHullVertices, m_vHullPolygonIdxs);	//10 - 50 ms
	reconstructLock.unlock();

	//Check if a correspondence needs to be established from convex hull to input set with viewpoint
	//It is risk because the third - party (e.g.,PCL) implementation with differnet version would get a uniform point order 
	if (bIndexRelation){

		//construt a relationship between vertice and input point with viewpoint
		IndexFromHulltoInput(m_pHullVertices);

		//the index relationship has been established
		m_bToWorldIndex = true;

	}

}

/*=======================================
GetOccludedIdx
Input: none
Output: vOccludedIdx - the index of occluded point
Function: output the index of the occluded point without viewpoint (*note*)
========================================*/
std::vector<int> GHPR::GetOccludedIdx(){

	//define output
	std::vector<int> vOccludedIdx;

	if (!m_bToWorldIndex){
		std::cout << "Correspondence has not been established, return empty output!" << std::endl;
		return vOccludedIdx;
	}

	for (int i = 0; i != m_vHullInInputIdx.size(); ++i){
		//Cut out the viewpoint
		if (m_vHullInInputIdx[i] != m_iViewWorldIdx)
			vOccludedIdx.push_back(m_vHullInInputIdx[i]);
	
	}

	return vOccludedIdx;

}

/*=======================================
GetConvexHullWorldIdx
Input: none
Output: vConvexHullInWorld - the index of the input points corresponding to the vertices of the convex hull
Function: output the index of each vertice on the convex hull corresponding to the input point set in the world coordinate.
Note that the output includes the viewpoint, which means that the corresponding point cloud must be the input points with viewpoint
e.g., pcloud.points.push_back(viewpoint)
========================================*/
std::vector<pcl::Vertices> GHPR::GetConvexHullWorldIdx(){
	
	//convex hull includes the viewpoint
	std::vector<pcl::Vertices> vConvexHullInWorld;

	if (!m_bToWorldIndex){
		std::cout << "Correspondence has not been established, return empty output!" << std::endl;
		return vConvexHullInWorld;
	}

	for (int i = 0; i != m_vHullPolygonIdxs.size(); ++i){
		//Cut out the viewpoint
		pcl::Vertices oOnePolyWorldIdx;

		for (int j = 0; j != m_vHullPolygonIdxs[i].vertices.size(); ++j){
			//get the local idx of each vertices
			int iOneIdx = m_vHullPolygonIdxs[i].vertices[j];
			//from local to world idx
			oOnePolyWorldIdx.vertices.push_back(m_vHullInInputIdx[iOneIdx]);

		}//end j

		vConvexHullInWorld.push_back(oOnePolyWorldIdx);

	}//end i

	return vConvexHullInWorld;

}

/*=======================================
ConstructSurfaceIdx
Input: none
Output: vModelHull - the index of the model surface vertices without face relative to viewpoint
Function: output the index of each vertice on the model surface corresponding to the input point set in the world coordinate.
Note that the output does not include the viewpoint
========================================*/
std::vector<pcl::Vertices> GHPR::ConstructSurfaceIdx(){

	//convex hull includes the viewpoint
	std::vector<pcl::Vertices> vModelHull;

	if (!m_bToWorldIndex){
		std::cout << "Correspondence has not been established, return empty output!" << std::endl;
		return vModelHull;
	}

	for (int i = 0; i != m_vHullPolygonIdxs.size(); ++i){
		//Cut out the viewpoint
		pcl::Vertices oOnePolyWorldIdx;
		bool bNonViewPoint = true;

		for (int j = 0; j != m_vHullPolygonIdxs[i].vertices.size(); ++j){
			//get the local idx of each vertices
			int iOneLocalIdx = m_vHullPolygonIdxs[i].vertices[j];
			int iOneWorldIdx = m_vHullInInputIdx[iOneLocalIdx];
			//from local to world idx
			oOnePolyWorldIdx.vertices.push_back(iOneWorldIdx);
			//check the face vertices are without viewpoint
			if (iOneWorldIdx == m_iViewWorldIdx)
				bNonViewPoint = false;

		}//end j

		if (bNonViewPoint)
			vModelHull.push_back(oOnePolyWorldIdx);

	}//end i

	return vModelHull;

}