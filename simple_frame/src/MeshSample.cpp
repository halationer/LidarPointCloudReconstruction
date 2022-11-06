#include "MeshSample.h"

// fix the problem of **Error: no override found for 'vtkPolyDataMapper'.
#include <GL/glut.h>
#include <vtkAutoInit.h> 
VTK_MODULE_INIT(vtkRenderingOpenGL);
// fix end

MeshSample::MeshSample() : m_iSamPointNum(10000), m_bOutNormals(true), m_fLeafSize(0.1f),
						   m_pSampedCloud(new pcl::PointCloud<pcl::PointNormal>)
{
}

MeshSample::~MeshSample()
{
}

void MeshSample::SetSamplePointNum(int iSamPointNum)
{

	m_iSamPointNum = iSamPointNum;
}

// whether output normal of sampled point
void MeshSample::SetOutNormals(bool bOutNormals)
{

	m_bOutNormals = bOutNormals;
}

// the length of voxelization
void MeshSample::SetLeafSize(float fLeafSize)
{

	m_fLeafSize = fLeafSize;
}

double MeshSample::uniform_deviate(int seed)
{
	double ran = seed * (1.0 / (RAND_MAX + 1.0));
	return ran;
}

void MeshSample::randomPointTriangle(float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3, Eigen::Vector4f &p)
{
	float r1 = static_cast<float>(uniform_deviate(rand()));
	float r2 = static_cast<float>(uniform_deviate(rand()));
	float r1sqr = std::sqrt(r1);
	float OneMinR1Sqr = (1 - r1sqr);
	float OneMinR2 = (1 - r2);
	a1 *= OneMinR1Sqr;
	a2 *= OneMinR1Sqr;
	a3 *= OneMinR1Sqr;
	b1 *= OneMinR2;
	b2 *= OneMinR2;
	b3 *= OneMinR2;
	c1 = r1sqr * (r2 * c1 + b1) + a1;
	c2 = r1sqr * (r2 * c2 + b2) + a2;
	c3 = r1sqr * (r2 * c3 + b3) + a3;
	p[0] = c1;
	p[1] = c2;
	p[2] = c3;
	p[3] = 0;
}

void MeshSample::randPSurface(vtkPolyData *polydata,
							  std::vector<double> *cumulativeAreas,
							  double totalArea,
							  Eigen::Vector4f &p,
							  bool calcNormal,
							  Eigen::Vector3f &n)
{
	float r = static_cast<float>(uniform_deviate(rand()) * totalArea);

	std::vector<double>::iterator low = std::lower_bound(cumulativeAreas->begin(), cumulativeAreas->end(), r);
	vtkIdType el = vtkIdType(low - cumulativeAreas->begin());

	double A[3], B[3], C[3];
	vtkIdType npts = 0;
	vtkIdType *ptIds = NULL;
	polydata->GetCellPoints(el, npts, ptIds);
	polydata->GetPoint(ptIds[0], A);
	polydata->GetPoint(ptIds[1], B);
	polydata->GetPoint(ptIds[2], C);

	if (calcNormal)
	{
		// OBJ: Vertices are stored in a counter-clockwise order by default
		Eigen::Vector3f v1 = Eigen::Vector3f(A[0], A[1], A[2]) - Eigen::Vector3f(C[0], C[1], C[2]);
		Eigen::Vector3f v2 = Eigen::Vector3f(B[0], B[1], B[2]) - Eigen::Vector3f(C[0], C[1], C[2]);
		n = v1.cross(v2);
		n.normalize();
	}

	randomPointTriangle(float(A[0]), float(A[1]), float(A[2]),
						float(B[0]), float(B[1]), float(B[2]),
						float(C[0]), float(C[1]), float(C[2]), p);
}

/*=======================================
uniform_sampling
Input: polydata 	- brief
Input: n_samples 	- number of sample points
Input: calc_normal 	- whether to calc the normal?
Output: cloud_out 	- brief
Function:
========================================*/
void MeshSample::uniform_sampling(vtkSmartPointer<vtkPolyData> polydata,
								  size_t n_samples,
								  bool calc_normal,
								  pcl::PointCloud<pcl::PointNormal> &cloud_out)
{
	polydata->BuildCells();
	vtkSmartPointer<vtkCellArray> cells = polydata->GetPolys();

	double p1[3], p2[3], p3[3], totalArea = 0;
	std::vector<double> cumulativeAreas(cells->GetNumberOfCells(), 0);
	size_t i = 0;
	vtkIdType npts = 0, *ptIds = NULL;
	for (cells->InitTraversal(); cells->GetNextCell(npts, ptIds); i++)
	{
		polydata->GetPoint(ptIds[0], p1);
		polydata->GetPoint(ptIds[1], p2);
		polydata->GetPoint(ptIds[2], p3);
		totalArea += vtkTriangle::TriangleArea(p1, p2, p3);
		cumulativeAreas[i] = totalArea;
	}

	cloud_out.points.resize(n_samples);
	cloud_out.width = static_cast<pcl::uint32_t>(n_samples);
	cloud_out.height = 1;

	for (i = 0; i < n_samples; i++)
	{
		Eigen::Vector4f p;
		Eigen::Vector3f n;
		randPSurface(polydata, &cumulativeAreas, totalArea, p, calc_normal, n);
		cloud_out.points[i].x = p[0];
		cloud_out.points[i].y = p[1];
		cloud_out.points[i].z = p[2];
		if (calc_normal)
		{
			cloud_out.points[i].normal_x = n[0];
			cloud_out.points[i].normal_y = n[1];
			cloud_out.points[i].normal_z = n[2];
		}
	}
}

/* ---[ */
void MeshSample::SampleMeshToPoints(const pcl::PolygonMesh &oMesh)
{

	// get the vtk type data
	vtkSmartPointer<vtkPolyData> vtkPolydata1 = vtkSmartPointer<vtkPolyData>::New();
	pcl::io::mesh2vtk(oMesh, vtkPolydata1);

	// make sure that the polygons are triangles!
	vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
#if VTK_MAJOR_VERSION < 6
	triangleFilter->SetInput(polydata1);
#else
	triangleFilter->SetInputData(vtkPolydata1);
#endif

	triangleFilter->Update();

	vtkSmartPointer<vtkPolyDataMapper> triangleMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	triangleMapper->SetInputConnection(triangleFilter->GetOutputPort());
	triangleMapper->Update();
	vtkPolydata1 = triangleMapper->GetInput();

	// make sampling
	uniform_sampling(vtkPolydata1, m_iSamPointNum, m_bOutNormals, *m_pSampedCloud);
}

void MeshSample::OutputSampledClouds(pcl::PointCloud<pcl::PointXYZ> &vCloud)
{

	vCloud.clear();

	// Strip uninitialized normals from cloud:
	pcl::copyPointCloud(*m_pSampedCloud, vCloud);

	savePLYFileASCII("sampled_clouds.ply", *m_pSampedCloud);
}

void MeshSample::OutputVoxelizedClouds(pcl::PointCloud<pcl::PointNormal> &voxel_cloud)
{

	// Voxelgrid
	VoxelGrid<PointNormal> grid_;
	grid_.setInputCloud(m_pSampedCloud);
	grid_.setLeafSize(m_fLeafSize, m_fLeafSize, m_fLeafSize);

	// sampled by voxelization
	grid_.filter(voxel_cloud);

	savePLYFileASCII("sampled_clouds.ply", voxel_cloud);
}

void MeshSample::OutputVoxelizedClouds(pcl::PointCloud<pcl::PointXYZ> &vVoxelCloud)
{

	// Voxelgrid
	VoxelGrid<PointNormal> grid_;
	grid_.setInputCloud(m_pSampedCloud);
	grid_.setLeafSize(m_fLeafSize, m_fLeafSize, m_fLeafSize);

	// sampled by voxelization
	pcl::PointCloud<pcl::PointNormal> voxeln_cloud;
	grid_.filter(voxeln_cloud);

	pcl::copyPointCloud(voxeln_cloud, vVoxelCloud);

	savePLYFileASCII("sampled_clouds.ply", vVoxelCloud);
}



void MeshSample::GetAdditionalPointCloud(const std::vector<std::vector<pcl::PointXYZI>>& in_cloud, std::vector<std::vector<pcl::PointXYZI>>& out_cloud) {
    

}

void MeshSample::GetAdditionalPointCloud(const pcl::PointCloud<pcl::PointXYZI>& in_face_center, const std::vector<float>& in_face_weight, 
    const Eigen::MatrixXf& in_face_normal, pcl::PointCloud<pcl::PointNormal>& out_cloud, pcl::PointCloud<pcl::PointXYZI>& out_display_cloud) {
    
    for(int i = 0; i < in_face_center.size(); ++i) {

        pcl::PointNormal oPoint;
        oPoint.x = in_face_center.points[i].x;
        oPoint.y = in_face_center.points[i].y;
        oPoint.z = in_face_center.points[i].z;
        oPoint.normal_x = in_face_normal.row(i).x();
        oPoint.normal_y = in_face_normal.row(i).y();
        oPoint.normal_z = in_face_normal.row(i).z();
        oPoint.data_n[3] = in_face_weight[i];
        out_cloud.push_back(oPoint);

        pcl::PointXYZI oDisplayPoint;
        oDisplayPoint.x = oPoint.x;
        oDisplayPoint.y = oPoint.y;
        oDisplayPoint.z = oPoint.z;
        oDisplayPoint.intensity = oPoint.data_n[3];
        out_display_cloud.push_back(oDisplayPoint);
    }

}

template<class T>
void MeshSample::GetAdditionalPointCloud(const pcl::PointCloud<T>& in_cloud, const std::vector<pcl::Vertices>& in_polygons, const std::vector<float>& in_face_weight,  
    const Eigen::MatrixXf& in_face_normal, pcl::PointCloud<pcl::PointNormal>& out_cloud, pcl::PointCloud<pcl::PointXYZI>& out_display_cloud) {

    //固定采样数量
    int iSamplePointNum = 2;
    for(int i = 0; i < in_polygons.size(); ++i) {

        // TODO: 根据面片边长或者面积动态决定采样数量
    
        Eigen::Vector4f point;
        const T& a = in_cloud[in_polygons[i].vertices[0]];
        const T& b = in_cloud[in_polygons[i].vertices[1]];
        const T& c = in_cloud[in_polygons[i].vertices[2]];

        for(int j = 0; j < iSamplePointNum; ++j) {

            MeshSample::randomPointTriangle(a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z, point);

            pcl::PointNormal oPoint;
            oPoint.x = point.x();
            oPoint.y = point.y();
            oPoint.z = point.z();
            oPoint.normal_x = in_face_normal.row(i).x();
            oPoint.normal_y = in_face_normal.row(i).y();
            oPoint.normal_z = in_face_normal.row(i).z();
            oPoint.data_n[3] = -in_face_weight[i];
            if(oPoint.data_n[3] > 0.1)
                out_cloud.push_back(oPoint);

            pcl::PointXYZI oDisplayPoint;
            oDisplayPoint.x = oPoint.x;
            oDisplayPoint.y = oPoint.y;
            oDisplayPoint.z = oPoint.z;
            oDisplayPoint.intensity = oPoint.data_n[3];
            if(oPoint.data_n[3] > 0.1)
                out_display_cloud.push_back(oDisplayPoint);
        }

    }
}
template void MeshSample::GetAdditionalPointCloud(const pcl::PointCloud<pcl::PointNormal>& in_cloud, const std::vector<pcl::Vertices>& in_polygons, const std::vector<float>& in_face_weight,  
    const Eigen::MatrixXf& in_face_normal, pcl::PointCloud<pcl::PointNormal>& out_cloud, pcl::PointCloud<pcl::PointXYZI>& out_display_cloud);
template void MeshSample::GetAdditionalPointCloud(const pcl::PointCloud<pcl::PointXYZI>& in_cloud, const std::vector<pcl::Vertices>& in_polygons, const std::vector<float>& in_face_weight,  
	const Eigen::MatrixXf& in_face_normal, pcl::PointCloud<pcl::PointNormal>& out_cloud, pcl::PointCloud<pcl::PointXYZI>& out_display_cloud);



template<class T>
void MeshSample::GetAdditionalPointCloud(const pcl::PointCloud<T>& in_cloud, const std::vector<pcl::Vertices>& in_polygons, const std::vector<Confidence>& in_face_weight,  
    const Eigen::MatrixXf& in_face_normal, pcl::PointCloud<pcl::PointNormal>& out_cloud, pcl::PointCloud<pcl::PointXYZI>& out_display_cloud) {

    //固定采样数量
    constexpr int iSamplePointNum = 2;
    for(int i = 0; i < in_polygons.size(); ++i) {

    	if(in_face_weight[i].normal_confidence > 0.1) {

			Eigen::Vector4f point;
			const T& a = in_cloud[in_polygons[i].vertices[0]];
			const T& b = in_cloud[in_polygons[i].vertices[1]];
			const T& c = in_cloud[in_polygons[i].vertices[2]];

        	// TODO: 根据面片边长或者面积动态决定采样数量
			for(int j = 0; j < iSamplePointNum; ++j) {

				MeshSample::randomPointTriangle(a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z, point);

				pcl::PointNormal oPoint;
				oPoint.x = point.x();
				oPoint.y = point.y();
				oPoint.z = point.z();
				oPoint.normal_x = in_face_normal.row(i).x();
				oPoint.normal_y = in_face_normal.row(i).y();
				oPoint.normal_z = in_face_normal.row(i).z();
				oPoint.data_n[3] = in_face_weight[i].depth_confidence;
				out_cloud.push_back(oPoint);
					
				pcl::PointXYZI oDisplayPoint;
				oDisplayPoint.x = oPoint.x;
				oDisplayPoint.y = oPoint.y;
				oDisplayPoint.z = oPoint.z;
				oDisplayPoint.intensity = oPoint.data_n[3];
				out_display_cloud.push_back(oDisplayPoint);
			}
		}
	}
}
template void MeshSample::GetAdditionalPointCloud(const pcl::PointCloud<pcl::PointNormal>& in_cloud, const std::vector<pcl::Vertices>& in_polygons, const std::vector<Confidence>& in_face_weight,  
    const Eigen::MatrixXf& in_face_normal, pcl::PointCloud<pcl::PointNormal>& out_cloud, pcl::PointCloud<pcl::PointXYZI>& out_display_cloud);
template void MeshSample::GetAdditionalPointCloud(const pcl::PointCloud<pcl::PointXYZI>& in_cloud, const std::vector<pcl::Vertices>& in_polygons, const std::vector<Confidence>& in_face_weight,  
	const Eigen::MatrixXf& in_face_normal, pcl::PointCloud<pcl::PointNormal>& out_cloud, pcl::PointCloud<pcl::PointXYZI>& out_display_cloud);