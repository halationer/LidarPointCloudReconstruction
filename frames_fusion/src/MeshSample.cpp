#include "MeshSample.h"

MeshSample::MeshSample() : m_iSamPointNum(10000), m_bOutNormals(true), m_fLeafSize(0.1f),
											m_pSampedCloud(new pcl::PointCloud<pcl::PointNormal>){

}

MeshSample::~MeshSample(){


}


void MeshSample::SetSamplePointNum(int iSamPointNum){

	m_iSamPointNum = iSamPointNum;

}

//whether output normal of sampled point
void MeshSample::SetOutNormals(bool bOutNormals){

	m_bOutNormals = bOutNormals;

}


//the length of voxelization
void MeshSample::SetLeafSize(float fLeafSize){

	m_fLeafSize = fLeafSize;

}

double MeshSample::uniform_deviate(int seed)
{
	double ran = seed * (1.0 / (RAND_MAX + 1.0));
	return ran;
}



void MeshSample::randomPointTriangle(float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3, Eigen::Vector4f& p)
{
	float r1 = static_cast<float> (uniform_deviate(rand()));
	float r2 = static_cast<float> (uniform_deviate(rand()));
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

void MeshSample::randPSurface(vtkPolyData * polydata, std::vector<double> * cumulativeAreas, double totalArea, Eigen::Vector4f& p, bool calcNormal, Eigen::Vector3f& n)
{
	float r = static_cast<float> (uniform_deviate(rand()) * totalArea);

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

void MeshSample::uniform_sampling(vtkSmartPointer<vtkPolyData> polydata, size_t n_samples, bool calc_normal, pcl::PointCloud<pcl::PointNormal> & cloud_out)
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
	cloud_out.width = static_cast<pcl::uint32_t> (n_samples);
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
void MeshSample::SampleMeshToPoints(const pcl::PolygonMesh & oMesh){

	//get the vtk type data
	vtkSmartPointer<vtkPolyData> polydata1 = vtkSmartPointer<vtkPolyData>::New();
	pcl::io::mesh2vtk(oMesh, polydata1);


	//make sure that the polygons are triangles!
	vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
#if VTK_MAJOR_VERSION < 6
	triangleFilter->SetInput(polydata1);
#else
	triangleFilter->SetInputData(polydata1);
#endif

	triangleFilter->Update();

	vtkSmartPointer<vtkPolyDataMapper> triangleMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	triangleMapper->SetInputConnection(triangleFilter->GetOutputPort());
	triangleMapper->Update();
	polydata1 = triangleMapper->GetInput();

	//make sampling
	uniform_sampling(polydata1, m_iSamPointNum, m_bOutNormals, *m_pSampedCloud);

}



void MeshSample::OutputSampledClouds(pcl::PointCloud<pcl::PointXYZ> & vCloud){

	vCloud.clear();

	// Strip uninitialized normals from cloud:
	pcl::copyPointCloud(*m_pSampedCloud, vCloud);

	savePLYFileASCII("sampled_clouds.ply", *m_pSampedCloud);

}


void MeshSample::OutputVoxelizedClouds(pcl::PointCloud<pcl::PointNormal> & voxel_cloud){

	// Voxelgrid
	VoxelGrid<PointNormal> grid_;
	grid_.setInputCloud(m_pSampedCloud);
	grid_.setLeafSize(m_fLeafSize, m_fLeafSize, m_fLeafSize);

	//sampled by voxelization
	grid_.filter(voxel_cloud);

	savePLYFileASCII("sampled_clouds.ply", voxel_cloud);

}

void MeshSample::OutputVoxelizedClouds(pcl::PointCloud<pcl::PointXYZ> & vVoxelCloud){

	// Voxelgrid
	VoxelGrid<PointNormal> grid_;
	grid_.setInputCloud(m_pSampedCloud);
	grid_.setLeafSize(m_fLeafSize, m_fLeafSize, m_fLeafSize);

	//sampled by voxelization
	pcl::PointCloud<pcl::PointNormal> voxeln_cloud;
	grid_.filter(voxeln_cloud);

	pcl::copyPointCloud(voxeln_cloud, vVoxelCloud);

	savePLYFileASCII("sampled_clouds.ply", vVoxelCloud);

}



