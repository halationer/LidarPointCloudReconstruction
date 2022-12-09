////////////////////////////////////////////////////////
#include<mutex>
#include<pcl/surface/convex_hull.h>

void pcl_convex(
    const pcl::PointCloud<pcl::PointXYZI>& input_cloud,
    pcl::PointCloud<pcl::PointXYZI>& output_cloud,
    std::vector<pcl::Vertices>& output_vertices,
    std::mutex& convex_mutex,
    int id) 
{
    pcl::ConvexHull<pcl::PointXYZI> oConvexHull;
    oConvexHull.setInputCloud(input_cloud.makeShared());
    oConvexHull.setDimension(3);
    std::lock_guard<std::mutex> reconstruct_lock(convex_mutex);
    oConvexHull.reconstruct(output_cloud, output_vertices);
}

////////////////////////////////////////////////////////
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/convex_hull_3.h>
#include <vector>
#include <algorithm>

typedef CGAL::Exact_predicates_inexact_constructions_kernel  K;
typedef K::Point_3                                Point_3;
typedef CGAL::Surface_mesh<Point_3>               Surface_mesh;

void cgal_convex(
    const pcl::PointCloud<pcl::PointXYZI>& input_cloud,
    pcl::PointCloud<pcl::PointXYZI>& output_cloud,
    std::vector<pcl::Vertices>& output_vertices,
    std::mutex& convex_mutex,
    int id)
{
    clock_t start_time = clock(), end_time;
    std::cout << "#" << id << "sta: " << 1000.0 * start_time / CLOCKS_PER_SEC << std::endl;

    std::vector<Point_3> cgal_cloud;
    for(auto& point : input_cloud) 
        cgal_cloud.emplace_back(point.x, point.y, point.z);
    
    end_time = clock();
    std::cout << "#" << id << " pre: " << 1000.0 * (end_time - start_time) / CLOCKS_PER_SEC << "ms" << std::endl;


    Surface_mesh cgal_mesh;
    // for(int i = 0; i < 1000; ++i)
        CGAL::convex_hull_3(cgal_cloud.begin(), cgal_cloud.end(), cgal_mesh);
        // std::sort(cgal_cloud.begin(), cgal_cloud.end(), [](const Point_3& a, const Point_3& b) {return a.x() > b.x();});

    end_time = clock();
    std::cout << "#" << id << " cur: " << 1000.0 * (end_time - start_time) / CLOCKS_PER_SEC << "ms" << std::endl;
    

    for(auto& point : cgal_mesh.points()) {
        pcl::PointXYZI pcl_point;
        pcl_point.x = point.x();
        pcl_point.y = point.y();
        pcl_point.z = point.z();
        output_cloud.push_back(pcl_point);
    }

    for(auto& face : cgal_mesh.faces()) {
        CGAL::Vertex_around_face_iterator<Surface_mesh> vbegin, vend;
        pcl::Vertices pcl_vertice;
        for(boost::tie(vbegin, vend) = cgal_mesh.vertices_around_face(cgal_mesh.halfedge(face)); 
            vbegin != vend;
            ++vbegin) 
        {
            auto vertex = *vbegin;
            pcl_vertice.vertices.push_back(*reinterpret_cast<int*>(&vertex));
        }
        output_vertices.push_back(pcl_vertice);
    }

    end_time = clock();
    std::cout << "#" << id << " aft: " << 1000.0 * (end_time - start_time) / CLOCKS_PER_SEC << "ms" << std::endl;
}