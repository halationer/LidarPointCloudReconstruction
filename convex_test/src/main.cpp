#include <string>
#include <ctime>
#include <iostream>
#include <random>
#include <thread>
#include <mutex>
#include <unordered_map>
#include <sys/time.h>

//ros related
#include <ros/ros.h>
#include "all_convex.h"

//pcl related
#include "pcl_ros/transforms.h"  

#include <pcl/io/pcd_io.h>         
#include <pcl/io/ply_io.h>         
#include <pcl/point_types.h> 
#include <pcl_conversions/pcl_conversions.h>

//polygon related
#include <visualization_msgs/Marker.h>

namespace std {
	const char* format_white = "\033[0m";
	const char* format_yellow =  "\033[33m";
	const char* format_red = "\033[31m";
}

void PublishMesh(const pcl::PointCloud<pcl::PointXYZI>& convex_point, const std::vector<pcl::Vertices>& convex_triangle, visualization_msgs::Marker& oMeshMsgs);


int main(int argc, char** argv){

    ros::init(argc, argv, "convex_test");
    ros::NodeHandle node;
    ros::NodeHandle privateNode("~");

    bool MultiThread = false;
    privateNode.param("multi_thread", MultiThread, false);

    int ThreadNum = 8;
    privateNode.param("thread_num", ThreadNum, 8);

    std::string Version = "cgal";
    privateNode.param("version", Version, std::string("cgal"));

    enum version_val { PCL, CGAL, ESAY3D, };
    std::unordered_map<std::string, int> version_ref;
    version_ref["cgal"] = version_val::CGAL;
    version_ref["pcl"] = version_val::PCL;
    version_ref["esay3d"] = version_val::ESAY3D;
    version_val iVersion = (version_val)version_ref[Version];

    std::cout << "convex_test node start!" << std::endl;



    // #############################
    // #### create point cloud #####
    // #############################

    std::vector<pcl::PointCloud<pcl::PointXYZI>> point_cloud(ThreadNum);
    pcl::PointCloud<pcl::PointXYZI> all_point;
    std::default_random_engine e(time(0));
    std::uniform_real_distribution<float> uniform_rand(20, 80);

    int cube_seed[24] = {
         1,  1,  1,
         1,  1, -1,
         1, -1,  1,
         1, -1, -1,
        -1,  1,  1,
        -1,  1, -1,
        -1, -1,  1,
        -1, -1, -1,
    };

    for(int cube_pos = 0; cube_pos < ThreadNum; ++cube_pos) {

        for(int point_index = 0; point_index < 3000; ++point_index) {

            pcl::PointXYZI point;

            point.x = uniform_rand(e) * cube_seed[cube_pos * 3];
            point.y = uniform_rand(e) * cube_seed[cube_pos * 3 + 1];
            point.z = uniform_rand(e) * cube_seed[cube_pos * 3 + 2];
            point.intensity = cube_pos;

            point_cloud[cube_pos].push_back(point);
        }

        all_point += point_cloud[cube_pos];
    }

    
    sensor_msgs::PointCloud2 vCloudData;
	pcl::toROSMsg(all_point, vCloudData);
	vCloudData.header.frame_id = "/map";
	vCloudData.header.stamp = ros::Time::now();
    ros::Publisher publisher1 = privateNode.advertise<sensor_msgs::PointCloud2>("/convex_test", 1, true);
	publisher1.publish(vCloudData);



    // #############################
    // ### calculate convex hull ###
    // #############################

    std::vector<pcl::PointCloud<pcl::PointXYZI>> convex_point(ThreadNum);
    std::vector<std::vector<pcl::Vertices>> convex_triangle(ThreadNum);

    std::vector<std::function<void(void)>> convex_function(ThreadNum);
    std::vector<std::thread> convex_thread;
    std::mutex convex_mutex;
    for(int cube_pos = 0; cube_pos < ThreadNum; ++cube_pos) {

        switch(iVersion)
        {
            case CGAL:
                convex_function[cube_pos] = [&, cube_pos]() {
                    cgal_convex(point_cloud[cube_pos], convex_point[cube_pos], convex_triangle[cube_pos], convex_mutex, cube_pos);
                };  
                break;

            case PCL:
                convex_function[cube_pos] = [&, cube_pos]() {
                    pcl_convex(point_cloud[cube_pos], convex_point[cube_pos], convex_triangle[cube_pos], convex_mutex, cube_pos);
                };  
                break;
        
        }
    }

    clock_t start_time = clock();
    struct timeval start;
    struct timeval end;
    gettimeofday(&start, NULL);

	if(!MultiThread) {

        for(int cube_pos = 0; cube_pos < ThreadNum; ++cube_pos)  
            convex_function[cube_pos]();
    }
    else {

        for(int cube_pos = 0; cube_pos < ThreadNum; ++cube_pos)
            convex_thread.emplace_back(convex_function[cube_pos]);

        for(auto& thread : convex_thread) thread.join();
    }

    gettimeofday(&end,NULL);
    double parallel_time = (end.tv_sec - start.tv_sec) * 1000.0 +(end.tv_usec - start.tv_usec) * 0.001;
    clock_t reconstruct_time = 1000.0 * (clock() - start_time) / CLOCKS_PER_SEC;
    std::cout << std::format_yellow << "reconstruct time:" << reconstruct_time << "ms" << std::format_white << std::endl;
    std::cout << std::format_yellow << "reconstruct time2:" << parallel_time << "ms" << std::format_white << std::endl;

    pcl::PointCloud<pcl::PointXYZI> all_convex_point;
    for(int cube_pos = 0; cube_pos < ThreadNum; ++cube_pos) 
        all_convex_point += convex_point[cube_pos];

    pcl::PointCloud<pcl::PointXYZI> out_convex_point;
    std::vector<pcl::Vertices> out_convex_triangle;
    pcl::ConvexHull<pcl::PointXYZI> oConvexHull;
    oConvexHull.setInputCloud(all_convex_point.makeShared());
    oConvexHull.setDimension(3);
    oConvexHull.reconstruct(out_convex_point, out_convex_triangle);
    std::cout << "point_num: " << out_convex_point.size() << std::endl;
    std::cout << "vertice_num: " << out_convex_triangle.size() << std::endl;

  	//new a visual message
	visualization_msgs::Marker oMeshMsgs;
    PublishMesh(convex_point[0], convex_triangle[0], oMeshMsgs);
    ros::Publisher publisher2 = privateNode.advertise<visualization_msgs::Marker>("/convex_mesh", 1, true);
	publisher2.publish(oMeshMsgs);

  	//new a visual message
	visualization_msgs::Marker oMeshMsgsAll;
    PublishMesh(out_convex_point, out_convex_triangle, oMeshMsgsAll);
    ros::Publisher publisher3 = privateNode.advertise<visualization_msgs::Marker>("/convex_mesh_all", 1, true);
	publisher3.publish(oMeshMsgsAll);


    std::cout << "convex_test node finish!" << std::endl;
    ros::spin();

    return 0;
}


void PublishMesh(const pcl::PointCloud<pcl::PointXYZI>& convex_point, const std::vector<pcl::Vertices>& convex_triangle, visualization_msgs::Marker& oMeshMsgs) {
  	
	
	//define header of message
	oMeshMsgs.header.frame_id = "/map";
	oMeshMsgs.header.stamp = ros::Time::now();
	oMeshMsgs.type = visualization_msgs::Marker::TRIANGLE_LIST;
	oMeshMsgs.action = visualization_msgs::Marker::ADD;

	oMeshMsgs.scale.x = 1.0;
	oMeshMsgs.scale.y = 1.0;
	oMeshMsgs.scale.z = 1.0;

	oMeshMsgs.pose.position.x = 0.0;
	oMeshMsgs.pose.position.y = 0.0;
	oMeshMsgs.pose.position.z = 0.0;

	oMeshMsgs.pose.orientation.x = 0.0;
	oMeshMsgs.pose.orientation.y = 0.0;
	oMeshMsgs.pose.orientation.z = 0.0;
	oMeshMsgs.pose.orientation.w = 1.0;

	std_msgs::ColorRGBA color;
	color.a = 1.0;
	color.r = 1.0;
	color.g = 1.0;
	color.b = 1.0;
    oMeshMsgs.color = color;

	//convert to publishable message
	for (int k = 0; k < convex_triangle.size(); ++k){

        for(int i = 0; i < 3; ++i) {
            
            //temp point
            geometry_msgs::Point oPTemp;
            oPTemp.x = convex_point[convex_triangle[k].vertices.at(i)].x;
            oPTemp.y = convex_point[convex_triangle[k].vertices.at(i)].y;
            oPTemp.z = convex_point[convex_triangle[k].vertices.at(i)].z;

            //color
            oMeshMsgs.points.push_back(oPTemp);
        }
	}//end k

    std::cout << "publish_mesh_size: " << oMeshMsgs.points.size() << std::endl;

}