#include "SimpleRecon.h"
#include "OutputUtils.h"

#include <map>
#include <algorithm>
#include <assert.h>
#include <sstream>

/*************************************************
Function: SimpleRecon
Description: constrcution function for SimpleRecon class
Calls: all member functions
Called By: main function of project
Table Accessed: none
Table Updated: none
Input: node - a ros node class
     nodeHandle - a private ros node class
*************************************************/
SimpleRecon::SimpleRecon(ros::NodeHandle & node, ros::NodeHandle & nodeHandle):super(node, nodeHandle){

}

/*************************************************
Function: LazyLoading
Description: read ros params, subcribe and advertise ros topics
Called By: main function of project
*************************************************/
void SimpleRecon::LazyLoading() {

    std::cout << "Load Simple Reconstruction..." << std::endl;

	//read parameters
	ReadLaunchParams(nodeHandle);

	//***subscriber related*** 
	//subscribe (hear) the odometry information (trajectory)
	m_oOdomSuber = nodeHandle.subscribe(m_sInOdomTopic, 2, &SimpleRecon::HandleTrajectory, this);	//记录运动信息到 m_vOdomHistory 的循环数组之中，并且 m_iTrajCount++

	//subscribe (hear) the point cloud topic 
	m_oCloudSuber = nodeHandle.subscribe(m_sInCloudTopic, 1, &SimpleRecon::HandlePointClouds, this);	//在m_vMapPCN记录法向点集，发布Mesh主题

	//***publisher related*** 
	//publish point cloud after processing
	m_oCloudPublisher = nodeHandle.advertise<sensor_msgs::PointCloud2>(m_sOutCloudTopic, 2, true);	//发布含有法向量的点云

  	//publish polygon constructed from one frame point cloud
	m_oMeshPublisher = nodeHandle.advertise<visualization_msgs::Marker>(m_sOutMeshTopic, 1);		//在接受到点云重建完之后， 被 PublishMeshs() 函数调用

    m_oAdditionalPointPublisher = nodeHandle.advertise<sensor_msgs::PointCloud2>(m_sAdditionalPointTopic, 1, true); //发布补充的点云
}

/*************************************************
Function: ~SimpleRecon
Description: deconstrcution function for SimpleRecon class
*************************************************/
SimpleRecon::~SimpleRecon() {

}

/*************************************************
Function: ReadLaunchParams
Description: read setting params in the launch file or command args
*************************************************/
bool SimpleRecon::ReadLaunchParams(ros::NodeHandle & nodeHandle) {

    super::ReadLaunchParams(nodeHandle);
    
    nodeHandle.param("lidar_type", m_iLidarType, static_cast<int>(LIDAR_TYPE::__16_LINES__));

    nodeHandle.param("show_confidence", m_bShowConfidence, false);

    nodeHandle.param("combine_clouds", m_bCombineTwoClouds, true);

    return true;
}

/*************************************************
Function: HandlePointClouds
Description: a callback function: single frame reconstruction, publish meshes and normal points
*************************************************/
void SimpleRecon::HandlePointClouds(const sensor_msgs::PointCloud2 & vLaserData)
{

	if (!(m_iPCFrameCount % m_iFrameSmpNum)){ //根据帧采样频率记录

		std::cout << "Now frame count is: " << m_iPCFrameCount << ";\t"
			<< "header is: {" << vLaserData.header << "}";
		m_iTotalFrameNum = vLaserData.header.seq;

		//开始算法计时
		clock_t start_time = clock();
		pcl::PointCloud<pcl::PointXYZI>::Ptr pRawCloud(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::fromROSMsg(vLaserData, *pRawCloud);

        //获取中心视点
		pcl::PointXYZI oCurrentViewP;
		if (m_vOdomHistory.size()){
			oCurrentViewP = ComputeQueryTraj(vLaserData.header.stamp);	//当前点云对应的观测位置（Odom与frame并非一一对应，因此需要计算插值）
		}else{
			std::cout << std::format_red << " Error: No odom matched!" << std::format_white << std::endl;
			return;
		}

        //根据intensity信息组织点云结构,将点云整合成二维数组（有序不等长）的结构
        std::vector<pcl::PointCloud<pcl::PointXYZI>> vCloudList(m_iLidarType);
        SimpleRecon::ReOrganizePoints(*pRawCloud, vCloudList);

        //将点云组织成类似于图片的结构，二维数组（有序等长，没点的地方空白填充）
        std::vector<std::vector<pcl::PointXYZI>> vCloudVector;
        SimpleRecon::FromCloudListToPointVector(vCloudList, vCloudVector, m_iLidarType);

        //使用邻接生成法生成网格 7ms
        std::vector<pcl::Vertices> vMeshPolygons;
        pcl::PointCloud<pcl::PointXYZI> vMeshCloud;
        SimpleRecon::FromPointVectorToMesh(vCloudVector, vMeshCloud, vMeshPolygons);

        //剔除网格中与射线夹角接近直角的面 6ms
        //TODO：这一步不仅根据角度对面片进行剔除，还得到了面片的置信度，这使得点云补全时使用了该面片的置信度，整体问题在于置信度的规范不统一
        pcl::PointCloud<pcl::PointNormal> vMeshCloudWithNormal;
        std::vector<pcl::Vertices> vNewMeshPolygons;
        pcl::PointCloud<pcl::PointXYZI> vFaceCenter;
        Eigen::MatrixXf oMatNormal;
        std::vector<Confidence> vFaceWeight;
        SimpleRecon::RemoveMeshFaces(oCurrentViewP, vMeshCloud, vMeshPolygons, vMeshCloudWithNormal, vNewMeshPolygons, vFaceCenter, oMatNormal, vFaceWeight);

        // 保存网格结果
        // SimpleRecon::SaveMesh(vMeshCloudWithNormal, vNewMeshPolygons);

        // 点云补全
        pcl::PointCloud<pcl::PointNormal> vAdditionalPoints;
        pcl::PointCloud<pcl::PointXYZI> vDisplayAdditionalPoints;
        // SimpleRecon::GetAdditionalPointCloud(vFaceCenter, vFaceWeight, oMatNormal, vAdditionalPoints, vDisplayAdditionalPoints);
        MeshSample::GetAdditionalPointCloud(vMeshCloudWithNormal, vNewMeshPolygons, vFaceWeight, oMatNormal, vAdditionalPoints, vDisplayAdditionalPoints);

        // 发布点云/网格的可视化消息
        PublishPointCloud(vDisplayAdditionalPoints, m_oAdditionalPointPublisher);
        PublishMesh(vMeshCloudWithNormal, vNewMeshPolygons);

        // TODO： 合并两种点云的发布（为了防止漏帧）
        if(m_bCombineTwoClouds) {

            pcl::PointCloud<pcl::PointNormal> vCombinedCloud;
            vCombinedCloud += vMeshCloudWithNormal;
            vCombinedCloud += vAdditionalPoints;

            // TODO: 添加中心视点，方便多帧进程识别
            pcl::PointNormal oViewPoint;
            oViewPoint.x = oCurrentViewP.x;
            oViewPoint.y = oCurrentViewP.y;
            oViewPoint.z = oCurrentViewP.z;
            oViewPoint.curvature = -1;      //识别码
            oViewPoint.normal_x = vMeshCloudWithNormal.size(); //识别码2
            vCombinedCloud.push_back(oViewPoint);

            //一起发布消息
            PublishPointCloud(vCombinedCloud);
        }
        else {

            // TODO: 添加中心视点，方便多帧进程识别
            pcl::PointNormal oViewPoint;
            oViewPoint.x = oCurrentViewP.x;
            oViewPoint.y = oCurrentViewP.y;
            oViewPoint.z = oCurrentViewP.z;
            oViewPoint.curvature = -1;      //识别码
            vMeshCloudWithNormal.push_back(oViewPoint);
            vAdditionalPoints.push_back(oViewPoint);

            //分别发布消息
            PublishPointCloud(vMeshCloudWithNormal, m_oCloudPublisher);
            vAdditionalPoints.is_dense = false;
            PublishPointCloud(vAdditionalPoints, m_oCloudPublisher);
        }

		//结束算法计时并记录执行时间
		clock_t frame_reconstruct_time = 1000.0 * (clock() - start_time) / CLOCKS_PER_SEC;
		std::cout << ";\ttime:" << frame_reconstruct_time << "ms" << std::endl;
		m_dAverageReconstructTime += frame_reconstruct_time;
		m_dMaxReconstructTime = frame_reconstruct_time > m_dMaxReconstructTime ? frame_reconstruct_time : m_dMaxReconstructTime;
		++m_iReconstructFrameNum;
	}

	//count
	m_iPCFrameCount++;

	return;

}



/*************************************************
Function: HandleTrajectory
Description: a callback function, store the odometry received by subcriber
*************************************************/	
void SimpleRecon::HandleTrajectory(const nav_msgs::Odometry & oTrajectory)
{
	super::HandleTrajectory(oTrajectory);
}


/************************** ROS VISUALIZATION *************************************/

/*************************************************
Function: PublishMesh
Description: publish pcl mesh to ros visualization_msgs marker, in order to show in rviz
*************************************************/	
template<class T>
void SimpleRecon::PublishMesh(pcl::PointCloud<T>& pointcloud, std::vector<pcl::Vertices>& triangles) {

    //new a visual message
	visualization_msgs::Marker oMeshMsgs;
	
	//define header of message
	oMeshMsgs.header.frame_id = m_sOutMeshTFId;
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

    oMeshMsgs.color.a = 1;
    oMeshMsgs.color.r = 1.0;
    oMeshMsgs.color.g = 1.0;
    oMeshMsgs.color.b = 1.0;

    std_msgs::ColorRGBA red;
	red.a = 1;
	red.r = 1.0;
	red.g = 0.1;
	red.b = 0.1;

    std_msgs::ColorRGBA green;
	green.a = 1;
	green.r = 0.1;
	green.g = 1.0;
	green.b = 0.1;

    auto Lerp = [](const std_msgs::ColorRGBA& color_a, const std_msgs::ColorRGBA& color_b, const float& intensity) {
        std_msgs::ColorRGBA color;
        color.a = color_a.a * (1 - intensity) + color_b.a * intensity;
        color.r = color_a.r * (1 - intensity) + color_b.r * intensity;
        color.g = color_a.g * (1 - intensity) + color_b.g * intensity;
        color.b = color_a.b * (1 - intensity) + color_b.b * intensity;
        return color;
    };

	//repeatable vertices
	pcl::PointCloud<T> vMeshVertices;

	//get the reconstruted mesh
    for(auto triangle : triangles) {
        for(int i = 0; i < 3; ++i) {
            vMeshVertices.push_back(pointcloud.points[triangle.vertices[i]]);
        }
    }

	//convert to publishable message
	for (int k = 0; k < vMeshVertices.points.size(); ++k){

		//temp point
    	geometry_msgs::Point oPTemp;
        oPTemp.x = vMeshVertices.points[k].x;
        oPTemp.y = vMeshVertices.points[k].y;
        oPTemp.z = vMeshVertices.points[k].z;
        oMeshMsgs.points.push_back(oPTemp);

        //color
        if(m_bShowConfidence)
            oMeshMsgs.colors.push_back(Lerp(red, green, vMeshVertices.points[k].data_n[3]));

	}//end k

	m_oMeshPublisher.publish(oMeshMsgs);

}
// template void SimpleRecon::PublishMesh(pcl::PointCloud<pcl::PointXYZI>& pointcloud, std::vector<pcl::Vertices>& triangles);
template void SimpleRecon::PublishMesh(pcl::PointCloud<pcl::PointNormal>& pointcloud, std::vector<pcl::Vertices>& triangles);


/*************************************************
Function: PublishMesh
Description: save pcl mesh to folder single_img_clouds, the file type is .ply
*************************************************/	
template<class T>
void SimpleRecon::SaveMesh(const pcl::PointCloud<T>& pointcloud, const std::vector<pcl::Vertices>& triangles) {

    pcl::PolygonMesh mesh;
    pcl::toPCLPointCloud2(pointcloud, mesh.cloud);
    mesh.polygons = triangles;
    
    std::stringstream ss;
    ss << m_sFileHead << "single_img_clouds/";
    std::string command = "mkdir -p " + ss.str();
    system(command.c_str());
    ss << m_iPCFrameCount << "_processed_mesh.ply";
    pcl::io::savePLYFileBinary(ss.str(), mesh);
}
template void SimpleRecon::SaveMesh(const pcl::PointCloud<pcl::PointXYZI>& pointcloud, const std::vector<pcl::Vertices>& triangles);
template void SimpleRecon::SaveMesh(const pcl::PointCloud<pcl::PointNormal>& pointcloud, const std::vector<pcl::Vertices>& triangles);


//************************* util static functions ***********************************

/*************************************************
Function: ReOrganizePoints
    @param in_cloud: raw pcl point cloud
    @param out_cloud: vector of pcl point clouds divided by scan line
    @brief reorder the points by row and col of the points, output a vector of pcl pointclouds which represent a row of scan line
*************************************************/	
void SimpleRecon::ReOrganizePoints(const pcl::PointCloud<pcl::PointXYZI>& in_cloud, std::vector<pcl::PointCloud<pcl::PointXYZI>>& out_cloud) {

    for(int i = 0; i < in_cloud.size(); ++i) {
        
        //which row line of the scan 0 - 15，intensity的整数部分代表第几行
        int scanID = std::abs(in_cloud.points[i].intensity);

        //col line token of the scan 0.0 - 0.1，intensity小数部分代表行中的第几个点
        float relTime = in_cloud.points[i].intensity - scanID;
    
        //store points in the same row to the same vector
        if(scanID >= 0 && scanID < out_cloud.size()) {

            out_cloud[scanID].push_back(in_cloud.points[i]);
        
        }
    }
    
    //大部分的点还是有序的，只有部分帧存在无序的点，排序操作保证帧的intensity为递增顺序
    //reorder points by intensity (which means reorder by col line token)
    for(int i = 0; i < out_cloud.size(); ++i) {
        
        auto& point_line = out_cloud[i].points;

        sort(point_line.begin(), point_line.end(), [](pcl::PointXYZI& a, pcl::PointXYZI& b){
            return a.intensity < b.intensity;
        });
        // std::cout << point_line.size() << " " << point_line.begin()->intensity << "," << point_line.rbegin()->intensity << std::endl;
    }
}

/*************************************************
Function: GetCloudPointDelta
Description: output the count of delta of near points, in order to help the program to handle point cols
*************************************************/	
float SimpleRecon::GetCloudPointDelta(const std::vector<pcl::PointCloud<pcl::PointXYZI>>& in_cloud, DELTA_TYPE::DELTA_TYPE delta_type) {

    std::map<float, int> deltaCount;
    std::map<int, float, std::greater<int>> deltaCountSort;

    for(int i = 0; i < in_cloud.size(); ++i) {

        auto& point_line = in_cloud[i].points;

        //count deltas used for test
        for(int j = 0; j < point_line.size() - 1; ++j) {
            float delta = point_line[j + 1].intensity - point_line[j].intensity;
            ++deltaCount[delta];
        }
    }

    //max count
    for(auto& [delta,count] : deltaCount) {
        if(!deltaCountSort.count(count)) deltaCountSort[count] = delta;
        // else if(deltaCountSort.find(count) == deltaCountSort.begin()) 
        //     std::cout << std::format_red << "\tMax count " << count << " repeat!" << std::format_white;
    }
    std::cout << "\tcount: " << deltaCountSort.begin()->first << "," << deltaCountSort.begin()->second;


    switch (delta_type)
    {
        case DELTA_TYPE::AVERAGE: 
        {
            //average count
            float average_delta = std::accumulate(deltaCountSort.begin(), deltaCountSort.end(), .0f, [](float offset, std::pair<int, float> a) {
                return offset + a.second * a.first;
            }) / std::accumulate(deltaCountSort.begin(), deltaCountSort.end(), 0, [](int offset, std::pair<int, float> a) {
                return offset + a.first;
            });
            std::cout << "average: " << average_delta << std::endl;
            return average_delta;
        }
            

        case DELTA_TYPE::MODE: 
        {
            return deltaCountSort.begin()->second;
        }

        default : return 0;
    }
}

/*************************************************
Function: FromCloudListToRGB
    @param in_cloud: vector of pcl point clouds divided by scan line
    @param out_rgb: vector of unsigned char recording the x y z of points in order
    @param lidar_type: type of lidar, 16, 32, 64 lines are optional
    @brief transform the in_cloud to img data form
*************************************************/	
void SimpleRecon::FromCloudListToRGB(const std::vector<pcl::PointCloud<pcl::PointXYZI>>& in_cloud, std::vector<unsigned char>& out_rgb, int lidar_type) {

    int max_x = in_cloud[0].points[0].x, min_x = in_cloud[0].points[0].x;
    int max_y = in_cloud[0].points[0].y, min_y = in_cloud[0].points[0].y;
    int max_z = in_cloud[0].points[0].z, min_z = in_cloud[0].points[0].z;

    float mode_delta = SimpleRecon::GetCloudPointDelta(in_cloud, DELTA_TYPE::MODE);
    float half_mode_delta = 0.5 * mode_delta;

    int image_width = 0.1 / mode_delta;
    int max_test = 1;
    while(image_width > max_test) max_test <<= 1;
    image_width = max_test;
    int image_height = lidar_type;
    std::cout << "image_size: [" << image_width << "," << image_height << "]" << std::endl;

    std::vector<std::vector<pcl::PointXYZI>> vCloudPicture(image_height, std::vector<pcl::PointXYZI>(image_width, 0));
    for(int i = 0; i < in_cloud.size(); ++i) {

        if(in_cloud[i].points[0].intensity - i < half_mode_delta)
            vCloudPicture[i][0] =  in_cloud[i].points[0];
        else vCloudPicture[i][0].intensity = i;

        int sum_stamp = 0;
        for(int j = 1; j < in_cloud[i].size(); ++j) {

            auto& now_point = in_cloud[i].points[j];

            if(now_point.x > max_x) max_x = now_point.x;
            if(now_point.y > max_y) max_y = now_point.y;
            if(now_point.z > max_z) max_z = now_point.z;
            
            if(now_point.x < min_x) min_x = now_point.x;
            if(now_point.y < min_y) min_y = now_point.y;
            if(now_point.z < min_z) min_z = now_point.z;
            
            float delta = now_point.intensity - in_cloud[i].points[j-1].intensity;
            int jump_stamp = delta / mode_delta + 0.5;
            assert(jump_stamp > 0);
            sum_stamp += jump_stamp - 1;
            vCloudPicture[i][j + sum_stamp] = now_point;
        }
    }

    auto GetColor = [](float val, float min, float max) {
        return (val - min) / (max - min) * 255;
    };

    out_rgb.clear();
    out_rgb.resize(image_width * image_height * 3, 0);
    for(int i = 0; i < vCloudPicture.size(); ++i) {
        for(int j = 0; j < vCloudPicture[i].size(); ++j) {
        
            auto& now_point = vCloudPicture[i][j];

            out_rgb[i * image_width * 3 + j * 3 + 0] = GetColor(now_point.x, min_x, max_x);
            out_rgb[i * image_width * 3 + j * 3 + 1] = GetColor(now_point.y, min_y, max_y);
            out_rgb[i * image_width * 3 + j * 3 + 2] = GetColor(now_point.z, min_z, max_z);
        }
    }
}

/*************************************************
Function: FromCloudListToDepth
    @param in_cloud: vector of pcl point clouds divided by scan line
    @param out_depth: vector of unsigned char recording the depth of points in order relative to the view point
    @param lidar_type: type of lidar, 16, 32, 64 lines are optional
    @param view_point: the lidar center position of the current frame
    @brief transform the in_cloud to depth img data form
*************************************************/	
void SimpleRecon::FromCloudListToDepth(const std::vector<pcl::PointCloud<pcl::PointXYZI>>& in_cloud, std::vector<unsigned char>& out_depth, int lidar_type, pcl::PointXYZI view_point) {

    float mode_delta = SimpleRecon::GetCloudPointDelta(in_cloud, DELTA_TYPE::MODE);
    float half_mode_delta = 0.5 * mode_delta;

    int image_width = 0.1 / mode_delta;
    int max_test = 1;
    while(image_width > max_test) max_test <<= 1;
    image_width = max_test;
    int image_height = lidar_type;
    std::cout << "image_size: [" << image_width << "," << image_height << "]" << std::endl;

    pcl::PointXYZI init_point = view_point;
    init_point.x += 100.0;
    std::vector<std::vector<pcl::PointXYZI>> vCloudPicture(image_height, std::vector<pcl::PointXYZI>(image_width, init_point));
    for(int i = 0; i < in_cloud.size(); ++i) {

        if(in_cloud[i].points[0].intensity - i < half_mode_delta)
            vCloudPicture[i][0] =  in_cloud[i].points[0];
        else vCloudPicture[i][0].intensity = i;

        int sum_stamp = 0;
        for(int j = 1; j < in_cloud[i].size(); ++j) {

            auto& now_point = in_cloud[i].points[j];
            
            float delta = now_point.intensity - in_cloud[i].points[j-1].intensity;
            int jump_stamp = delta / mode_delta + 0.5;
            assert(jump_stamp > 0);
            sum_stamp += jump_stamp - 1;
            vCloudPicture[i][j + sum_stamp] = now_point;
        }
    }

    auto GetDepth = [](const pcl::PointXYZI& p, const pcl::PointXYZI& v) {
        pcl::PointXYZI p_;
        p_.x = p.x - v.x;
        p_.y = p.y - v.y;
        p_.z = p.z - v.z;
        return std::sqrt(p_.x * p_.x + p_.y * p_.y + p_.z * p_.z);
    };
    auto GetColor = [](float val, float min, float max) {
        return (val - min) / (max - min) * 255;
    };

    out_depth.clear();
    out_depth.resize(image_width * image_height, 0);
    for(int i = 0; i < vCloudPicture.size(); ++i) {
        for(int j = 0; j < vCloudPicture[i].size(); ++j) {
        
            auto& now_point = vCloudPicture[i][j];

            out_depth[i * image_width + j + 0] = 255 - GetColor(GetDepth(now_point, view_point), 0.1, 100.0);
        }
    }
}

/*************************************************
Function: FromCloudListToPointVector
    @param in_cloud: vector of pcl point clouds divided by scan line
    @param out_vector: 2d vector of points, you can get the point by calling out_vector[u,v]
    @param lidar_type: type of lidar, 16, 32, 64 lines are optional
    @brief transform the in_cloud to ordered vector form
*************************************************/	
void SimpleRecon::FromCloudListToPointVector(const std::vector<pcl::PointCloud<pcl::PointXYZI>>& in_cloud, std::vector<std::vector<pcl::PointXYZI>>& out_vector, int lidar_type) {
    
    // get mode delta between near points，delta 可以反映相邻扫描点的角度间隔
    float mode_delta = SimpleRecon::GetCloudPointDelta(in_cloud, DELTA_TYPE::MODE);
    float half_mode_delta = 0.5 * mode_delta;

    // calculate 2d vector size
    int image_width = 0.1 / mode_delta;
    int max_test = 1;
    while(image_width > max_test) max_test <<= 1;
    image_width = max_test;             //image_width最终等于大于原来image_width值的最小2幂值
    int image_height = lidar_type;
    // std::cout << "image_size: [" << image_width << "," << image_height << "]" << std::endl;
    out_vector.resize(image_height);
    for(int i = 0; i < out_vector.size(); ++i) 
        out_vector[i].resize(image_width);

    // place the points into 2d vector
    for(int i = 0; i < in_cloud.size(); ++i) {

        // intensity - i is to get the decimal part of the number
        // if(in_cloud[i].points[0].intensity - i < half_mode_delta)
        //     out_vector[i][0] =  in_cloud[i].points[0];
        // else out_vector[i][0].intensity = i;

        for(int j = 0; j < in_cloud[i].size(); ++j) {

            auto& now_point = in_cloud[i].points[j];
            int now_col = (now_point.intensity - i) / mode_delta + 0.5;
            out_vector[i][now_col] = now_point;
        }
    }
}

/*************************************************
Function: FromPointVectorToMesh
    @param in_cloud: 2d vector of points, you can get the point by calling out_vector[u,v]
    @param out_cloud: the point cloud of the reconstructed result mesh. 
    @param out_polygons: the vector of vertices of the reconstructed result mesh.
    @brief reconstruct the in_cloud to mesh using simple Adjacency method.
*************************************************/	
void SimpleRecon::FromPointVectorToMesh(const std::vector<std::vector<pcl::PointXYZI>>& in_cloud, pcl::PointCloud<pcl::PointXYZI>& out_cloud, std::vector<pcl::Vertices>& out_polygons) {
    
    out_cloud.clear();

    //生成输出mesh的顶点，并记录其索引
    std::vector<std::vector<int>> vPointIndex(in_cloud.size());
    for(int i = 0; i < in_cloud.size(); ++i) {
        vPointIndex[i].resize(in_cloud[i].size());
        for(int j = 0; j < in_cloud[i].size(); ++j) {

            if(in_cloud[i][j].x || in_cloud[i][j].y || in_cloud[i][j].z)
            {
                vPointIndex[i][j] = out_cloud.size();
                out_cloud.push_back(in_cloud[i][j]);
            }
            else vPointIndex[i][j] = -1;
        }
    }

    //内部函数，用于检查图像上的点是否存在
    auto CheckUVExists = [&vPointIndex](int u, int v)->bool{
        if(u >= 0 && u < vPointIndex.size() && v >= 0 && v < vPointIndex[u].size())
            return vPointIndex[u][v] != -1;
        else return false;
    };

    out_polygons.clear();

    //按照邻接关系生成网格
    for(int i = 0; i < vPointIndex.size(); ++i) {
        for(int j = 0; j < vPointIndex[i].size(); ++j) {
            
            if(vPointIndex[i][j] == -1) continue;

            if(CheckUVExists(i + 1, j + 1)) {

                if(CheckUVExists(i + 1, j)) {

                    pcl::Vertices triangle;
                    triangle.vertices.push_back(vPointIndex[i][j]);
                    triangle.vertices.push_back(vPointIndex[i + 1][j + 1]);
                    triangle.vertices.push_back(vPointIndex[i + 1][j]);
                    out_polygons.push_back(triangle);
                }

                if(CheckUVExists(i, j + 1)) {

                    pcl::Vertices triangle;
                    triangle.vertices.push_back(vPointIndex[i][j]);
                    triangle.vertices.push_back(vPointIndex[i][j + 1]);
                    triangle.vertices.push_back(vPointIndex[i + 1][j + 1]);
                    out_polygons.push_back(triangle);
                }
            }
            else if(CheckUVExists(i + 1, j) && CheckUVExists(i, j + 1)) {

                pcl::Vertices triangle;
                triangle.vertices.push_back(vPointIndex[i][j]);
                triangle.vertices.push_back(vPointIndex[i][j + 1]);
                triangle.vertices.push_back(vPointIndex[i + 1][j]);
                out_polygons.push_back(triangle);
            }
        }
    }
}


/*************************************************
Function: RecordPseudoFaces
    @param oViewPoint:      the lidar center position of the current frame
    @param vCenterPoint:    the center position of faces of the mesh
    @param vFaces:          faces of the mesh
    @param oMatNormal:      the normal vector of faces of the mesh. 
    @param vTrueFaceStatus: false if the corresponding face is pseudo face.
    @param vFaceWeight:     the confidence value of corresponding face.
    @param fPseudoFaceThr:  threshold of judging pseudo face.
    @brief calculate the face confidence as weight and record the pseudo faces which have low confidence.
*************************************************/	
void SimpleRecon::RecordPseudoFaces(const pcl::PointXYZI & oViewPoint, const pcl::PointCloud<pcl::PointXYZI> & vCenterPoints, const std::vector<pcl::Vertices> & vFaces, const Eigen::MatrixXf & oMatNormal,
	std::vector<bool> & vTrueFaceStatus, std::vector<float> & vFaceWeight, float fPseudoFaceThr){

    vTrueFaceStatus.resize(vFaces.size(), true);
    vFaceWeight.resize(vFaces.size());
    
	//for each face
	for (int i = 0; i != vFaces.size(); ++i){

		//vector from viewpoint to centerpoint of face
		Eigen::Vector3f oToCenterVec(vCenterPoints.points[i].x - oViewPoint.x, vCenterPoints.points[i].y - oViewPoint.y, vCenterPoints.points[i].z - oViewPoint.z);

		//calculate normal weights, e.g., the angle cospin value between normal and ray, where ranges are from - 1 to 1
		vFaceWeight[i] = oToCenterVec.dot(oMatNormal.row(i));
		vFaceWeight[i] = vFaceWeight[i] / oToCenterVec.norm();

		//if it is close to be vertical
		if (fabs(vFaceWeight[i]) < fPseudoFaceThr)
			vTrueFaceStatus[i] = false;

	}//end for i
}

/*************************************************
Function: RemoveMeshFaces
    @param oViewPoint:      the lidar center position of the current frame
    @param in_cloud:        point cloud of input mesh
    @param in_polygons:     faces of the input mesh
    @param out_cloud:       point cloud of output mesh
    @param out_polygons:    faces of the output mesh
    @param fPseudoFaceThr:  threshold of judging pseudo face
    @brief save the faces with high confidence of input mesh to output mesh
*************************************************/	
void SimpleRecon::RemoveMeshFaces(const pcl::PointXYZI& oViewPoint, const pcl::PointCloud<pcl::PointXYZI>& in_cloud, std::vector<pcl::Vertices>& in_polygons,
        pcl::PointCloud<pcl::PointNormal>& out_cloud, std::vector<pcl::Vertices>& out_polygons, float fPseudoFaceThr) {

    //***start the mesh related operation***
    MeshOperation oMeshOper;

    //center point of each faces
    pcl::PointCloud<pcl::PointXYZI>::Ptr pCenterPoints(new pcl::PointCloud<pcl::PointXYZI>);
    oMeshOper.ComputeCenterPoint(in_cloud, in_polygons, *pCenterPoints);

    //face normals
    Eigen::MatrixXf oMatNormal;
    Eigen::VectorXf vfDParam;
    oMeshOper.ComputeAllFaceParams(oViewPoint, in_cloud, in_polygons, oMatNormal, vfDParam);

    //face status - whether the face is wrong
    std::vector<bool> vTrueFaceStatus;
    std::vector<float> vFaceWeight;
    SimpleRecon::RecordPseudoFaces(oViewPoint, *pCenterPoints, in_polygons, oMatNormal, vTrueFaceStatus, vFaceWeight, fPseudoFaceThr);

    //propagate the normal vector to each vertex
    //linearly compute weighted neighboring normal vector
    oMeshOper.LocalFaceNormalAndConfidence(in_cloud, in_polygons, oMatNormal, oViewPoint, out_cloud);

    //Record the remaining triangles
    out_polygons.clear();
    for (int j = 0; j != vTrueFaceStatus.size(); ++j){

        if (true || vTrueFaceStatus[j]){
            pcl::Vertices oOneFace(in_polygons[j]);
            out_polygons.push_back(oOneFace);
        }//end if

    }//end for j != vTrueFaceStatus.size()
}

/*************************************************
Function: RemoveMeshFaces
    @param oViewPoint:      the lidar center position of the current frame
    @param in_cloud:        point cloud of input mesh
    @param in_polygons:     faces of the input mesh
    @param out_cloud:       point cloud of output mesh
    @param out_polygons:    faces of the output mesh
    @param fPseudoFaceThr:  threshold of judging pseudo face
    @brief save the faces with high confidence of input mesh to output mesh
*************************************************/	
void SimpleRecon::RemoveMeshFaces(const pcl::PointXYZI& oViewPoint, const pcl::PointCloud<pcl::PointXYZI>& in_cloud, std::vector<pcl::Vertices>& in_polygons,
        pcl::PointCloud<pcl::PointNormal>& out_cloud, std::vector<pcl::Vertices>& out_polygons, 
        pcl::PointCloud<pcl::PointXYZI>& vCenterPoints, Eigen::MatrixXf& oMatNormal, std::vector<float>& vFaceWeight, float fPseudoFaceThr) {

    //***start the mesh related operation***
    MeshOperation oMeshOper;

    //center point of each faces
    oMeshOper.ComputeCenterPoint(in_cloud, in_polygons, vCenterPoints);

    //face normals
    Eigen::VectorXf vfDParam;
    oMeshOper.ComputeAllFaceParams(oViewPoint, in_cloud, in_polygons, oMatNormal, vfDParam);

    //face status - whether the face is wrong
    //这里计算了面片的置信度
    std::vector<bool> vTrueFaceStatus;
    SimpleRecon::RecordPseudoFaces(oViewPoint, vCenterPoints, in_polygons, oMatNormal, vTrueFaceStatus, vFaceWeight, fPseudoFaceThr);

    //propagate the normal vector to each vertex
    //linearly compute weighted neighboring normal vector
    //这里不仅按照面片平均了每个点的法向量，而且还计算了每个点关于法线的置信度值（记录在 data_n[3] 的位置上）
    oMeshOper.LocalFaceNormalAndConfidence(in_cloud, in_polygons, oMatNormal, oViewPoint, out_cloud);

    //Record the remaining triangles
    out_polygons.clear();
    for (int j = 0; j != vTrueFaceStatus.size(); ++j){

        if (true || vTrueFaceStatus[j]){
            pcl::Vertices oOneFace(in_polygons[j]);
            out_polygons.push_back(oOneFace);
        }//end if

    }//end for j != vTrueFaceStatus.size()
}

/**TODO: 如果是补全点云遮挡了 用原始点云重建出的mesh, 则说明这部分点云是伪面
 *       如果是用原始点云重建出的mesh 遮挡了补全点云, 则说明这部分点云是伪面
 * 
 *       之前设想的连续法向量相同的面也有判断不了的情况,比如:
 *          | \
 *          |  \
 *          |   \
 *          |____\_
 *       这也是补充点云后造成坑的原因, 即使是基于视点朝向的权重也不能排除这种情况,反而放大了错误
 * 
 *       除了凹形状,还有凸形状也会受到影响,比如:
 * 
 *          |_\____
 *             \  |
 *              \ |
 *               \|______
 */

//  ### 添加多参数Confidence作为置信度参考 ###

/*************************************************
Function: RemoveMeshFaces
    @param oViewPoint:      the lidar center position of the current frame
    @param in_cloud:        point cloud of input mesh
    @param in_polygons:     faces of the input mesh
    @param out_cloud:       point cloud of output mesh
    @param out_polygons:    faces of the output mesh
    @param vCenterPoints    output points
    @param oMatNormal       output normals
    @param vFaceWeight      output face confidences
    @param fPseudoFaceThr:  threshold of judging pseudo face
    @brief save the faces with high confidence of input mesh to output mesh
*************************************************/	
void SimpleRecon::RemoveMeshFaces(const pcl::PointXYZI& oViewPoint, const pcl::PointCloud<pcl::PointXYZI>& in_cloud, std::vector<pcl::Vertices>& in_polygons,
    pcl::PointCloud<pcl::PointNormal>& out_cloud, std::vector<pcl::Vertices>& out_polygons, 
    pcl::PointCloud<pcl::PointXYZI>& vCenterPoints, Eigen::MatrixXf& oMatNormal, std::vector<Confidence>& vFaceWeight, float fPseudoFaceThr) {

    //***start the mesh related operation***
    MeshOperation oMeshOper;

    //center point of each faces
    oMeshOper.ComputeCenterPoint(in_cloud, in_polygons, vCenterPoints);

    //face normals
    Eigen::VectorXf vfDParam;
    oMeshOper.ComputeAllFaceParams(oViewPoint, in_cloud, in_polygons, oMatNormal, vfDParam);

    //face status - whether the face is wrong
    //这里计算了面片的置信度
    std::vector<bool> vTrueFaceStatus;
    SimpleRecon::RecordPseudoFaces(oViewPoint, vCenterPoints, in_polygons, oMatNormal, vTrueFaceStatus, vFaceWeight, fPseudoFaceThr);

    //propagate the normal vector to each vertex
    //linearly compute weighted neighboring normal vector
    //这里不仅按照面片平均了每个点的法向量，而且还计算了每个点关于法线的置信度值（记录在 data_n[3] 的位置上）
    oMeshOper.LocalFaceNormalAndDistanceConfidence(in_cloud, in_polygons, oMatNormal, oViewPoint, vTrueFaceStatus, out_cloud);

    //Record the remaining triangles
    out_polygons.clear();
    for (int j = 0; j != vTrueFaceStatus.size(); ++j){

        if (true || vTrueFaceStatus[j]){
            pcl::Vertices oOneFace(in_polygons[j]);
            out_polygons.push_back(oOneFace);
        }//end if

    }//end for j != vTrueFaceStatus.size()
}

/*************************************************
Function: RecordPseudoFaces
    @param oViewPoint:      the lidar center position of the current frame
    @param vCenterPoint:    the center position of faces of the mesh
    @param vFaces:          faces of the mesh
    @param oMatNormal:      the normal vector of faces of the mesh. 
    @param vTrueFaceStatus: false if the corresponding face is pseudo face.
    @param vFaceWeight:     the confidence value of corresponding face.
    @param fPseudoFaceThr:  threshold of judging pseudo face.
    @brief calculate the face confidence as weight and record the pseudo faces which have low confidence.
*************************************************/	
void SimpleRecon::RecordPseudoFaces(const pcl::PointXYZI & oViewPoint, const pcl::PointCloud<pcl::PointXYZI> & vCenterPoints, const std::vector<pcl::Vertices> & vFaces, const Eigen::MatrixXf & oMatNormal, 
    std::vector<bool> & vTrueFaceStatus, std::vector<Confidence> & vFaceWeight, float fPseudoFaceThr) {

    vTrueFaceStatus.resize(vFaces.size(), true);
    vFaceWeight.resize(vFaces.size());
    
    //开始置信度记录器
    Confidence::StartCounter();

	//for each face
	for (int i = 0; i != vFaces.size(); ++i){

		vFaceWeight[i].ComputeConfidence(oViewPoint, vCenterPoints.points[i], oMatNormal.row(i));
        
		//if it is close to be vertical
		if (vFaceWeight[i].normal_confidence < fPseudoFaceThr)
			vTrueFaceStatus[i] = false;

	}//end for i

    std::cout << ";\tface_conf: " << Confidence::min_depth_confidence << ", " << Confidence::max_depth_confidence;
}