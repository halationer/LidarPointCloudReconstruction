#ifndef HPDPOINTCLOUDDISPLAY_H //defeat reconstruction of h.file
#define HPDPOINTCLOUDDISPLAY_H 
#include"LasOperator.h"
#include <string>
#include <vector>
#include <ctime>
#include <iostream>
#include <sstream>//for istringstream function

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>



//颜色类R,G,B
struct ColorRGB{
	float r;
	float g;
	float b;
	std::string name; 
};
/*====================================
一个HPD显示类借助PCL和于永涛代码 
Edited by huang Peng-di 2014.05.07
主要功能：点云的属性显示
内含：
1.整体显示；2.高程值显示；3.单点邻域显示；
4.分类显示；5.特征值显示；
输入通常为点云las或pcd文件的存储容器
相应函数的参数可以根据实际情况对应设置
具体参看每条函数的说明（在函数定义上方）
注意：该类功能函数返回的是一个窗口指针，所以
需要实现定义一个指针，如下示例：
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
HpdDisplay displayer;
viewer=displayer.Showclassification(point3d,"random");
玩的愉快,具体示例在最下方，直接拷贝可用
灵活使用，这几个函数几乎可以应对大部分点云显示要求
（通过拷贝目标数据，转换格式，在函数外转换特征成类别等）
====================================*/
class HpdDisplay{

public:
	//简化
typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> PointCloudViewerPtr;
    //
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudXYZPtr;
	//******************初始化函数**************************
HpdDisplay();
~HpdDisplay();
//生成颜色
bool Generatecolor();
//设置颜色变化区间
void HpdDisplay::Setcolorvariationrange(float hr=255.0,float hg=0.0,float hb=0.0,
	float tr=0.0,float tg=255.0,float tb=0.0);
//设置背景颜色
void Setbackgroudcolor(float,float,float);
    //*******************功能函数***************************
//显示点云（单色显示）
//显示点云的函数所有点显示同一颜色，颜色可调
PointCloudViewerPtr Showsimplecolor(PointCloudXYZPtr &,char *colorvalue="white");

//显示高程信息（Elevation）
//显示点云的高程信息，可抽样，就是以较少点数代替显示
PointCloudViewerPtr Showelevation(PointCloudXYZPtr & ,int sampleInterval=1);

//显示邻域点云
//邻域显示，显示某点的邻域信息，可以是随机点或指定点
PointCloudViewerPtr Showneighboorcloud(PointCloudXYZPtr &, int type=1, float radius=0.3, int viewpoint=-1);
PointCloudViewerPtr Showneighboorcloud(PointCloudXYZPtr &, double f_x, double f_y, double f_z, float radius=0.3,int type=1 );

//显示分类信息
//显示点云的类别，即观测点云的分类结果
PointCloudViewerPtr Showclassification(std::vector<Point3D> &, char *keyword="random");
PointCloudViewerPtr Showclassification(PointCloudXYZPtr &, std::vector<int> &, char *keyword = "random");
PointCloudViewerPtr Showclassification(pcl::PointCloud<pcl::PointXYZ> & vCloud, std::vector<int> & vClasses, char *keyword = "random");

//显示点云特征
//点云每点的但特征值结果变化分布显示，显示每一个点的一个特征值，需要每个点的特征值容器
//"gray","redgreen" ,"greenblue","redblue","free"五种模式
PointCloudViewerPtr Showfeatureresult(std::vector<Point3D> &,std::vector<double> &, char *keyword="redgreen");
PointCloudViewerPtr Showfeatureresult(PointCloudXYZPtr & , std::vector<float> & , char *keyword = "redgreen");
PointCloudViewerPtr Showfeatureresult(pcl::PointCloud<pcl::PointXYZ> &, std::vector<float> &, char *keyword = "redgreen");

//显示一些几何物体
PointCloudViewerPtr Creatgeometry(PointCloudXYZPtr &,Point3D &);

//show center points with lines
//L indicates lines with blue color 
//S indicates sphere (points) with red color
void AddLineWithPoints(boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer,
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & vSkelinePoints,
	float fRadius = 0.01, float fLr = 0.0, float fLg = 0.0, float fLb = 1.0,
	float fSr = 1.0, float fSg = 0.0, float fSb = 0.0);

void AddSphereAtPoints(boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer,
	pcl::PointCloud<pcl::PointXYZ>::Ptr & vSkelinePoints,
	float fRadius = 0.01, float fSr = 1.0, float fSg = 0.0, float fSb = 0.0);


//一个随机数小函数，不用在意
double random(double,double);
void displaytable();

private:
//方便颜色调用的存储器
	std::vector<ColorRGB> colors;
	//colors生成旗
	bool colorflag;
	bool colorvarflag;
	//背景颜色
	ColorRGB backgroudcolor;
	ColorRGB headcolor;
	ColorRGB tailcolor;
};
//颜色变化定位函数
float Colorvar(float headnum,float tailnum,float proportion);
/*==================================Example,you can copy this down on your main cpp to run
One Example:
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
std::vector<Point3D> point3d;
HPDpointclouddataread("CornerXYZPCD.pcd",cloud, point3d,2);
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
HpdDisplay hpdisplay;
viewer=hpdisplay.Showsimplecolor(cloud,"yellow");
while (!viewer->wasStopped())
{
     viewer->spinOnce ();
}
=====================================*/

#endif
