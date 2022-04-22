#include "HpdPointCloudDisplay.h"
//------------------------------------------------------------------------------------------
typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> PointCloudViewerPtr;

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudXYZPtr;
/*================================
HpdDisplay构造函数简介
功能：初始化某些参数
backgroudcolor初始化背景颜色
colorflag旗子
Generatecolor()用于初始化调色板
=================================*/
HpdDisplay::HpdDisplay(){
	colorflag=false;
	colorflag=Generatecolor();
	//默认背景白色
	backgroudcolor.r=colors[0].r;
	backgroudcolor.g=colors[0].g;
	backgroudcolor.b=colors[0].b;
	Setcolorvariationrange();
	colorvarflag=false;
}
/*================================
~HpdDisplay()析构函数简介
功能：不解释
=================================*/
HpdDisplay::~HpdDisplay(){

}
/*================================
Setcolorvariationrange析构函数简介
功能：不解释
=================================*/
void HpdDisplay::Setcolorvariationrange(float hr,float hg,float hb,
	float tr,float tg,float tb){
	headcolor.r=hr;
	headcolor.g=hg;
	headcolor.b=hb;
	tailcolor.r=tr;
	tailcolor.g=tg;
	tailcolor.b=tb;
	colorvarflag=true;
}
/*================================
Generatecolor()函数简介
功能：初始化调色板并赋值
总共27总重要颜色方便随时调用
用到颜色的地方直接调用这些容器内的值
=================================*/
bool HpdDisplay::Generatecolor(){
	//可爱的调色板君
colors.resize(27);
//灰度系
colors[0].r=255;colors[0].g=255;colors[0].b=255;colors[0].name="纯白";
colors[1].r=0;colors[1].g=0;colors[1].b=0;colors[1].name="纯黑";
colors[2].r=185;colors[2].g=185;colors[2].b=185;colors[2].name="灰色";
//绿色系
colors[3].r=0;colors[3].g=255;colors[3].b=0;colors[3].name="橙绿";
colors[4].r=46;colors[4].g=139;colors[4].b=87;colors[4].name="海洋绿";
colors[5].r=0;colors[5].g=128;colors[5].b=0;colors[5].name="纯绿";
colors[6].r=124;colors[6].g=252;colors[6].b=0;colors[6].name="草坪绿";
//青色系
colors[7].r=0;colors[7].g=255;colors[7].b=255;colors[7].name="青色";
colors[8].r=32;colors[8].g=178;colors[8].b=170;colors[8].name="浅海洋清";
colors[9].r=0;colors[9].g=139;colors[9].b=139;colors[9].name="深青色";
//黄色系
colors[10].r=255;colors[10].g=255;colors[10].b=0;colors[10].name="纯黄";
colors[11].r=189;colors[11].g=183;colors[11].b=107;colors[11].name="卡布奇诺";
colors[12].r=255;colors[12].g=215;colors[12].b=0;colors[12].name="金色";
colors[13].r=255;colors[13].g=165;colors[13].b=0;colors[13].name="橙色";
colors[14].r=255;colors[14].g=222;colors[14].b=173;colors[14].name="皮肤黄";
colors[15].r=255;colors[15].g=140;colors[15].b=0;colors[15].name="深橙色";
colors[16].r=210;colors[16].g=105;colors[16].b=30;colors[16].name="巧克力色";
//红色系
colors[17].r=255;colors[17].g=69;colors[17].b=0;colors[17].name="橙红色";
colors[18].r=255;colors[18].g=0;colors[18].b=0;colors[18].name="纯红";
colors[19].r=139;colors[19].g=9;colors[19].b=0;colors[19].name="深红色";
//紫色系
colors[20].r=128;colors[20].g=0;colors[20].b=128;colors[20].name="紫色";
colors[21].r=148;colors[21].g=0;colors[21].b=211;colors[21].name="深紫色";
colors[22].r=255;colors[22].g=105;colors[22].b=180;colors[22].name="粉红色";
//蓝色系
colors[23].r=0;colors[23].g=0;colors[23].b=255;colors[23].name="蓝色";
colors[24].r=0;colors[24].g=0;colors[24].b=139;colors[24].name="深蓝色";
colors[25].r=30;colors[25].g=144;colors[25].b=255;colors[25].name="道奇蓝";
colors[26].r=0;colors[26].g=191;colors[26].b=255;colors[26].name="天蓝色";

return true;
}
/*================================
Setbackgroudcolor函数简介
功能：设置背景颜色
形参1,2,3分别是r,g,b颜色值
主要是为了方便调整背景颜色
大于1按r,g,b的255方式计数
=================================*/
void HpdDisplay::Setbackgroudcolor(float f_r,float f_g,float f_b){
	//防止乱来
	if(f_r*f_g*f_b<0){
	backgroudcolor.r=0;
	backgroudcolor.g=0;
	backgroudcolor.b=0;
	}
	//大于1的输入可能是255格式的rgb
	if(f_r>1)
	backgroudcolor.r=f_r/255;
	if(f_g>1)
	backgroudcolor.g=f_g/255;
	if(f_b>1)
	backgroudcolor.b=f_b/255;
}
/*================================
Showsimplecolor函数简介
功能：显示点云
输入形参1：一个指向pointXYZ类型的指针
输入形参2：一个char型数组，只允许yellow,white,black和red
输出：简单的点云显示窗口及点云
=================================*/
PointCloudViewerPtr HpdDisplay::Showsimplecolor(PointCloudXYZPtr & pointxyz,char *colorchoose){
	//先判断调色板是否已经生成
	while(!colorflag){
		colorflag=Generatecolor();
	}
	//索引
	int i;
	//显示对象
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("单色点云显示窗口"));
	//一个字符串判断给第二个形参,默认白色
	if (!strcmp(colorchoose,"white"))
	i=0;
	else if(!strcmp(colorchoose,"red"))
	i=18;
	else if(!strcmp(colorchoose,"yellow"))
	i=10;
	else if(!strcmp(colorchoose,"black"))
	i=1;
	else if(!strcmp(colorchoose,"grey"))
	i=2;
	else if(!strcmp(colorchoose,"free")){
	std::cout<<"请问大人，您要上什么颜色? 请输入序号"<<std::endl;
	displaytable();
	std::cin>>i;
	}
	//颜色赋值
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color (pointxyz, colors[i].r, colors[i].g, colors[i].b);
	viewer->addPointCloud(pointxyz,color,"Simple Cloud");
	//背景
	viewer->setBackgroundColor(backgroudcolor.r, backgroudcolor.g, backgroudcolor.b);
	return(viewer);
}//end Showsimplecolor

/*================================
Showelevation函数简介
功能：显示高程度，就是高度信息的可视化（彩色化）
输入形参1：一个指向pointXYZ类型的指针
输入形参2：一个整型用于隔sampleInterval个点采样，1为全采样你懂的
输出：随高度（点的z轴值）变化的彩色点云，高度若不是z轴就伤不起了
=================================*/
PointCloudViewerPtr HpdDisplay::Showelevation(PointCloudXYZPtr & pointxyz,int sampleInterval){

	//一个彩色点云用于保存颜色信息
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb(new pcl::PointCloud<pcl::PointXYZRGB>);

	//PCL XYZ格式
	unsigned long pointNum = pointxyz->size()/sampleInterval;
    cloudrgb->width = pointNum;
	cloudrgb->height = 1;
	cloudrgb->is_dense = false;
	cloudrgb->points.resize(cloudrgb->width*cloudrgb->height);
    //计算z轴最大最小值，方便归一化后重新赋值
	double minElevation = pointxyz->points[0].z, maxElevation = pointxyz->points[0].z;
	for(size_t i=1;i<pointNum;++i){
		if(pointxyz->points[i].z<minElevation)
			minElevation = pointxyz->points[i].z;
		if( pointxyz->points[i].z>maxElevation)
			maxElevation = pointxyz->points[i].z;
	}
	//以等间隔采样的方式选择点云
	for(size_t i=0,j=0;i<cloudrgb->points.size();++i,j+=sampleInterval){
		cloudrgb->points[i].x = pointxyz->points[j].x;
		cloudrgb->points[i].y = pointxyz->points[j].y;
		cloudrgb->points[i].z = pointxyz->points[j].z;
		//同时根据归一化公式等比例绘制图像的高程值
		//华丽的配色方案，谁叫我以前是颜色实验室的呢,我会跟你讲亚当斯密图中蓝色最难分辨吗
		cloudrgb->points[i].r = Colorvar(headcolor.r,tailcolor.r,(pointxyz->points[j].z-minElevation)/(maxElevation-minElevation));
	    cloudrgb->points[i].g = Colorvar(headcolor.g,tailcolor.g,(pointxyz->points[j].z-minElevation)/(maxElevation-minElevation));
	    cloudrgb->points[i].b = Colorvar(headcolor.b,tailcolor.b,(pointxyz->points[j].z-minElevation)/(maxElevation-minElevation));
		
	}
	//建立一个PCL的显示对象
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("高程点云显示窗口"));
	
	//PCL赋色
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloudrgb);
	viewer->addPointCloud(cloudrgb,rgb,"Elevation Cloud");	
	//背景
	viewer->setBackgroundColor(backgroudcolor.r, backgroudcolor.g, backgroudcolor.b);
	//返回窗口
    return(viewer);
	
}//end Showelevation
/*================================
Showneighboorcloud函数简介
功能：某个点邻域信息的可视化（彩色化）
输入形参1：一个指向pointXYZ类型的指针
输入形参2：一个整型用于定义类型，数值1为高亮邻域，其他数值为单独显示邻域
输入形参3：float型的半径，这个很重要
输入形参4：视点，按照序号来选择，比如有10000个点，你选择第3021个，
因此不能任选一点，必须事先知道其序号，不选的话就随机一个
输出：可视化的点云邻域结果
=================================*/
//显示邻域的函数
PointCloudViewerPtr HpdDisplay::Showneighboorcloud(PointCloudXYZPtr & pointxyz, int type, float radius, int viewpoint){
	
	//一个彩色点云用于保存颜色信息
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointrgb(new pcl::PointCloud<pcl::PointXYZRGB>);

	//	
	if (viewpoint==-1){
	    srand(unsigned(time(0)));//用系统时间作为随机种子
	    for(int icnt = 0; icnt != 2; ++icnt)//随机数运行两次随机较分散
			viewpoint=int(random(0,pointxyz->size()-1)); 
	}
	else if (viewpoint>=pointxyz->size()){
		std::cout<<"Out of the points number range!!!!"<<std::endl;
	    exit(0);
	}

	//生成kdtree并索引
	pcl::KdTreeFLANN<pcl::PointXYZ>kdtree;
    kdtree.setInputCloud (pointxyz);
	//设置搜索中心
    pcl::PointXYZ searchPoint;
    searchPoint.x=pointxyz->points[viewpoint].x;
    searchPoint.y=pointxyz->points[viewpoint].y;
    searchPoint.z=pointxyz->points[viewpoint].z;
    //+++++++设置搜索半径++++++important
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
//执行搜索
if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) >0 )
  {
	  //+++++++++++++++++++++++++++++++++++++++++++两种模式++++++++++++++++++++++
	  //+++++++++++++++++++++++++++++模式1全部显示，高亮邻域（默认）
	  if(type==1){
		  //先统一给点云附上白色
	unsigned long pointNum = pointxyz->size();
    pointrgb->width = pointNum;
	pointrgb->height = 1;
	pointrgb->is_dense = false;
	pointrgb->points.resize(pointrgb->width*pointrgb->height);
	for(size_t i=0;i<pointxyz->points.size();++i){
		pointrgb->points[i].x = pointxyz->points[i].x;
		pointrgb->points[i].y = pointxyz->points[i].y;
		pointrgb->points[i].z = pointxyz->points[i].z;
		pointrgb->points[i].r = 150;
		pointrgb->points[i].g = 150;
		pointrgb->points[i].b = 150;
	}
	//给邻域附上对应的颜色
for (size_t i=0; i<pointIdxRadiusSearch.size (); ++i){
pointrgb->points[ pointIdxRadiusSearch[i] ].r=255;  
pointrgb->points[ pointIdxRadiusSearch[i] ].g=255;
pointrgb->points[ pointIdxRadiusSearch[i] ].b=0;
}//end for
	  }//end second if
	  else{
		  //++++++++++++++++++++++++++++++++++++++++其他情况都用第二种模式
		  //+++++++++++++++++++++++++只显示邻域的点
		  //先统一给点云附上白色
	unsigned long pointNum =(unsigned long)pointIdxRadiusSearch.size();
    pointrgb->width = pointNum;
	pointrgb->height = 1;
	pointrgb->is_dense = false;
	pointrgb->points.resize(pointrgb->width*pointrgb->height);
	for(size_t i=0;i!=pointIdxRadiusSearch.size();++i){
		//只显示邻域点的值
		pointrgb->points[i].x= pointxyz->points[ pointIdxRadiusSearch[i] ].x ;
		pointrgb->points[i].y= pointxyz->points[ pointIdxRadiusSearch[i] ].y;
		pointrgb->points[i].z= pointxyz->points[ pointIdxRadiusSearch[i] ].z;
        pointrgb->points[i].r=255;  
        pointrgb->points[i].g=255;
        pointrgb->points[i].b=0;
              }//end for
	  }//end else
}//end first if
//
//建立一个PCL的显示对象
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("邻域点云可视化"));
	//
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointrgb);
	viewer->addPointCloud(pointrgb,rgb,"Neighboor Cloud Show");	
	//背景
	viewer->setBackgroundColor(backgroudcolor.r, backgroudcolor.g, backgroudcolor.b);
	//hold住窗口
	return(viewer);
}
PointCloudViewerPtr HpdDisplay::Showneighboorcloud(PointCloudXYZPtr & pointxyz, 
	double f_x, double f_y, double f_z, float radius,int type){
	//一个彩色点云用于保存颜色信息
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointrgb(new pcl::PointCloud<pcl::PointXYZRGB>);

	//生成kdtree并索引
	pcl::KdTreeFLANN<pcl::PointXYZ>kdtree;
    kdtree.setInputCloud (pointxyz);
	//设置搜索中心
    pcl::PointXYZ searchPoint;
    searchPoint.x=float(f_x);
    searchPoint.y=float(f_y);
    searchPoint.z=float(f_z);
    //+++++++设置搜索半径++++++important
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
//执行搜索
if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) >0 )
  {
	  //+++++++++++++++++++++++++++++++++++++++++++两种模式++++++++++++++++++++++
	  //+++++++++++++++++++++++++++++模式1全部显示，高亮邻域（默认）
	  if(type==1){
		  //先统一给点云附上白色
	unsigned long pointNum = pointxyz->size();
    pointrgb->width = pointNum;
	pointrgb->height = 1;
	pointrgb->is_dense = false;
	pointrgb->points.resize(pointrgb->width*pointrgb->height);
	for(size_t i=0;i<pointxyz->points.size();++i){
		pointrgb->points[i].x = pointxyz->points[i].x;
		pointrgb->points[i].y = pointxyz->points[i].y;
		pointrgb->points[i].z = pointxyz->points[i].z;
		pointrgb->points[i].r = 150;
		pointrgb->points[i].g = 150;
		pointrgb->points[i].b = 150;
	}
	//给邻域附上对应的颜色
for (size_t i=0; i<pointIdxRadiusSearch.size (); ++i){
pointrgb->points[ pointIdxRadiusSearch[i] ].r=255;  
pointrgb->points[ pointIdxRadiusSearch[i] ].g=255;
pointrgb->points[ pointIdxRadiusSearch[i] ].b=0;
}//end for
	  }//end second if
	  else{
		  //++++++++++++++++++++++++++++++++++++++++其他情况都用第二种模式
		  //+++++++++++++++++++++++++只显示邻域的点
		  //先统一给点云附上白色
	unsigned long pointNum =(unsigned long)pointIdxRadiusSearch.size();
    pointrgb->width = pointNum;
	pointrgb->height = 1;
	pointrgb->is_dense = false;
	pointrgb->points.resize(pointrgb->width*pointrgb->height);
	for(size_t i=0;i!=pointIdxRadiusSearch.size();++i){
		//只显示邻域点的值
		pointrgb->points[i].x= pointxyz->points[ pointIdxRadiusSearch[i] ].x ;
		pointrgb->points[i].y= pointxyz->points[ pointIdxRadiusSearch[i] ].y;
		pointrgb->points[i].z= pointxyz->points[ pointIdxRadiusSearch[i] ].z;
        pointrgb->points[i].r=255;  
        pointrgb->points[i].g=255;
        pointrgb->points[i].b=0;
              }//end for
	  }//end else
}//end first if
//
//建立一个PCL的显示对象
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("邻域点云可视化"));
	//
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointrgb);
	viewer->addPointCloud(pointrgb,rgb,"Neighboor Cloud Show");	
	//背景
	viewer->setBackgroundColor(backgroudcolor.r, backgroudcolor.g, backgroudcolor.b);
	//hold住窗口
	return(viewer);
}
/*================================
HpdDisplay::Showclassification函数简介
功能：显示特征提取或分类识别的结果
输入形参1：一个Point3D类型的点云数据（一般是指针开辟的内存区域）
输入形参2：一个字符串用于判断条件，有指定"assign", 随机"random"两种方式
指定就是交互式选择颜色，随机是随机颜色
注意：点云类别必须线性，即1,2,3,4这样的序号，而非1,13,44,89这样跳跃的排序
输出：分类结果图
=================================*/
PointCloudViewerPtr HpdDisplay::Showclassification(std::vector<Point3D> & point3d,char *keyword){
	//先判断调色板是否已经生成
	while(!colorflag){
		colorflag=Generatecolor();
	}
	//计算模式
char *dictionary[2] = {"assign", "random"};//指针数组
int keynumber;
for(keynumber=0;keynumber<2;keynumber++)
{
  if(!strcmp(keyword,dictionary[keynumber]))
	  break;
}
int maxclassification=-INT_MAX,minclassification=INT_MAX;
//找到最大类数
for(size_t i=0;i!=point3d.size();i++){
	if(point3d[i].classification> maxclassification)
		maxclassification=point3d[i].classification;
	if(point3d[i].classification< minclassification)
		minclassification=point3d[i].classification;
}
//最后
int classnumber=maxclassification-minclassification;
//初始化彩色点云
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb(new pcl::PointCloud<pcl::PointXYZRGB>);

unsigned long pointNum = point3d.size();
cloudrgb->width = pointNum;
cloudrgb->height = 1;
cloudrgb->is_dense = false;
cloudrgb->points.resize(cloudrgb->width*cloudrgb->height);

switch(keynumber){
   //指定模式，类别比较少，有利于人眼分析的
case 0:{
//手动制定颜色，先显示一些东西
	std::vector<int> colorindices(classnumber+1); 
	std::cout<<"共"<<classnumber+1<<"类"<<std::endl;
	std::cout<<"请问大人，您要上什么颜色? 请输入序号"<<std::endl;
	displaytable();
   //手动赋值
	for(int i=0;i<classnumber+1;i++){
		std::cout<<"类"<<i+1<<"颜色序号，请输入并回车"<<std::endl;
	    std::cin>> colorindices[i];
	}
   //上色
for(size_t i=0;i<cloudrgb->points.size();++i){
	cloudrgb->points[i].x = point3d[i].x;
	cloudrgb->points[i].y = point3d[i].y;
	cloudrgb->points[i].z = point3d[i].z;
	//通过索引来调用
	cloudrgb->points[i].r = colors[colorindices[point3d[i].classification-minclassification]].r;
	cloudrgb->points[i].g = colors[colorindices[point3d[i].classification-minclassification]].g;
	cloudrgb->points[i].b = colors[colorindices[point3d[i].classification-minclassification]].b;
}
break;}
   //随机模式,用于点云类别比较多的情况
case 1:{
	//用于给类装颜色
	std::vector<ColorRGB> classcolors(classnumber+1);
	//用系统时间作为随机种子
	srand(unsigned(time(0)));
	for(size_t n=0;n!=classnumber+1;n++){
	    for(int icnt = -1; icnt != 3; ++icnt){//利用时间间隔
			if(icnt==0)
			classcolors[n].r=float(random(0,255)); 
		    if(icnt==1)
			classcolors[n].g=float(random(0,255)); 
			if(icnt==2)
			classcolors[n].g=float(random(0,255)); 
	}
}
	int indices=0;
//以等间隔采样的方式选择点云
    for(size_t i=0;i<cloudrgb->points.size();++i){
	cloudrgb->points[i].x = point3d[i].x;
	cloudrgb->points[i].y = point3d[i].y;
	cloudrgb->points[i].z = point3d[i].z;
	//找到该类对应的随机颜色值
	indices=point3d[i].classification-minclassification;
	cloudrgb->points[i].r = classcolors[indices].r;
	cloudrgb->points[i].g = classcolors[indices].g;
	cloudrgb->points[i].b = classcolors[indices].b;
}
	break;}
//默认模式
default:{
	std::cout<<"天堂有路你不走，地狱无门你偏闯！模式选择错误"<<std::endl;
	std::cin.get();
	exit(0);
	break;}
}//end switch

	//建立一个PCL的显示对象
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloudrgb);
	viewer->addPointCloud<pcl::PointXYZRGB> (cloudrgb, rgb, "classified cloud");
	
	viewer->setBackgroundColor (backgroudcolor.r, backgroudcolor.g, backgroudcolor.b);
	//hold住窗口
	return(viewer);

}//end Classification


/*================================
HpdDisplay::Showclassification函数简介
功能：显示特征提取或分类识别的结果
输入形参1：一个Point3D类型的点云数据（一般是指针开辟的内存区域）
输入形参2：一个字符串用于判断条件，有指定"assign", 随机"random"两种方式
指定就是交互式选择颜色，随机是随机颜色
注意：点云类别必须线性，即1,2,3,4这样的序号，而非1,13,44,89这样跳跃的排序
输出：分类结果图
=================================*/
PointCloudViewerPtr HpdDisplay::Showclassification(PointCloudXYZPtr & pCloud, std::vector<int> & vClasses, char *keyword){
	
		//先判断调色板是否已经生成
		while (!colorflag){
			colorflag = Generatecolor();
		}
		//计算模式
		char *dictionary[2] = { "assign", "random" };//指针数组
		int keynumber;
		for (keynumber = 0; keynumber<2; keynumber++)
		{
			if (!strcmp(keyword, dictionary[keynumber]))
				break;
		}
		int maxclassification = -INT_MAX, minclassification = INT_MAX;
		//找到最大类数
		for (size_t i = 0; i != vClasses.size(); i++){
			if (vClasses[i] > maxclassification)
				maxclassification = vClasses[i];
			if (vClasses[i] < minclassification)
				minclassification = vClasses[i];
		}
		//最后
		int classnumber = maxclassification - minclassification;
		//初始化彩色点云
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb(new pcl::PointCloud<pcl::PointXYZRGB>);

		unsigned long pointNum = pCloud->points.size();
		cloudrgb->width = pointNum;
		cloudrgb->height = 1;
		cloudrgb->is_dense = false;
		cloudrgb->points.resize(cloudrgb->width*cloudrgb->height);

		switch (keynumber){
			//指定模式，类别比较少，有利于人眼分析的
		case 0:{
			//手动制定颜色，先显示一些东西
			std::vector<int> colorindices(classnumber + 1);
			std::cout << "共" << classnumber + 1 << "类" << std::endl;
			std::cout << "请问大人，您要上什么颜色? 请输入序号" << std::endl;
			displaytable();
			//手动赋值
			for (int i = 0; i<classnumber + 1; i++){
				std::cout << "类" << i + 1 << "颜色序号，请输入并回车" << std::endl;
				std::cin >> colorindices[i];
			}
			//上色
			for (size_t i = 0; i<cloudrgb->points.size(); ++i){
				cloudrgb->points[i].x = pCloud->points[i].x;
				cloudrgb->points[i].y = pCloud->points[i].y;
				cloudrgb->points[i].z = pCloud->points[i].z;
				//通过索引来调用
				cloudrgb->points[i].r = colors[colorindices[vClasses[i] - minclassification]].r;
				cloudrgb->points[i].g = colors[colorindices[vClasses[i] - minclassification]].g;
				cloudrgb->points[i].b = colors[colorindices[vClasses[i] - minclassification]].b;
			}
			break; }
			//随机模式,用于点云类别比较多的情况
		case 1:{
			//用于给类装颜色
			std::vector<ColorRGB> classcolors(classnumber + 1);
			//用系统时间作为随机种子
			srand(unsigned(time(0)));
			for (size_t n = 0; n != classnumber + 1; n++){
				for (int icnt = -1; icnt != 3; ++icnt){//利用时间间隔
					if (icnt == 0)
						classcolors[n].r = float(random(0, 255));
					if (icnt == 1)
						classcolors[n].g = float(random(0, 255));
					if (icnt == 2)
						classcolors[n].g = float(random(0, 255));
				}
			}
			int indices = 0;
			//以等间隔采样的方式选择点云
			for (size_t i = 0; i<cloudrgb->points.size(); ++i){
				cloudrgb->points[i].x = pCloud->points[i].x;
				cloudrgb->points[i].y = pCloud->points[i].y;
				cloudrgb->points[i].z = pCloud->points[i].z;
				//找到该类对应的随机颜色值
				indices = vClasses[i] - minclassification;
				cloudrgb->points[i].r = classcolors[indices].r;
				cloudrgb->points[i].g = classcolors[indices].g;
				cloudrgb->points[i].b = classcolors[indices].b;
			}
			break; }
			//默认模式
		default:{
			std::cout << "天堂有路你不走，地狱无门你偏闯！模式选择错误" << std::endl;
			std::cin.get();
			exit(0);
			break; }
		}//end switch

		//建立一个PCL的显示对象
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloudrgb);
		viewer->addPointCloud<pcl::PointXYZRGB>(cloudrgb, rgb, "classified cloud");

		viewer->setBackgroundColor(backgroudcolor.r, backgroudcolor.g, backgroudcolor.b);
		//hold住窗口
		return(viewer);

}//end Classification


/*================================
HpdDisplay::Showclassification函数简介
功能：显示特征提取或分类识别的结果
输入形参1：一个Point3D类型的点云数据（一般是指针开辟的内存区域）
输入形参2：一个字符串用于判断条件，有指定"assign", 随机"random"两种方式
指定就是交互式选择颜色，随机是随机颜色
注意：点云类别必须线性，即1,2,3,4这样的序号，而非1,13,44,89这样跳跃的排序
输出：分类结果图
=================================*/
PointCloudViewerPtr HpdDisplay::Showclassification(pcl::PointCloud<pcl::PointXYZ> & vCloud, std::vector<int> & vClasses, char *keyword){

	//先判断调色板是否已经生成
	while (!colorflag){
		colorflag = Generatecolor();
	}
	//计算模式
	char *dictionary[2] = { "assign", "random" };//指针数组
	int keynumber;
	for (keynumber = 0; keynumber<2; keynumber++)
	{
		if (!strcmp(keyword, dictionary[keynumber]))
			break;
	}
	int maxclassification = -INT_MAX, minclassification = INT_MAX;
	//找到最大类数
	for (size_t i = 0; i != vClasses.size(); i++){
		if (vClasses[i] > maxclassification)
			maxclassification = vClasses[i];
		if (vClasses[i] < minclassification)
			minclassification = vClasses[i];
	}
	//最后
	int classnumber = maxclassification - minclassification;
	//初始化彩色点云
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb(new pcl::PointCloud<pcl::PointXYZRGB>);

	unsigned long pointNum = vCloud.points.size();
	cloudrgb->width = pointNum;
	cloudrgb->height = 1;
	cloudrgb->is_dense = false;
	cloudrgb->points.resize(cloudrgb->width*cloudrgb->height);

	switch (keynumber){
		//指定模式，类别比较少，有利于人眼分析的
	case 0:{
		//手动制定颜色，先显示一些东西
		std::vector<int> colorindices(classnumber + 1);
		std::cout << "共" << classnumber + 1 << "类" << std::endl;
		std::cout << "请问大人，您要上什么颜色? 请输入序号" << std::endl;
		displaytable();
		//手动赋值
		for (int i = 0; i<classnumber + 1; i++){
			std::cout << "类" << i + 1 << "颜色序号，请输入并回车" << std::endl;
			std::cin >> colorindices[i];
		}
		//上色
		for (size_t i = 0; i<cloudrgb->points.size(); ++i){
			cloudrgb->points[i].x = vCloud.points[i].x;
			cloudrgb->points[i].y = vCloud.points[i].y;
			cloudrgb->points[i].z = vCloud.points[i].z;
			//通过索引来调用
			cloudrgb->points[i].r = colors[colorindices[vClasses[i] - minclassification]].r;
			cloudrgb->points[i].g = colors[colorindices[vClasses[i] - minclassification]].g;
			cloudrgb->points[i].b = colors[colorindices[vClasses[i] - minclassification]].b;
		}
		break; }
		//随机模式,用于点云类别比较多的情况
	case 1:{
		//用于给类装颜色
		std::vector<ColorRGB> classcolors(classnumber + 1);
		//用系统时间作为随机种子
		srand(unsigned(time(0)));
		for (size_t n = 0; n != classnumber + 1; n++){
			for (int icnt = -1; icnt != 3; ++icnt){//利用时间间隔
				if (icnt == 0)
					classcolors[n].r = float(random(0, 255));
				if (icnt == 1)
					classcolors[n].g = float(random(0, 255));
				if (icnt == 2)
					classcolors[n].g = float(random(0, 255));
			}
		}
		int indices = 0;
		//以等间隔采样的方式选择点云
		for (size_t i = 0; i<cloudrgb->points.size(); ++i){
			cloudrgb->points[i].x = vCloud.points[i].x;
			cloudrgb->points[i].y = vCloud.points[i].y;
			cloudrgb->points[i].z = vCloud.points[i].z;
			//找到该类对应的随机颜色值
			indices = vClasses[i] - minclassification;
			cloudrgb->points[i].r = classcolors[indices].r;
			cloudrgb->points[i].g = classcolors[indices].g;
			cloudrgb->points[i].b = classcolors[indices].b;
		}
		break; }
		//默认模式
	default:{
		std::cout << "天堂有路你不走，地狱无门你偏闯！模式选择错误" << std::endl;
		std::cin.get();
		exit(0);
		break; }
	}//end switch

	//建立一个PCL的显示对象
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloudrgb);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloudrgb, rgb, "classified cloud");

	viewer->setBackgroundColor(backgroudcolor.r, backgroudcolor.g, backgroudcolor.b);
	//hold住窗口
	return(viewer);

}//end Classification

/*================================
Showfeatureresult函数简介
功能：显示点云特征分布
可能选择三总颜色模式，分别是红绿，灰度，红蓝目标显示
通常某一算法计算后在某个点上会有一个以上的特征值，该函数只显示一个特征值
若四个本身也不方便同时显示，可以1个个轮流显示
输入形参1：一个point3d点云引用
输入形参2：一个double型的容器存储特征值
注意：特征是按每点为单位的
输出：特征分布图
"gray","redgreen" ,"greenblue","redblue","free"五种模式
=================================*/
PointCloudViewerPtr HpdDisplay::Showfeatureresult(std::vector<Point3D> & point3d,std::vector<double> & features,char *keyword){
	//先判断是不是同一个点云集的
	if(point3d.size()!=features.size()){
		std::cout<<"Error:点云集和特征集个数不匹配！"<<std::endl;
		std::cout<<"请按回车结束程序，确认数据问题"<<std::endl;
		std::cin.get();
		exit(0);
	}
	//********************判断是用已给的特征值还是用现在新给的***************************
	if(colorvarflag){//按先前设置的选
		//什么都不干，前面已经干了
    }else{//按函数方式选
	//颜色选择配对
	char *dictionary[5] = {"gray","redgreen" ,"greenblue","redblue","free"};//指针数组
    int keynumber;
    for(keynumber=0;keynumber<5;keynumber++)
    {
    if(!strcmp(keyword,dictionary[keynumber]))
	  break;
    }
		 
switch(keynumber){
	//******************模式1红绿变化显示
case 0:{
	//点云上色
	headcolor=colors[0];
	tailcolor=colors[1];
	//相应背景颜色
	backgroudcolor=colors[4];
	break;
	   }
   //******************模式2绿红
case 1:{
	//点云上色
	headcolor.r=0.0;
	headcolor.g=255.0;
	headcolor.b=0.0;
	tailcolor.r=255.0;
	tailcolor.g=0.0;
	tailcolor.b=0.0;
	//相应背景颜色
	backgroudcolor=colors[0];
	break;
	   }
	//******************模式3绿蓝显示
case 2:{
	//点云上色
	headcolor.r=189.0;
	headcolor.g=183.0;
	headcolor.b=107.0;
	tailcolor.r=0.0;
	tailcolor.g=107.0;
	tailcolor.b=139.0;
	//白色
	backgroudcolor=colors[0];
	break;
	   }
	  //******************模式4红蓝显示
case 3:{
	//点云上色
	headcolor.r=9.0;
	headcolor.g=50.0;
	headcolor.b=204.0;
	tailcolor.r=249.0;
	tailcolor.g=50.0;
	tailcolor.b=204.0;
	//于永涛喜欢的背景配色
	backgroudcolor.r=0.33;backgroudcolor.g=0.67;backgroudcolor.b=0.58;
	break;
	   }
	   //******************模式4自由选择颜色
case 4:{
	int cc;
	displaytable();
	std::cout<<"请问大人，头颜色你的决定是? 请输入序号"<<std::endl;
	std::cin>>cc;
	headcolor=colors[cc];
	std::cout<<"请问大人，尾颜色你的决定是? 请输入序号"<<std::endl;
	std::cin>>cc;
	tailcolor=colors[cc];
	//于永涛喜欢的背景配色
	std::cout<<"弱弱的问，背景颜色大人您~~~您要选择哪个?(\\^0^)// 请输入序号,"<<std::endl;
	std::cin>>cc;
	backgroudcolor=colors[cc];
	break;
	   }
default:{
	std::cout<<"Interesting but inpossible! You choose something wrong!"<<std::endl;
	std::cin.get();
	exit(0);
	break;
	}
       }//end switch
    }//end super big if

   //计算特征值的范围，方便归一化
	double minvalue = features[0], maxvalue = features[0];
	for(size_t i=1;i<features.size();++i){
		if(features[i]<minvalue)
		minvalue = features[i];
		if(features[i]> maxvalue)
		maxvalue = features[i];
	}
	//一个彩色点云用于保存颜色信息
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointxyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);

	//PCL RGB格式
	unsigned long pointNum = point3d.size();
    pointxyzrgb->width = pointNum;
	pointxyzrgb->height = 1;
	pointxyzrgb->is_dense = false;
	pointxyzrgb->points.resize(pointxyzrgb->width*pointxyzrgb->height);
    //上色
    for(size_t i=0;i<pointxyzrgb->points.size();i++){
		pointxyzrgb->points[i].x = point3d[i].x;
		pointxyzrgb->points[i].y = point3d[i].y;
		pointxyzrgb->points[i].z = point3d[i].z;
		//计算特征值的变化过程
		//这个配色方案比较适合蓝色背景
	    pointxyzrgb->points[i].r = Colorvar(headcolor.r,tailcolor.r,(features[i]-minvalue)/(maxvalue-minvalue));
	    pointxyzrgb->points[i].g = Colorvar(headcolor.g,tailcolor.g,(features[i]-minvalue)/(maxvalue-minvalue));
	    pointxyzrgb->points[i].b = Colorvar(headcolor.b,tailcolor.b,(features[i]-minvalue)/(maxvalue-minvalue));
	}
	//建立一个PCL的显示对象
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("高程点云显示窗口"));
	//赋色
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointxyzrgb);
	viewer->addPointCloud(pointxyzrgb,rgb,"Features Show");	
	//背景
	viewer->setBackgroundColor(backgroudcolor.r, backgroudcolor.g, backgroudcolor.b);
	//返回窗口
    return(viewer);
	
}//end 
//reload with float type
PointCloudViewerPtr HpdDisplay::Showfeatureresult(PointCloudXYZPtr & pCloud, std::vector<float> & features, char *keyword) {
	//先判断是不是同一个点云集的
	if (pCloud->points.size() != features.size()) {
		std::cout << "Error:点云集和特征集个数不匹配！" << std::endl;
		std::cout << "请按回车结束程序，确认数据问题" << std::endl;
		std::cin.get();
		exit(0);
	}
	//********************判断是用已给的特征值还是用现在新给的***************************
	if (colorvarflag) {//按先前设置的选
					   //什么都不干，前面已经干了
	}
	else {//按函数方式选
		  //颜色选择配对
		char *dictionary[5] = { "grey","redgreen" ,"greenblue","redblue","free" };//指针数组
		int keynumber;
		for (keynumber = 0; keynumber<5; keynumber++)
		{
			if (!strcmp(keyword, dictionary[keynumber]))
				break;
		}

		switch (keynumber) {
			//******************模式1红绿变化显示
		case 0: {
			//点云上色
			headcolor = colors[0];
			tailcolor = colors[1];
			//相应背景颜色
			backgroudcolor = colors[4];
			break;
		}
				//******************模式2绿红
		case 1: {
			//点云上色
			headcolor.r = 0.0;
			headcolor.g = 255.0;
			headcolor.b = 0.0;
			tailcolor.r = 255.0;
			tailcolor.g = 0.0;
			tailcolor.b = 0.0;
			//相应背景颜色
			backgroudcolor = colors[0];
			break;
		}
				//******************模式3绿蓝显示
		case 2: {
			//点云上色
			headcolor.r = 189.0;
			headcolor.g = 183.0;
			headcolor.b = 107.0;
			tailcolor.r = 0.0;
			tailcolor.g = 107.0;
			tailcolor.b = 139.0;
			//白色
			backgroudcolor = colors[0];
			break;
		}
				//******************模式4红蓝显示
		case 3: {
			//点云上色
			headcolor.r = 9.0;
			headcolor.g = 50.0;
			headcolor.b = 204.0;
			tailcolor.r = 249.0;
			tailcolor.g = 50.0;
			tailcolor.b = 204.0;
			//于永涛喜欢的背景配色
			backgroudcolor.r = 0.33; backgroudcolor.g = 0.67; backgroudcolor.b = 0.58;
			break;
		}
				//******************模式4自由选择颜色
		case 4: {
			int cc;
			displaytable();
			std::cout << "请问大人，头颜色你的决定是? 请输入序号" << std::endl;
			std::cin >> cc;
			headcolor = colors[cc];
			std::cout << "请问大人，尾颜色你的决定是? 请输入序号" << std::endl;
			std::cin >> cc;
			tailcolor = colors[cc];
			//于永涛喜欢的背景配色
			std::cout << "弱弱的问，背景颜色大人您~~~您要选择哪个?(\\^0^)// 请输入序号," << std::endl;
			std::cin >> cc;
			backgroudcolor = colors[cc];
			break;
		}
		default: {
			std::cout << "Interesting but inpossible! You choose something wrong!" << std::endl;
			std::cin.get();
			exit(0);
			break;
		}
		}//end switch
	}//end super big if

	 //计算特征值的范围，方便归一化
	double minvalue = features[0], maxvalue = features[0];
	for (size_t i = 1; i<features.size(); ++i) {
		if (features[i]<minvalue)
			minvalue = features[i];
		if (features[i]> maxvalue)
			maxvalue = features[i];
	}
	//一个彩色点云用于保存颜色信息
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointxyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);

	//PCL RGB格式
	unsigned long pointNum = pCloud->points.size();
	pointxyzrgb->width = pointNum;
	pointxyzrgb->height = 1;
	pointxyzrgb->is_dense = false;
	pointxyzrgb->points.resize(pointxyzrgb->width*pointxyzrgb->height);
	//上色
	for (size_t i = 0; i<pointxyzrgb->points.size(); i++) {
		pointxyzrgb->points[i].x = pCloud->points[i].x;
		pointxyzrgb->points[i].y = pCloud->points[i].y;
		pointxyzrgb->points[i].z = pCloud->points[i].z;
		//计算特征值的变化过程
		//这个配色方案比较适合蓝色背景
		pointxyzrgb->points[i].r = Colorvar(headcolor.r, tailcolor.r, (features[i] - minvalue) / (maxvalue - minvalue));
		pointxyzrgb->points[i].g = Colorvar(headcolor.g, tailcolor.g, (features[i] - minvalue) / (maxvalue - minvalue));
		pointxyzrgb->points[i].b = Colorvar(headcolor.b, tailcolor.b, (features[i] - minvalue) / (maxvalue - minvalue));
	}
	//建立一个PCL的显示对象
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("高程点云显示窗口"));
	//赋色
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointxyzrgb);
	viewer->addPointCloud(pointxyzrgb, rgb, "Features Show");
	//背景
	viewer->setBackgroundColor(backgroudcolor.r, backgroudcolor.g, backgroudcolor.b);
	//返回窗口
	return(viewer);

}//end 

PointCloudViewerPtr HpdDisplay::Showfeatureresult(pcl::PointCloud<pcl::PointXYZ> & vCloud, std::vector<float> & features, char *keyword) {
	//先判断是不是同一个点云集的
	if (vCloud.points.size() != features.size()) {
		std::cout << "Error:点云集和特征集个数不匹配！" << std::endl;
		std::cout << "请按回车结束程序，确认数据问题" << std::endl;
		std::cin.get();
		exit(0);
	}
	//********************判断是用已给的特征值还是用现在新给的***************************
	if (colorvarflag) {//按先前设置的选
		//什么都不干，前面已经干了
	}
	else {//按函数方式选
		//颜色选择配对
		char *dictionary[5] = { "grey", "redgreen", "greenblue", "redblue", "free" };//指针数组
		int keynumber;
		for (keynumber = 0; keynumber<5; keynumber++)
		{
			if (!strcmp(keyword, dictionary[keynumber]))
				break;
		}

		switch (keynumber) {
			//******************模式1红绿变化显示
		case 0: {
			//点云上色
			headcolor = colors[0];
			tailcolor = colors[1];
			//相应背景颜色
			backgroudcolor = colors[4];
			break;
		}
			//******************模式2绿红
		case 1: {
			//点云上色
			headcolor.r = 0.0;
			headcolor.g = 255.0;
			headcolor.b = 0.0;
			tailcolor.r = 255.0;
			tailcolor.g = 0.0;
			tailcolor.b = 0.0;
			//相应背景颜色
			backgroudcolor = colors[0];
			break;
		}
			//******************模式3绿蓝显示
		case 2: {
			//点云上色
			headcolor.r = 189.0;
			headcolor.g = 183.0;
			headcolor.b = 107.0;
			tailcolor.r = 0.0;
			tailcolor.g = 107.0;
			tailcolor.b = 139.0;
			//白色
			backgroudcolor = colors[0];
			break;
		}
			//******************模式4红蓝显示
		case 3: {
			//点云上色
			headcolor.r = 9.0;
			headcolor.g = 50.0;
			headcolor.b = 204.0;
			tailcolor.r = 249.0;
			tailcolor.g = 50.0;
			tailcolor.b = 204.0;
			//于永涛喜欢的背景配色
			backgroudcolor.r = 0.33; backgroudcolor.g = 0.67; backgroudcolor.b = 0.58;
			break;
		}
			//******************模式4自由选择颜色
		case 4: {
			int cc;
			displaytable();
			std::cout << "请问大人，头颜色你的决定是? 请输入序号" << std::endl;
			std::cin >> cc;
			headcolor = colors[cc];
			std::cout << "请问大人，尾颜色你的决定是? 请输入序号" << std::endl;
			std::cin >> cc;
			tailcolor = colors[cc];
			//于永涛喜欢的背景配色
			std::cout << "弱弱的问，背景颜色大人您~~~您要选择哪个?(\\^0^)// 请输入序号," << std::endl;
			std::cin >> cc;
			backgroudcolor = colors[cc];
			break;
		}
		default: {
			std::cout << "Interesting but inpossible! You choose something wrong!" << std::endl;
			std::cin.get();
			exit(0);
			break;
		}
		}//end switch
	}//end super big if

	//计算特征值的范围，方便归一化
	double minvalue = features[0], maxvalue = features[0];
	for (size_t i = 1; i<features.size(); ++i) {
		if (features[i]<minvalue)
			minvalue = features[i];
		if (features[i]> maxvalue)
			maxvalue = features[i];
	}
	//一个彩色点云用于保存颜色信息
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointxyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);

	//PCL RGB格式
	unsigned long pointNum = vCloud.points.size();
	pointxyzrgb->width = pointNum;
	pointxyzrgb->height = 1;
	pointxyzrgb->is_dense = false;
	pointxyzrgb->points.resize(pointxyzrgb->width*pointxyzrgb->height);
	//上色
	for (size_t i = 0; i<pointxyzrgb->points.size(); i++) {
		pointxyzrgb->points[i].x = vCloud.points[i].x;
		pointxyzrgb->points[i].y = vCloud.points[i].y;
		pointxyzrgb->points[i].z = vCloud.points[i].z;
		//计算特征值的变化过程
		//这个配色方案比较适合蓝色背景
		pointxyzrgb->points[i].r = Colorvar(headcolor.r, tailcolor.r, (features[i] - minvalue) / (maxvalue - minvalue));
		pointxyzrgb->points[i].g = Colorvar(headcolor.g, tailcolor.g, (features[i] - minvalue) / (maxvalue - minvalue));
		pointxyzrgb->points[i].b = Colorvar(headcolor.b, tailcolor.b, (features[i] - minvalue) / (maxvalue - minvalue));
	}
	//建立一个PCL的显示对象
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("高程点云显示窗口"));
	//赋色
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointxyzrgb);
	viewer->addPointCloud(pointxyzrgb, rgb, "Features Show");
	//背景
	viewer->setBackgroundColor(backgroudcolor.r, backgroudcolor.g, backgroudcolor.b);
	//返回窗口
	return(viewer);

}//end 

PointCloudViewerPtr  HpdDisplay::Creatgeometry(PointCloudXYZPtr & cloud,Point3D & centerp){
	//球的圆心
	pcl::PointXYZ o;
	o.x=centerp.x;
	o.y=centerp.y;
	o.z=centerp.z;
	//建立一个PCL的显示对象
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("高程点云显示窗口"));
	//PCL赋色
	viewer->addPointCloud(cloud,"Key points");	
	viewer->addSphere(o,0.1,"sphere",0);
	//背景
	viewer->setBackgroundColor(backgroudcolor.r, backgroudcolor.g, backgroudcolor.b);
	//返回窗口
    return(viewer);
}
/*================================
random函数简介
功能：随机数主要用于邻域显示函数的点随机选择显示和分类中类别颜色的随机显示
=================================*/
//一个产生随机数的函数，可以规定随机数范围
double HpdDisplay::random(double start, double end)
{
    return start+(end-start)*rand()/(RAND_MAX + 1.0);
}

//
/*================================
displaytable()函数简介
功能：不解释
=================================*/
void HpdDisplay::displaytable(){
	std::cout<<"      颜色表如下："<<std::endl;
	for(size_t i=0;i<colors.size();i++)
		std::cout<<"序号"<<i<<"是"<<colors[i].name<<std::endl;
}
/*================================
Colorvar()函数简介
功能：计算渐变颜色中对应的颜色值
输入：headnum 渐变颜色的头颜色单值，可以是r,g,b
      tailnum 尾颜色
	  proportion 属性值在原来集中的位置比例
输出：所属颜色值
=================================*/
float Colorvar(float headnum,float tailnum,float proportion){
	return headnum+(tailnum-headnum)*proportion;
}

/*================================
AddLineWithPoints
=================================*/
void HpdDisplay::AddLineWithPoints(boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer,
		                           std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & vSkelinePoints,
	                               float fSRadius, float fLr, float fLg, float fLb,
	                               float fSr, float fSg, float fSb) {
	
	for (int k = 0; k != vSkelinePoints.size(); ++k) {

		for (int i = 0; i != vSkelinePoints[k]->points.size() - 1; ++i) {
			
			std::stringstream sLinestream;
			sLinestream << k << "_" << i << "_" << i + 1 << "_Line";
			std::string sLineName;
			sLinestream >> sLineName;

			viewer->addLine<pcl::PointXYZ>(vSkelinePoints[k]->points[i], vSkelinePoints[k]->points[i + 1], fLr, fLg, fLb, sLineName);
			
			std::stringstream sSpherestream;
			sSpherestream << k << "_" << i << "_Sphere";
			std::string sSphereName;
			sSpherestream >> sSphereName;
			viewer->addSphere(vSkelinePoints[k]->points[i], fSRadius, fSr, fSg, fSb, sSphereName);
		}//end for i

		int iTailNum = vSkelinePoints[k]->points.size() - 1;
		std::stringstream sTailStream;
		sTailStream << k << "_" << iTailNum << "_Sphere";
		std::string sTailName;
		sTailStream >> sTailName;
		viewer->addSphere(vSkelinePoints[k]->points[iTailNum], fSRadius, fSr, fSg, fSb, sTailName);

	}//end for k

}

/*================================
AddSphereAtPoints
=================================*/
void HpdDisplay::AddSphereAtPoints(boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer,
	pcl::PointCloud<pcl::PointXYZ>::Ptr & vSkelinePoints,
	float fSRadius, float fSr, float fSg, float fSb) {

	for (int k = 0; k != vSkelinePoints->points.size(); ++k) {

			std::stringstream sSpherestream;
			sSpherestream << k << "_ball";
			std::string sSphereName;
			sSpherestream >> sSphereName;
			viewer->addSphere(vSkelinePoints->points[k], fSRadius, fSr, fSg, fSb, sSphereName);

	}//end for k

}