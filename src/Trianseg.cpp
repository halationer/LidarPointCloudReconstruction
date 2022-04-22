#include"Trianseg.h"
#include"Hpdmath.h"

/*=================================
Trianseg函数

==================================*/
Trianseg::Trianseg(){
	pointflag=false;
}
/*=================================
Setopandvpoint函数
输入：两个坐标点（一个是三角形顶点，另一个垂直平分线相交点）
输出：中轴线距离
功能：获取两个基准点从而为构建三角形做准备
==================================*/
void Trianseg::Setopandvpoint(Point3D & f_viewpoint,Point3D & f_targetpoint){
	viewpoint=f_viewpoint;
	targetpoint=f_targetpoint;
	//计算中轴线，就是中间的棍状线
	axisldis=Gomputedis(viewpoint.x,viewpoint.y,targetpoint.x,targetpoint.y);
	//反旗
	pointflag=true;
}
/*=================================
Getrivetrexs函数
输入：无
输出：三角形三个顶点的数据坐标下结果
功能：获取三角形数据坐标下的三个顶点
==================================*/
std::vector<Point3D> Trianseg::Getrivetrexs(){
	//转换成point3d格式
	Point3D vertex={0.0,0.0,0.0,0};
	std::vector<Point3D> trivetvecs;
	vertex.x=trianvertex[0](0);
	vertex.y=trianvertex[0](1);
	trivetvecs.push_back(vertex);//原点的数据坐标
	vertex.x=trianvertex[1](0);
	vertex.y=trianvertex[1](1);
	trivetvecs.push_back(vertex);//顶点2
	vertex.x=trianvertex[2](0);
	vertex.y=trianvertex[2](1);
	trivetvecs.push_back(vertex);//顶点3
	return trivetvecs;
}
/*=================================
Generatetrian函数
输入：人眼视觉水平角，以及透视距离，即要看多远哈
      double dis 中轴线的长度，或是中轴线延伸的长度
      double angle 视角范围，度数非弧度，舒适视域60，立体可见124
	  bool distype 选择直接设置长度还是添加长度balse添加长度,true是直接设置长度
输出：等腰三角形的三个顶点值（在数据坐标系下）
==================================*/
void Trianseg::Generatetrian(double width,double anglethr,bool distype){
	//先判断是否已经输入重要的点位
	if(!pointflag){
		std::cout<<"The loaction of view point and target point have no been input!!"<<std::endl;
	    std::cout<<"Please check!! Enter to end this programe!"<<std::endl;//真啰嗦
		std::cin.get();
		exit(0);
	}
	double angle;
	//设置中轴线长度，reset distance of the axisline of triangle
	if(distype){
	    axisldis=axisldis+width;
	    angle=Trangletoradian(anglethr)/2.0;
	}else{
		if((width/axisldis)>=1){
		angle=89.0;
		angle=Trangletoradian(angle);
		}else
		angle=asin(width/axisldis);
		//
		if(angle<(Trangletoradian(anglethr)/2.0))
		angle=Trangletoradian(anglethr)/2.0;
		axisldis=axisldis+width;
	}
	//获得两顶点坐标
	//水平视角分左右两边的呀
	double oppside=axisldis*tan(angle);
	//****************建立几何变换矩阵*******************
	//平移分量
	double Tx=viewpoint.x-0.0; //x'=x+Tx==>>Tx=x'-x
	double Ty=viewpoint.y-0.0; //y'=y+Ty==>>Ty=y'-y
	//旋转分量
	Eigen::Vector2d axisvec(0.0,axisldis);//算夹角无关长度
	Eigen::Vector2d targetvec(targetpoint.x-Tx,targetpoint.y-Ty);
	double rotanle=acos(Computeanglecosine(axisvec,targetvec));
	//判断旋转后的分量在哪个象限，从而确定是角度是正还是负
	if(targetpoint.x-Tx>0)//顺时针为负，逆时针为正
		rotanle=-rotanle;
	//生成待变换矩阵
	//烦，小东西很挺麻烦的，矩阵乘法献上
	Eigen::Matrix<double,2,3> rawvertex;
	rawvertex<<oppside,axisldis,1.0,-oppside,axisldis,1.0;//按照原始坐标行向量排列
	//std::cout<<rawvertex<<endl;
	//***生成变换矩阵***先旋转后平移
	//旋转矩阵
	Eigen::Matrix3d rotam;
	rotam<<cos(rotanle),sin(rotanle),0.0,-sin(rotanle),cos(rotanle),0.0,0.0,0.0,1.0;
	//std::cout<<rotam<<endl;
	//变换后矩阵
	Eigen::Matrix<double,2,3> rotavertex=rawvertex*rotam;
	//std::cout<<rotavertex<<endl;
    //平移矩阵
	Eigen::Matrix3d transm;
	transm<<1.0,0.0,0.0,0.0,1.0,0.0,Tx,Ty,1.0;
	//std::cout<<transm<<endl;
	rotavertex=rotavertex*transm;//矩阵乘法变换，先平移后旋转，要记住啊
	//std::cout<<rotavertex<<endl;
	//赋值，对三个值都赋值
	//私有向量里的trianvertex
	
	Eigen::Vector2d onetex(viewpoint.x,viewpoint.y);//顶点1
	trianvertex.push_back(onetex);
	//std::cout<<onetex<<std::endl;
	onetex<<rotavertex(0,0),rotavertex(0,1); //顶点2
	trianvertex.push_back(onetex);
	//std::cout<<onetex<<std::endl;;
	onetex<<rotavertex(1,0),rotavertex(1,1); //顶点3
	trianvertex.push_back(onetex);
	//std::cout<<onetex<<std::endl;;
}
/*=================================
Generatetrian函数
输入：人眼视觉水平角，以及透视距离，即要看多远哈
输出：三角形的三个顶点
==================================*/
void Trianseg::Clear(bool deltran){
	pointflag=false;
	axisldis=0.0;
	if(deltran)
	trianvertex.clear();
}
/*=================================
Getriacludepoint函数
输入：std::vector<Point3D>格式的三维点云数据raw point3d
输出：std::vector<int> 的三角形区域内索引indices
==================================*/
std::vector<int> Trianseg::Getriacludepoint(std::vector<Point3D> & point3d){
	//判断三角形区域是否生成
	if(!trianvertex.size()){
	    std::cout<<"Sorry,The triangle have no been build!!"<<std::endl;
	    std::cout<<"Please check!! Enter to end this programe!"<<std::endl;//真啰嗦
		std::cin.get();
		exit(0);
	}
	//什么都不剩下
	std::vector<int> triandices;
	/*如果是三角形内的点满足如下三个条件,有方程P = A +  u * (C – A) + v * (B - A)
	其中A,B,C是三个顶点，P是查询点
    (1)u>=0
	(2)v>=0
    (3)u+v<=1                                      */
	//v0=C-A,v1=B-A,v2=P-A
    Eigen::Vector2d v0=trianvertex[2]-trianvertex[0];
    Eigen::Vector2d v1=trianvertex[1]-trianvertex[0];
	//各子项
    double dot00=v0.dot(v0);
    double dot01=v0.dot(v1);
    double dot11=v1.dot(v1);
	//公式
	//u = ((v1•v1)(v2•v0)-(v1•v0)(v2•v1)) / ((v0•v0)(v1•v1) - (v0•v1)(v1•v0))
	//v = ((v0•v0)(v2•v1)-(v0•v1)(v2•v0)) / ((v0•v0)(v1•v1) - (v0•v1)(v1•v0))
    float inverDeno=1/(dot00*dot11-dot01*dot01);
	//以下需要查询点
	//*****************************遍历所有点****************************
	for(int i=0;i!=point3d.size();i++){
    //二维投影
	Eigen::Vector2d queryp(point3d[i].x,point3d[i].y);
	//计算有关查询点的东西
    Eigen::Vector2d v2=queryp-trianvertex[0];
	//需要查询点参与的
	double dot02=v0.dot(v2);
    double dot12=v1.dot(v2);
	//u值判断
    float u=(dot11*dot02-dot01*dot12)*inverDeno;
    if(u<0||u>1) // if u out of range, return directly
       continue;//next
	//v值判断
    float v = (dot00*dot12-dot01*dot02)*inverDeno;
    if(v<0||v>1) // if v out of range, return directly
       continue;//next
	//u + v <= 1成立就正，不成立就是负
    if(u+v<=1)//next or save
	triandices.push_back(i);
	//是的话保存索引
	}
	return triandices;
}
/*=================================
PointinTriangle函数
输入：Eigen::Vector2d & A 三角形顶点1，用作原点
      Eigen::Vector2d & B 三角形顶点2
      Eigen::Vector2d & C 三角形顶点3
      Eigen::Vector2d & P 查询点
输出：bool 是否是内点，true为是
功能：判断点是否在三角形内
==================================*/
bool PointinTriangle(Eigen::Vector2d & A, Eigen::Vector2d & B,Eigen::Vector2d & C, Eigen::Vector2d & P)
{
	/*如果是三角形内的点满足如下三个条件,有方程P = A +  u * (C – A) + v * (B - A)
	其中A,B,C是三个顶点，P是查询点
    (1)u>=0
	(2)v>=0
    (3)u+v<=1                                      */
	//v0=C-A,v1=B-A,v2=P-A
    Eigen::Vector2d v0=C-A;
    Eigen::Vector2d v1=B-A;
	//各子项
    double dot00=v0.dot(v0);
    double dot01=v0.dot(v1);
    double dot11=v1.dot(v1);
	//公式
	//u = ((v1•v1)(v2•v0)-(v1•v0)(v2•v1)) / ((v0•v0)(v1•v1) - (v0•v1)(v1•v0))
	//v = ((v0•v0)(v2•v1)-(v0•v1)(v2•v0)) / ((v0•v0)(v1•v1) - (v0•v1)(v1•v0))
    float inverDeno=1/(dot00*dot11-dot01*dot01);
	//以下需要查询点
    Eigen::Vector2d v2=P-A;
	//需要查询点参与的
	double dot02=v0.dot(v2);
    double dot12=v1.dot(v2);
	//u值判断
    float u=(dot11*dot02-dot01*dot12)*inverDeno;
    if(u<0||u>1) // if u out of range, return directly
    {
        return false;
    }
	//v值判断
    float v = (dot00*dot12-dot01*dot02)*inverDeno;
    if(v<0||v>1) // if v out of range, return directly
    {
        return false;
    }
	//u + v <= 1成立就正，不成立就是负
    return u+v<=1;
}
//void(){
// int verCount=polygon_.size ();
//   if(verCount<2)
//       return false;
//   bool inside=false;
//
//   cl::geometry::IPoint2D A=polygon_[0];
//   for(int i=1;i<=verCount;++i)
//   {
//       cl::geometry::IPoint2D B=polygon_[i%verCount];
//       if(((B.y<=((int)pt.y))&&(((int)pt.y)<A.y))||((A.y<=((int)pt.y))&&(((int)pt.y)<B.y)))
//       {
//           double ABy=static_cast<double>(A.y-B.y);
//           double t=static_cast<double>((pt.x-B.x)*ABy-(A.x-B.x)*(pt.y-B.y));
//           if(ABy<0)
//               t=-t;
//           if(t<0)
//               inside=!inside;
//       }
//       A=B;
//   }
//   return inside;
//}