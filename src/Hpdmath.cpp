#include"Hpdmath.h"
//******Edited by Huang Pengdi 2014.12.09 Xiamen Unversity******
/*===================================================
构造函数Hpdwavelet()

=================================================*/
Hpdwavelet::Hpdwavelet(){
	flagm=false;
	ln=0;
	rm=0.0;
}
/*===================================================
函数Setscalem简介
设置尺度m
=================================================*/
void Hpdwavelet::Setemplv(float f_rm,int f_ln){
	ln=f_ln;
	rm=f_rm;
	flagm=true;
}
/*=================================================
函数Diff()简介：计算信号的差分结果
输入：原始信号 std::vector<float> & orifun
输出：差分信号 diffarray
==================================================*/
void Hpdwavelet::Diff(std::vector<float> & orifun){
	diffarray.clear();
	//前向差分就这么简单
	for(size_t i=1;i<orifun.size();i++){
		diffarray.push_back(orifun[i]-orifun[i-1]);
	}
	//
};
/*==============================================
函数clear简介：清理类内的值
输入：清除几乎所有类内的值
一般用不到
===============================================*/
void Hpdwavelet::clear(char* keyword){
	diffarray.clear();//差分变量
    moduleresults.clear();
	convresult.clear();//卷积结果
	if (!strcmp(keyword,"all"))
	waveletemplate.clear();
}
/*==============================================
函数Conswavtemplate简介：小波生成函数
输入：num小波模板大小,m分辨率,funame小波类型
输出：waveletemplate小波信号
===============================================*/
void Hpdwavelet::Conswavtemplate(int num,float m,char* funame){
	if(flagm){
	m=rm;num=ln;
	}
	//进入正题
	if (!strcmp(funame,"marr")){
	waveletemplate.clear();
	//多尺度
		float delta=pow(2,m);
	//模
    //幅度（num长度）
	float amplitude=-1/sqrt(2*PI);
	//模型
	float x;
	for(int n=0;n!=num;n++){
    //构造高斯函数的偏导数
	x=float(n+1)-float(num+1)/2;
	waveletemplate.push_back(amplitude*(x/pow(delta,2))*exp(-pow(x,2)/(2*pow(delta,2))));
	//
	      }//end for
	float normvect=Normvector(waveletemplate);
	for(int i=0;i!=num;i++){
	waveletemplate[i]=waveletemplate[i]/normvect;
	   }//end for
	}//end if
	if(!strcmp(funame,"harr")){
	/*正在施工。。。。。*/
	}
}
/*===============================================
函数Normvector简介：向量范数函数
输入：std::vector<float> & nvect向量，p范数（默认2范数）
输出：sum范数值
===============================================*/
float Hpdwavelet::Normvector(std::vector<float> & nvect,int p){
	if(p<0)
	p=2;//违反格式强行置换
	float sum=0.0;
	//p范数
	for(size_t i=0;i!=nvect.size();i++){
	sum=pow(fabs(nvect[i]),float(p))+sum;
	}
	sum=pow(sum,1/float(p));
	return sum;
}
/*===============================================
函数Conv简介：卷积
输入：std::vector<float> & nvect向量，p范数（默认2范数）
输出：sum范数值
===============================================*/
std::vector<float> Hpdwavelet::Conv(std::vector<float> & avector,std::vector<float> & bvector){
	//分配给函数的计算单元
 struct convector a,b,y;
 a.length =avector.size();
 b.length =bvector.size();
 a.pStart =new float [a.length];
 b.pStart =new float [b.length];
   //疯狂赋值
 for(size_t i=0;i!=avector.size();i++){
 *(a.pStart+i)=avector[i];
 }
 for(size_t i=0;i!=bvector.size();i++){
 *(b.pStart+i)=bvector[i];
 }
	//初始化
 std::vector<float> yvector;
 float aa,bb;
 y.length =a.length +b.length-1 ;
 y.pStart =new float [y.length];

 for (int i=0;i<y.length ;i++)
 *(y.pStart +i)=0;

 if (a.length >=b.length ){  //如果a的长度大于或等于b的长度
 for (int i=0;i<b.length;i++){
    for (int j=0;j<=i;j++){
     aa=*(a.pStart+i-j);
     bb=*(b.pStart +j);
     *(y.pStart +i)=*(y.pStart +i)+aa*bb;
    }
 }

 for (int i=b.length;i<a.length;i++){
    for (int j=0;j<b.length;j++){
        aa=*(a.pStart+i-j);
        bb=*(b.pStart +j);
        *(y.pStart +i)=*(y.pStart +i)+aa*bb;
     }
 }

 for (int i=a.length;i<a.length+b.length;i++){
    for (int j=i-a.length+1;j<b.length;j++){
      aa=*(a.pStart+i-j);
      bb=*(b.pStart +j);
      *(y.pStart +i)=*(y.pStart +i)+aa*bb;
     }
    }

 }//end if
 else  //如果b的长度大于或等于a的长度
{
for(int i=0;i<a.length;i++){
     for (int j=0;j<=i;j++){
     bb=*(b.pStart+i-j);
     aa=*(a.pStart +j);
     *(y.pStart +i)=*(y.pStart +i)+aa*bb;
    }
 }

 for (int i=a.length;i<b.length;i++){
    for (int j=0;j<a.length;j++){
       bb=*(b.pStart+i-j);
       aa=*(a.pStart +j);
       *(y.pStart +i)=*(y.pStart +i)+aa*bb;
    }
 }

 for (int i=b.length;i<b.length+a.length-1;i++){
     for (int j=i-b.length+1;j<a.length;j++){
       bb=*(b.pStart+i-j);
       aa=*(a.pStart +j);
       *(y.pStart+i)=*(y.pStart +i)+aa*bb;
    }
 }
 }//end else
 //输出赋值
 for(int i=0;i<y.length;i++)
	 yvector.push_back(*(y.pStart+i));
 //麻烦的善后工作
 delete [] a.pStart ;
 delete [] b.pStart ;
 delete [] y.pStart ;
 return(yvector);
 } 
/*===============================================
/**网上找的wkeep保持卷积后正确长度的函数
 * Keep part of vector.
 * For a vector, w = wkeep(v,L,opt) extracts the vector w from the vector v.
 * The length of w is L. If direction = "center" ("left", "rigth",
 * respectively), w is the central (left, right, respectively) part of v.
 * w = wkeep(x,L) is equivalent to w = wkeep(v,L,"center").
 * w = wkeep(v,L,first) returns the vector v[first] to v[first+L-1].
===============================================*/
std::vector<float> Hpdwavelet::wkeep( const std::vector<float> &v, int length,
                    char* direction ){
    int lv = v.size();
	std::vector<float> tmp(length);

    if( ( 0 <= length ) && ( length <= lv ) )
    {
        if(!strcmp(direction,"right")) 
            for( int i=0; i<length; ++i )
                tmp[i] = v[lv-length+i];
        else if(!strcmp(direction,"left")) 
            for( int i=0; i<length; ++i )
                tmp[i] = v[i];
        else
        {
            int first = (lv-length)/2;
            for( int i=0; i<length; ++i )
                tmp[i] = v[first+i];
        }

        return tmp;
    }
    else
    {
        cerr << "Invalid length input." << endl;
        return tmp;
    }
}
/*==========================
函数Maxofall
形参1：std::vector<float>的数组
输出：输出最大值的索引（注意是索引啊）
==========================*/
int Maxofall(std::vector<float> & absvector){
   int pos=0;
   float max=-DBL_MAX;
   for(int i=0;i!=absvector.size();i++){
       if(absvector[i]>max){
          max=absvector[i];
          pos=i;//给坐标
       }
   }
   return pos;
}
/*==========================
函数Minofall
形参1：std::vector<float>的数组
输出：输出最小的索引（注意是索引啊）
==========================*/
int Minofall(std::vector<float> & vec){
   int pos=0;
   float min=DBL_MAX;
   for(int i=0;i!=vec.size();i++){
       if(vec[i]<min){
          min=vec[i];
          pos=i;//给坐标
       }
   }
   return pos;
}
/*==========================
函数Findsigularity
形参1：std::vector<float>的数组
输出：输出最大值的索引（注意是索引啊）
==========================*/
int Hpdwavelet::Findsigularity(std::vector<float> & orisignals){
	//先差分获取一阶导数突变函数
	Diff(orisignals);
	//进行小波生成
	Conswavtemplate();
	//卷积进行小波分析
	convresult.clear();
	//考虑到第一个值突变太大了
	float veczero=diffarray[0];
	for(size_t i=0;i!=diffarray.size();i++)
    diffarray[i]=diffarray[i]-veczero;
	//卷积coven...
	convresult=Conv(diffarray,waveletemplate);
	//核对到原来的信号上
	moduleresults.clear();
	moduleresults=wkeep(convresult,diffarray.size());
	//取模值
	for(size_t i=0;i!=moduleresults.size();i++){
	moduleresults[i]=fabs(moduleresults[i]);
	}
	//获取最大值位置
	int numres=Maxofall(moduleresults);
	return numres+1;//这是因为差分函数比原函数少一个值（第一个值）
}
/*==========================
函数Normvector求向量模值，这可是很常用的哦
形参1：std::vector<float>的数组
输出：输出最大值的索引（注意是索引啊）
==========================*/
double Normvector(std::vector<double> & nvect,int p){
	if(p<0)
	p=2;//违反格式强行置换
	double sum=0.0;
	//p范数
	for(size_t i=0;i!=nvect.size();i++){
	sum=pow(fabs(nvect[i]),double(p))+sum;
	}
	sum=pow(sum,1/double(p));
	return sum;
}
/*==========================
函数Removerepetition
去除向量内相同的元素
==========================*/
void Removerepetition(std::vector<int> & iv){
//排序
sort(iv.begin(),iv.end());  
//删除相邻的相同值
vector<int>::iterator ix=unique(iv.begin(),iv.end()); 
iv.erase(ix,iv.end()); 
}
/*==========================
函数Gomputedis求向量模值，这可是很常用的哦
形参1：向量1的x,y,z值
形参2：向量2的x,y,z值
输出：欧式距离（注意是索引啊）
重载为计算二维的
==========================*/
double Gomputedis(double & x1, double & y1,double & z1, double & x2, double & y2,double & z2){
	return sqrt(pow(x1-x2,2.0)+pow(y1-y2,2.0)+pow(z1-z2,2.0));
}
double Gomputedis(double & x1, double & y1, double & x2, double & y2){
	return sqrt(pow(x1-x2,2.0)+pow(y1-y2,2.0));
}
/*===============================
Trangletoradian函数
角度转成弧度
===================================*/
double Trangletoradian(double & angle){
	return angle*PI/180.0;
}
/*===============================
Trangletoradian函数
角度转成弧度
===================================*/
double Computvanglecosine(Eigen::Vector3f & vecone,Eigen::Vector3f & vectwo){
	double dotres=vecone.dot(vectwo);
	return dotres/(vecone.norm()*vectwo.norm());
}
double Computeanglecosine(Eigen::Vector2d & vecone,Eigen::Vector2d & vectwo){
	double dotres=vecone.dot(vectwo);
	return dotres/(vecone.norm()*vectwo.norm());
}

double Numofsamenber(std::vector<int> & tarvec,std::vector<int> & modvec){
	int n=0;
	for(int i=0;i!=tarvec.size();i++){
		for(int j=0;j!=modvec.size();j++){
			if(tarvec[i]==modvec[j]){
			n++;
			break;
			}
	    }
	}
	return double(n)/double(tarvec.size());
}