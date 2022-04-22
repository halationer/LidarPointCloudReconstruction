/*========================
Huang pengdi的数学公式函数头文件
包含一些会用到的数学公式，方便调用
Firstly edited in 09.12.2014 secondly edited 05.24.2015
I mean large scale edition time
呵呵
==========================*/
#ifndef HPDMATH_H //防重复定义头文件
#define HPDMATH_H
#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <algorithm>
#include <pcl/common/eigen.h>
#define PI 3.1415926
using namespace std;
//卷积用的类
struct convector{
   float* pStart;
   int length;
}; 
//wavelet
class Hpdwavelet{

public:
//*******************初始函数********************
Hpdwavelet();
void clear(char* keyword="same");
void Setemplv(float f_rm,int);
//*******************功能函数********************
//差分函数
void Diff(std::vector<float> &);
//小波生成函数（只有Marr小波，因为要用到）
void Conswavtemplate(int num=6,float m=-2.0,char* funame="marr");
//向量范数计算函数
float Normvector(std::vector<float> & nvect,int p=2);
//计算卷积的函数
std::vector<float> Conv(std::vector<float> &,std::vector<float> &);
//保持信号长度
std::vector<float> wkeep( const std::vector<float> &v, int length,
                    char* direction="center" );
//获取奇点
int Findsigularity(std::vector<float> & orisignals);
//数据
std::vector<float> moduleresults;//模值结果
private:
//存储变量
    std::vector<float> diffarray;//差分变量
//模板
    int ln;//模板长度
	float rm;
	bool flagm;
    std::vector<float> waveletemplate;
//卷积结果
    std::vector<float> convresult;

};
//*****************************************全局函数**********************************
/*===========================
Maxofall函数
作用：获取向量内的最大值的索引
============================*/ 
int Maxofall(std::vector<float> & absvector);
int Minofall(std::vector<float> & vec);
/*===========================
Normvector函数
作用：获取向量的模值
============================*/ 
double Normvector(std::vector<double> & nvect,int p);
/*===========================
Removerepetitionr函数
作用：去除向量内相同的元素
============================*/ 
void Removerepetition();
/*===========================
Gomputedis函数
作用：获取向量内的最大值，不是最大值的索引
============================*/ 
template <typename T>
T Getmax(std::vector<T> & vec){
	T max;
	max=vec[0];
	for(size_t i=0;i!=vec.size();i++){
	if(vec[i]>max)
	max=vec[i];
	}
	return max;
}
template <typename T>
std::vector<T> Getmaxmin(std::vector<T> & vec){
	std::vector<T> maxmin;
	T max;
	T min;
	//if non-empty
	if(vec.size()){
	    max=vec[0];
	    min=vec[0];
	   for(size_t i=0;i!=vec.size();i++){
	   if(vec[i]>max)
	      max=vec[i];
	   if(vec[i]<min)
		  min=vec[i];
	}//end for
	    maxmin.push_back(max);
		maxmin.push_back(min);
	}else{//if empty
		maxmin.push_back(0.0);
		maxmin.push_back(0.0);
	}
		return maxmin;
}
/*===========================
Gomputedis函数
作用：计算两点之间的欧式距离
============================*/
double Gomputedis(double &, double &,double &, double &, double &,double &);
double Gomputedis(double &, double &, double &, double &);
/*===============================
Trangletoradian函数
角度转成弧度
===================================*/
double Trangletoradian(double &);
/*===============================
computeanglecosine函数
角度转成弧度
===================================*/
double Computeanglecosine(Eigen::Vector3f & vecone,Eigen::Vector3f & vectwo);
double Computeanglecosine(Eigen::Vector2d & vecone,Eigen::Vector2d & vectwo);
/*=======================================
Removevectormember函数
功能:去除制定的vector内元素
输入:容器vector和准备去除的索引遍历val或indices
========================================*/
template <typename T>
void Removevectormember(vector<T>& v,int val){
    vector<T>::iterator ite=v.begin();
    while(ite!=v.end()){
        if(*ite==val)
            ite=v.erase(ite);
        else
            ++ite;
    }
}
//函数重载有indices的形式
template <typename T>
void Removevectormember(vector<T>& v,std::vector<bool> indices){
    vector<T>::iterator ite=v.begin();
	int n=0;
    while(ite!=v.end()){
        if(!indices[n])
            ite=v.erase(ite);
        else
            ++ite;
		n++;
    }
}
/*===============================
Numofsamenber函数
寻找两个向量相同元素的个数
注：两个向量内元素都不重复
===================================*/
double Numofsamenber(std::vector<int> &,std::vector<int> &);




#endif