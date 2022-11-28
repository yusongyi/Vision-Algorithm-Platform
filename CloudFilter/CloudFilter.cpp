#include "pch.h"
#include "CloudFilter.h" 
#include <iostream> 
#define _CRT_SECURE_NO_WARNINGS
#define  Defultfilename  "CloudFilter"
typedef pcl::PointXYZ PointT;
using namespace pcl;
using namespace std;
#define EPSILON 0.001 //根据精度需要
 

 
/*

直通滤波器

	params[0]:
		//0 对x轴进行操作
		//1 对y轴进行操作
		//2 对z轴进行操作
	params[1]:
		//设置直通滤波器操作范围-min
	params[2]:
		//设置直通滤波器操作范围-max
	params[3]:
		//1表示保留范围内，0表示保留范围外

*/
int CloudFilter(pcl::PointCloud<PointT>::Ptr cloud, NodeInput** inputs, NodeOutput** outputs, float* params)
{
  
 

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	 

 

	coefficients->values.push_back(999);
	coefficients->values.push_back(999);
	coefficients->values.push_back(999);



	outputs[0]->coeff = coefficients;

	return 0;
}