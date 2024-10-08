#include "pch.h"
#include "passthrough.h" 
#include <iostream> 
#define _CRT_SECURE_NO_WARNINGS
#define  Defultfilename  "PassThrough"
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
int PassThrough(pcl::PointCloud<PointT>::Ptr input, pcl::PointCloud<PointT>::Ptr out, float* params)
{
	std::cout << "original cloud size : " << input->size() << std::endl;
	std::cout << "param1: " << params[0] << std::endl;
	std::cout << "param2: " << params[1] << std::endl;
	std::cout << "param3: " << params[2] << std::endl;

	//直通滤波器对点云进行处理。
	pcl::PassThrough<PointXYZ> passthrough;
	passthrough.setInputCloud(input);//输入点云

	if (fabs(params[0] - 0) < EPSILON) {
		passthrough.setFilterFieldName("x");//第一个参数为0 对x轴进行操作
	}else if (fabs(params[0] - 1) < EPSILON) {
		passthrough.setFilterFieldName("y");//第一个参数为1 对y轴进行操作
	}else if(fabs(params[0] - 2) < EPSILON) {
		passthrough.setFilterFieldName("z");//第一个参数为2 对z轴进行操作
	}
	
	passthrough.setFilterLimits(params[1], params[2]);//设置直通滤波器操作范围

	passthrough.setNegative(fabs(params[3] - 0) < EPSILON);//true表示保留范围内，false表示保留范围外

	passthrough.filter(*out);//执行滤波
 
	std::cout << "PassThrough size :" << out->size() << std::endl;

	return 0;
}