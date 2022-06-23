#include "pch.h"
#include "ExtractIndices.h" 
#include <iostream> 
#define _CRT_SECURE_NO_WARNINGS
#define  Defultfilename  "ExtractIndices"
typedef pcl::PointXYZ PointT;
using namespace pcl;
using namespace std;
#define EPSILON 0.001 //根据精度需要

/*

半径滤波器

	params[0]:
		//搜索半径
	params[1]:
		//邻近点集数 

*/
int ExtractIndices(pcl::PointCloud<PointT>::Ptr input, pcl::PointCloud<PointT>::Ptr out, float* params)
{
	std::cout << "original cloud size : " << input->size() << std::endl;
	std::cout << "param1: " << params[0] << std::endl;
	std::cout << "param2: " << params[1] << std::endl; 


	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;// 创建滤波器    
	outrem.setInputCloud(input);              //设置输入点云
	//搜索半径设为0.8，在此半径内点必须要有至少1个邻居时，此点才会被保留
	outrem.setRadiusSearch(params[0]);              //设置在0.8半径的范围内找邻近点
	outrem.setMinNeighborsInRadius(params[1]);        //设置查询点的邻近点集数小于1的删除
	outrem.filter(*out);           //执行条件滤波，存储结果到cloud_filtered

 
	std::cout << "RadiusOutlierRemoval size :" << out->size() << std::endl;

	return 0;
}