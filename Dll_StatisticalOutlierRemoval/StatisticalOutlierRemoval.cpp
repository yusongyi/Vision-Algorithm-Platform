#include "pch.h"
#include "StatisticalOutlierRemoval.h" 
#include <iostream> 
#define _CRT_SECURE_NO_WARNINGS
#define  Defultfilename  "StatisticalOutlierRemoval"
typedef pcl::PointXYZ PointT;
using namespace pcl;
using namespace std;

/*

统计滤波器

	params[0]:
		//近邻点的个数
	params[1]:
		//标准差乘数
*/
int StatisticalOutlierRemoval(pcl::PointCloud<PointT>::Ptr input, pcl::PointCloud<PointT>::Ptr out, float* params)
{
	std::cout << "original cloud size : " << input->size() << std::endl;
	std::cout << "param1: " << params[0] << std::endl;
	std::cout << "param2: " << params[1] << std::endl;

	//噪声点去除 
	pcl::StatisticalOutlierRemoval<PointXYZ> sor;
	sor.setInputCloud(input);
	sor.setMeanK(params[0]);
	sor.setStddevMulThresh(params[1]);
	sor.filter(*out);
	std::cout << "StatisticalOutlierRemoval size :" << out->size() << std::endl;

	return 0;
}