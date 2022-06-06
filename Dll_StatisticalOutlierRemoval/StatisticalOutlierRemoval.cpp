#include "pch.h"
#include "StatisticalOutlierRemoval.h" 
#include <iostream> 
#define _CRT_SECURE_NO_WARNINGS
#define  Defultfilename  "statisticalOutlierRemoval"
typedef pcl::PointXYZ PointT;
using namespace pcl;
using namespace std;

int statisticalOutlierRemoval(pcl::PointCloud<PointT>::Ptr input, pcl::PointCloud<PointT>::Ptr out, float* params)
{
	std::cout << "original cloud size : " << input->size() << std::endl;
	std::cout << "param1: " << params[0] << std::endl;
	std::cout << "param2: " << params[1] << std::endl;

	//ÔëÉùµãÈ¥³ý 
	StatisticalOutlierRemoval<PointXYZ> sor;
	sor.setInputCloud(input);
	sor.setMeanK(*params);
	sor.setStddevMulThresh(*(params++));
	sor.filter(*out);
	std::cout << "statisticalOutlierRemoval size :" << out->size() << std::endl;

	return 0;
}