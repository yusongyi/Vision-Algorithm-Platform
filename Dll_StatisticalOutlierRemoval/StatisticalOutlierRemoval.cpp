#include "pch.h"
#include "StatisticalOutlierRemoval.h" 
#include <iostream> 
#define _CRT_SECURE_NO_WARNINGS
#define  Defultfilename  "StatisticalOutlierRemoval"
typedef pcl::PointXYZ PointT;
using namespace pcl;
using namespace std;

/*

ͳ���˲���

	params[0]:
		//���ڵ�ĸ���
	params[1]:
		//��׼�����
*/
int StatisticalOutlierRemoval(pcl::PointCloud<PointT>::Ptr input, pcl::PointCloud<PointT>::Ptr out, float* params)
{
	std::cout << "original cloud size : " << input->size() << std::endl;
	std::cout << "param1: " << params[0] << std::endl;
	std::cout << "param2: " << params[1] << std::endl;

	//������ȥ�� 
	pcl::StatisticalOutlierRemoval<PointXYZ> sor;
	sor.setInputCloud(input);
	sor.setMeanK(params[0]);
	sor.setStddevMulThresh(params[1]);
	sor.filter(*out);
	std::cout << "StatisticalOutlierRemoval size :" << out->size() << std::endl;

	return 0;
}