#include "pch.h"
#include "CloudFilter.h" 
#include <iostream> 
#define _CRT_SECURE_NO_WARNINGS
#define  Defultfilename  "CloudFilter"
typedef pcl::PointXYZ PointT;
using namespace pcl;
using namespace std;
#define EPSILON 0.001 //���ݾ�����Ҫ
 

 
/*

ֱͨ�˲���

	params[0]:
		//0 ��x����в���
		//1 ��y����в���
		//2 ��z����в���
	params[1]:
		//����ֱͨ�˲���������Χ-min
	params[2]:
		//����ֱͨ�˲���������Χ-max
	params[3]:
		//1��ʾ������Χ�ڣ�0��ʾ������Χ��

*/
int CloudFilter(pcl::PointCloud<PointT>::Ptr cloud, NodeInput** inputs, NodeOutput** outputs, float* params)
{
  
 

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	  
	for (int i = 0; i < cloud->size(); i++) {
		coefficients->values.push_back(cloud->points[i].x);
		coefficients->values.push_back(cloud->points[i].y);
		coefficients->values.push_back(cloud->points[i].z);
	}

	outputs[0]->coeff = coefficients;

	return 0;
}