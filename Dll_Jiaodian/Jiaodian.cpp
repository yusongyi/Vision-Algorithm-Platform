#include "pch.h"
#include "Jiaodian.h" 
#include <iostream> 
#define _CRT_SECURE_NO_WARNINGS
#define  Defultfilename  "Jiaodian"
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
int Jiaodian(pcl::PointCloud<PointT>::Ptr cloud, NodeInput** inputs, NodeOutput** outputs, float* params)
{
  
 

	pcl::ModelCoefficients::Ptr points(new pcl::ModelCoefficients);
	points->values.resize(3);

	points->values.push_back(0);
	points->values.push_back(0);
	points->values.push_back(0); 

	outputs[0]->coeff = points; 

	return 0;
}