#include "pch.h"
#include "CloudBoundaryNode.h" 
#include <iostream> 
#define _CRT_SECURE_NO_WARNINGS
#define  Defultfilename  "CloudBoundaryNode"
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
int CloudBoundaryNode(pcl::PointCloud<PointT>::Ptr cloud, NodeInput** inputs, NodeOutput** outputs, float* params)
{
  
 

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	 

 

	coefficients->values.push_back(8);
	coefficients->values.push_back(4);
	coefficients->values.push_back(8);



	outputs[0]->coeff = coefficients;


	pcl::ModelCoefficients::Ptr coefficients1(new pcl::ModelCoefficients);




	coefficients1->values.push_back(8);
	coefficients1->values.push_back(4);
	coefficients1->values.push_back(3);

	coefficients1->values.push_back(8);
	coefficients1->values.push_back(4);
	coefficients1->values.push_back(13);



	outputs[1]->coeff = coefficients1;

	return 0;
}