#include "pch.h"
#include "passthrough.h" 
#include <iostream> 
#define _CRT_SECURE_NO_WARNINGS
#define  Defultfilename  "PassThrough"
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
int PassThrough(pcl::PointCloud<PointT>::Ptr input, pcl::PointCloud<PointT>::Ptr out, float* params)
{
	std::cout << "original cloud size : " << input->size() << std::endl;
	std::cout << "param1: " << params[0] << std::endl;
	std::cout << "param2: " << params[1] << std::endl;
	std::cout << "param3: " << params[2] << std::endl;

	//ֱͨ�˲����Ե��ƽ��д���
	pcl::PassThrough<PointXYZ> passthrough;
	passthrough.setInputCloud(input);//�������

	if (fabs(params[0] - 0) < EPSILON) {
		passthrough.setFilterFieldName("x");//��һ������Ϊ0 ��x����в���
	}else if (fabs(params[0] - 1) < EPSILON) {
		passthrough.setFilterFieldName("y");//��һ������Ϊ1 ��y����в���
	}else if(fabs(params[0] - 2) < EPSILON) {
		passthrough.setFilterFieldName("z");//��һ������Ϊ2 ��z����в���
	}
	
	passthrough.setFilterLimits(params[1], params[2]);//����ֱͨ�˲���������Χ

	passthrough.setNegative(fabs(params[3] - 0) < EPSILON);//true��ʾ������Χ�ڣ�false��ʾ������Χ��

	passthrough.filter(*out);//ִ���˲�
 
	std::cout << "PassThrough size :" << out->size() << std::endl;

	return 0;
}