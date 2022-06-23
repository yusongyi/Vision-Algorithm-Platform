#include "pch.h"
#include "ExtractIndices.h" 
#include <iostream> 
#define _CRT_SECURE_NO_WARNINGS
#define  Defultfilename  "ExtractIndices"
typedef pcl::PointXYZ PointT;
using namespace pcl;
using namespace std;
#define EPSILON 0.001 //���ݾ�����Ҫ

/*

�뾶�˲���

	params[0]:
		//�����뾶
	params[1]:
		//�ڽ��㼯�� 

*/
int ExtractIndices(pcl::PointCloud<PointT>::Ptr input, pcl::PointCloud<PointT>::Ptr out, float* params)
{
	std::cout << "original cloud size : " << input->size() << std::endl;
	std::cout << "param1: " << params[0] << std::endl;
	std::cout << "param2: " << params[1] << std::endl; 


	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;// �����˲���    
	outrem.setInputCloud(input);              //�����������
	//�����뾶��Ϊ0.8���ڴ˰뾶�ڵ����Ҫ������1���ھ�ʱ���˵�Żᱻ����
	outrem.setRadiusSearch(params[0]);              //������0.8�뾶�ķ�Χ�����ڽ���
	outrem.setMinNeighborsInRadius(params[1]);        //���ò�ѯ����ڽ��㼯��С��1��ɾ��
	outrem.filter(*out);           //ִ�������˲����洢�����cloud_filtered

 
	std::cout << "RadiusOutlierRemoval size :" << out->size() << std::endl;

	return 0;
}