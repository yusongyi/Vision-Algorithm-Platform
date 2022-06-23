#include "pch.h"
#include "VoxelGrid.h" 
#include <iostream> 
#define _CRT_SECURE_NO_WARNINGS
#define  Defultfilename  "VoxelGrid"
typedef pcl::PointXYZ PointT;
using namespace pcl;
using namespace std;
#define EPSILON 0.001 //���ݾ�����Ҫ

/*

�����˲���

	params[0]: ���ش�С
*/
int VoxelGrid(pcl::PointCloud<PointT>::Ptr input, pcl::PointCloud<PointT>::Ptr out, float* params)
{
	std::cout << "original cloud size : " << input->size() << std::endl;
	std::cout << "param1: " << params[0] << std::endl; 
	 
	pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
	voxelgrid.setInputCloud(input);//�����������
	voxelgrid.setLeafSize(params[0], params[0], params[0]);//AABB�����
	voxelgrid.filter(*out);
	 
 
	std::cout << "VoxelGrid size :" << out->size() << std::endl;

	return 0;
}