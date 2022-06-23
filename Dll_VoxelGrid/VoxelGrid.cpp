#include "pch.h"
#include "VoxelGrid.h" 
#include <iostream> 
#define _CRT_SECURE_NO_WARNINGS
#define  Defultfilename  "VoxelGrid"
typedef pcl::PointXYZ PointT;
using namespace pcl;
using namespace std;
#define EPSILON 0.001 //根据精度需要

/*

体素滤波器

	params[0]: 体素大小
*/
int VoxelGrid(pcl::PointCloud<PointT>::Ptr input, pcl::PointCloud<PointT>::Ptr out, float* params)
{
	std::cout << "original cloud size : " << input->size() << std::endl;
	std::cout << "param1: " << params[0] << std::endl; 
	 
	pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
	voxelgrid.setInputCloud(input);//输入点云数据
	voxelgrid.setLeafSize(params[0], params[0], params[0]);//AABB长宽高
	voxelgrid.filter(*out);
	 
 
	std::cout << "VoxelGrid size :" << out->size() << std::endl;

	return 0;
}