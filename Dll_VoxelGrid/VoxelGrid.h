#pragma once


#pragma   push_macro("min")  
#pragma   push_macro("max")  
#undef   min  
#undef   max  
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>   
#include <pcl/filters/voxel_grid.h>  
#pragma   pop_macro("min")  
#pragma   pop_macro("max")
#define _CRT_SECURE_NO_WARNINGS
typedef pcl::PointXYZ PointT;
extern "C" __declspec(dllexport) int VoxelGrid(pcl::PointCloud<PointT>::Ptr input, pcl::PointCloud<PointT>::Ptr out, float* params);