#pragma once


#pragma   push_macro("min")  
#pragma   push_macro("max")  
#undef   min  
#undef   max 
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>   
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/statistical_outlier_removal.h> 
#include <pcl/kdtree/kdtree.h>
#pragma   pop_macro("min")  
#pragma   pop_macro("max")
#define _CRT_SECURE_NO_WARNINGS
typedef pcl::PointXYZ PointT;
extern "C" __declspec(dllexport) int StatisticalOutlierRemoval(pcl::PointCloud<PointT>::Ptr input, pcl::PointCloud<PointT>::Ptr out, float* params);