#pragma once


#pragma   push_macro("min")  
#pragma   push_macro("max")  
#undef   min  
#undef   max 
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h> 
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/radius_outlier_removal.h>  
#include <pcl/filters/statistical_outlier_removal.h> 
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>  
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

#include <pcl/filters/extract_indices.h> 
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "NodeInput.h"
#include "NodeOutput.h"
#pragma   pop_macro("min")  
#pragma   pop_macro("max")
#define _CRT_SECURE_NO_WARNINGS
typedef pcl::PointXYZ PointT;
extern "C" __declspec(dllexport) int LineFit(pcl::PointCloud<PointT>::Ptr input, NodeInput** inputs, NodeOutput** outputs, float* params);