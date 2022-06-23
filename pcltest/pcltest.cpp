// pcltest.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h" 
#include <iostream>
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

#include <pcl/filters/extract_indices.h> 
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <time.h>
using namespace pcl;
using namespace std;

typedef pcl::PointXYZ PointT;
 

//显示
void visualization_point( PointCloud<PointXYZ>::Ptr &raw_point,PointCloud<PointXYZ>::Ptr &sor_cloud, PointCloud<PointXYZ>::Ptr &voxel, PointCloud<PointXYZ>::Ptr &uniform) {
	visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer("3d viewer"));
	int v1(0), v2(0), v3(0), v4(0);
	viewer->createViewPort(0.0, 0.0, 0.25, 1.0, v1);   
	viewer->addPointCloud(raw_point, "cloud1", v1);


	viewer->createViewPort(0.25, 0.0, 0.5, 1.0, v2);
	viewer->addPointCloud(sor_cloud, "cloud2", v2);


	viewer->createViewPort(0.5, 0.0, 0.75, 1.0, v3);
	viewer->addPointCloud(voxel, "cloud3", v3);

	viewer->createViewPort(0.75, 0.0, 1, 1.0, v3);
	viewer->addPointCloud(uniform, "cloud4", v3);

	viewer->spin(); 
}

int _tmain(int argc, _TCHAR* argv[])
{ 

	 pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
     pcl::io::loadPCDFile ("table.pcd", *cloud);
	 //pcl::io::loadPLYFile ("E:\\CODE\\three.js\\examples\\models\\ply\\binary\\Lucy100k.ply", *cloud);
	 std::cout << "original cloud size : " << cloud->size() << std::endl;

	 clock_t start, finish;
	 start = clock();

	 int j = 0;
	 PointCloud<PointXYZ>::Ptr out(new PointCloud<PointXYZ>);
	 while (j<99)
	 {
		 pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;// 创建滤波器    
		 outrem.setInputCloud(cloud);              //设置输入点云
		 //搜索半径设为0.8，在此半径内点必须要有至少1个邻居时，此点才会被保留
		 outrem.setRadiusSearch(0.02);              //设置在0.8半径的范围内找邻近点
		 outrem.setMinNeighborsInRadius(1);        //设置查询点的邻近点集数小于1的删除
		 outrem.filter(*out);           //执行条件滤波，存储结果到cloud_filtered
		 printf("res:%d,%d\r\n", cloud->size(), out->size());
		 j++;
	 }

	 finish = clock();

	 double duration = (double)(finish - start) / CLOCKS_PER_SEC;
	 printf("%f seconds\n", duration);


	 //噪声点去除
	 PointCloud<PointXYZ>::Ptr sor_cloud(new PointCloud<PointXYZ>);
	 StatisticalOutlierRemoval<PointXYZ> sor;
	 sor.setInputCloud(cloud);
	 sor.setMeanK(8);
	 sor.setStddevMulThresh(1);
	 sor.filter(*sor_cloud);

	 // 使用体素化网格(VoxelGrid)进行下采样
	 pcl::VoxelGrid<PointT> grid; //创建滤波对象
	 const float leaf = 0.05f;
	 grid.setLeafSize(leaf, leaf, leaf); // 设置体素体积
	 grid.setInputCloud(sor_cloud); // 设置点云
	 pcl::PointCloud<PointT>::Ptr voxelResult(new pcl::PointCloud<PointT>);
	 grid.filter(*voxelResult); // 执行滤波，输出结果
	 std::cout << "voxel downsample size :" << voxelResult->size() << std::endl;
	  

	 // 使用UniformSampling进行下采样
	/* pcl::UniformSampling<PointT> uniform_sampling;
	 uniform_sampling.setInputCloud(sor_cloud);
	 double radius = 0.05f;
	 uniform_sampling.setRadiusSearch(radius);
	 pcl::PointCloud<PointT>::Ptr uniformResult(new pcl::PointCloud<PointT>);
	 uniform_sampling.filter(*uniformResult);
	 std::cout << "UniformSampling size :" << uniformResult->size() << std::endl;*/


	 pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	 pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	 // Create the segmentation object
	 pcl::SACSegmentation<pcl::PointXYZ> seg;
	 // Optional
	 seg.setOptimizeCoefficients(true);
	 // Mandatory
	 seg.setModelType(pcl::SACMODEL_PLANE);
	 seg.setMethodType(pcl::SAC_RANSAC);
	 seg.setMaxIterations(1000);
	 seg.setDistanceThreshold(0.01);

	 // Create the filtering object
	 pcl::ExtractIndices<pcl::PointXYZ> extract;

	 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
	 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);


	 int i = 0, nr_points = (int)voxelResult->points.size();
	 // While 30% of the original cloud is still there
	 while (voxelResult->points.size() > 0.3 * nr_points)
	 {
		 // Segment the largest planar component from the remaining cloud
		 seg.setInputCloud(voxelResult);
		 seg.segment(*inliers, *coefficients);
		 if (inliers->indices.size() == 0)
		 {
			 std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			 break;
		 }

		 // Extract the inliers
		 extract.setInputCloud(voxelResult);
		 extract.setIndices(inliers);
		 extract.setNegative(false);
		 extract.filter(*cloud_p);
		 std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
		  
		 extract.setNegative(true);
		 extract.filter(*cloud_f);
		   
		 voxelResult.swap(cloud_f);
		 i++; 
		
	 } 
	  
	 visualization_point(cloud,sor_cloud, voxelResult, cloud_p);
 
	 system("pause");
    return 0;
}

