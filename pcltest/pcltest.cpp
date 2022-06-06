// pcltest.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h" 
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h> 
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/statistical_outlier_removal.h> 
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
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
     //pcl::io::loadPCDFile ("C:\\Users\\YuSongyi\\Desktop\\table.pcd", *cloud);
	 pcl::io::loadPLYFile ("E:\\CODE\\three.js\\examples\\models\\ply\\binary\\Lucy100k.ply", *cloud);
	 std::cout << "original cloud size : " << cloud->size() << std::endl;


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
	 pcl::UniformSampling<PointT> uniform_sampling;
	 uniform_sampling.setInputCloud(sor_cloud);
	 double radius = 0.05f;
	 uniform_sampling.setRadiusSearch(radius);
	 pcl::PointCloud<PointT>::Ptr uniformResult(new pcl::PointCloud<PointT>);
	 uniform_sampling.filter(*uniformResult);
	 std::cout << "UniformSampling size :" << uniformResult->size() << std::endl;
	  
	 visualization_point(cloud,sor_cloud, voxelResult, uniformResult);
 
	 system("pause");
    return 0;
}

