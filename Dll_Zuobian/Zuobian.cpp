#include "pch.h"
#include "Zuobian.h" 
#include <iostream> 
#define _CRT_SECURE_NO_WARNINGS
#define  Defultfilename  "Zuobian"
typedef pcl::PointXYZ PointT;
using namespace pcl;
using namespace std;
#define EPSILON 0.001 //根据精度需要

/*

直通滤波器

	params[0]:
		//0 对x轴进行操作
		//1 对y轴进行操作
		//2 对z轴进行操作
	params[1]:
		//设置直通滤波器操作范围-min
	params[2]:
		//设置直通滤波器操作范围-max
	params[3]:
		//1表示保留范围内，0表示保留范围外

*/
int MyPassThrough(pcl::PointCloud<PointT>::Ptr input, pcl::PointCloud<PointT>::Ptr out, float* params)
{
	std::cout << "original cloud size : " << input->size() << std::endl;
	std::cout << "param1: " << params[0] << std::endl;
	std::cout << "param2: " << params[1] << std::endl;
	std::cout << "param3: " << params[2] << std::endl;

	//直通滤波器对点云进行处理。
	pcl::PassThrough<PointXYZ> passthrough;
	passthrough.setInputCloud(input);//输入点云

	if (fabs(params[0] - 0) < EPSILON) {
		passthrough.setFilterFieldName("x");//第一个参数为0 对x轴进行操作
	}
	else if (fabs(params[0] - 1) < EPSILON) {
		passthrough.setFilterFieldName("y");//第一个参数为1 对y轴进行操作
	}
	else if (fabs(params[0] - 2) < EPSILON) {
		passthrough.setFilterFieldName("z");//第一个参数为2 对z轴进行操作
	}

	passthrough.setFilterLimits(params[1], params[2]);//设置直通滤波器操作范围

	passthrough.setNegative(fabs(params[3] - 0) < EPSILON);//true表示保留范围内，false表示保留范围外

	passthrough.filter(*out);//执行滤波

	std::cout << "PassThrough size :" << out->size() << std::endl;

	return 0;
}

 
/*

直通滤波器

	params[0]:
		//0 对x轴进行操作
		//1 对y轴进行操作
		//2 对z轴进行操作
	params[1]:
		//设置直通滤波器操作范围-min
	params[2]:
		//设置直通滤波器操作范围-max
	params[3]:
		//1表示保留范围内，0表示保留范围外

*/
int Zuobian(pcl::PointCloud<PointT>::Ptr cloud, NodeInput** inputs, NodeOutput** outputs, float* params1)
{
  
	PointCloud<PointXYZ>::Ptr through_out1(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr through_out2(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr through_out3(new PointCloud<PointXYZ>);

	//滤波
	pcl::VoxelGrid<PointT> grid;
	const float leaf = 0.05f;
	grid.setLeafSize(leaf, leaf, leaf);
	grid.setInputCloud(cloud);
	pcl::PointCloud<PointT>::Ptr voxelResult(new pcl::PointCloud<PointT>);
	grid.filter(*voxelResult);

	//切掉上边
	float params[4] = { 1,-0.5,-0.1,1 };
	MyPassThrough(voxelResult, through_out1, params);

	//切掉左右
	float params2[4] = { 0,-0.5,0.3,1 };
	MyPassThrough(through_out1, through_out2, params2);


	//找出边界
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	normEst.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(through_out2));
	normEst.setRadiusSearch(0.1);
	normEst.compute(*normals);
	pcl::PointCloud<pcl::Boundary> boundaries;
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst;
	boundEst.setInputCloud(through_out2);
	boundEst.setInputNormals(normals);
	boundEst.setRadiusSearch(0.1);
	boundEst.setAngleThreshold(M_PI *3/4);
	boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
	boundEst.compute(boundaries);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < through_out2->points.size(); i++)
	{

		if (boundaries[i].boundary_point > 0)
		{
			cloud_boundary->push_back(through_out2->points[i]);
		}
	}

	//切掉右边
	float params3[4] = { 0,-0.5,0,1 };
	MyPassThrough(cloud_boundary, through_out3, params3);



	//创建一个模型参数对象，用于记录结果
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers表示误差能容忍的点 记录的是点云的序号
	pcl::SACSegmentation<pcl::PointXYZ> seg;     // 创建一个分割器
	seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
	seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-设置目标几何形状
	seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
	seg.setDistanceThreshold(0.01);         //设置误差容忍范围，也就是阈值
	seg.setInputCloud(through_out3);               //输入点云
	seg.segment(*inliers, *coefficients);   //分割点云，获得平面和法向量 

	/*子集提取*/
	// 直线点获取
	pcl::PointCloud<pcl::PointXYZ>::Ptr c_plane(new pcl::PointCloud<pcl::PointXYZ>);


	pcl::ModelCoefficients::Ptr points(new pcl::ModelCoefficients);
 
	for (int i = 0; i < inliers->indices.size(); ++i) {
		c_plane->points.push_back(through_out3->points.at(inliers->indices[i]));
		points->values.push_back(through_out3->points.at(inliers->indices[i]).x);
		points->values.push_back(through_out3->points.at(inliers->indices[i]).y);
		points->values.push_back(through_out3->points.at(inliers->indices[i]).z);
	}

	//coefficients->values[2] = 0;

	//visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer("3d viewer"));
	//viewer->addPointCloud(cloud_boundary);

	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(c_plane, 255, 0, 0);
	//viewer->addPointCloud(c_plane, single_color1, "bondary", 0);

	//viewer->addLine(*coefficients, "line", 0);
	//viewer->spin();

	outputs[0]->coeff = points;
	outputs[1]->coeff = coefficients;

	return 0;
}