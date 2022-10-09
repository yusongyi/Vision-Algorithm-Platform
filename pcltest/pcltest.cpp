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

#include <time.h>
using namespace pcl;
using namespace std;

typedef pcl::PointXYZ PointT;
 

#define EPSILON 0.001 //根据精度需要



class fitLineRansac
{

public:
	vector<vector<float>> ransac_line(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float dist, int iterate)
	{
		/***
		 *		dist: 点到直线距离小于dist 即当前点在直线上
		 *      iterate: 迭代次数
		***/
		int allPts = cloud->points.size();
		vector<int> cur_ptsIdx(allPts);			//提取当前直线后，不在当前直线上的其他点索引
		for (int i = 0; i < allPts; i++)
			cur_ptsIdx[i] = i;

		int r = 0;
		vector<int> line_ptsIdx, all_ptsIdx(allPts);
		vector<vector<float>> lines;			//所有直线参数 [0]: k, [1]: b
		vector<float> cur_line(2);
		Eigen::Vector3f line_model, best_lineModel;
		while (1)
		{
			int line_pts = 0, tmp_pts;
			if (r >= 2) iterate = iterate / 3;
			if (cur_ptsIdx.size() < 10 && cur_ptsIdx.size() > 3) iterate = 4;
			for (int i = 0; i < iterate; i++) {
				line_model = leastSquare(cloud, cur_ptsIdx, dist);
				tmp_pts = line_model[2] / 1;
				if (tmp_pts > line_pts) {
					line_pts = tmp_pts;
					best_lineModel = line_model;
					line_ptsIdx = tmp_ptsIdx;
				}
				tmp_ptsIdx.clear();
			}

			cur_line[0] = best_lineModel[0]; cur_line[1] = best_lineModel[1];
			lines.push_back(cur_line);
			cout << "第　" << r++ << "　次循环,直线参数:  " << best_lineModel << endl;
			cout<<"所有点的个数:  "<<cur_ptsIdx.size()<<endl;
			cout<<"当前直线上的点数："<<line_ptsIdx.size()<<endl;

			//得到剩余点的索引
			for (int i = 0; i < line_ptsIdx.size(); i++)
				all_ptsIdx[line_ptsIdx[i]] = 1;
			cur_ptsIdx.clear();
			for (int j = 0; j < allPts; j++)
				if (!all_ptsIdx[j]) cur_ptsIdx.push_back(j);

			if (cur_ptsIdx.size() < 5) {
				cout << "点数剩余过少......." << endl;
				break;
			}
		}
		view(cloud, lines);
		return lines;
	}

private:
	vector<int> tmp_ptsIdx;			//当前直线上的点数
	Eigen::Vector3f leastSquare(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vector<int> pIdx, float dist)
	{
		//求解给定若干点的直线方程
		float a = 0, B = 0, c = 0, d = 0;          //a: x之和　b: y之和  c: x平方和  d: x*y之和  e: 样本数量
		int s = pIdx.size();
		vector<int> cur_ptsIdx = rangedRand(0, s, 4);					//４：每次选４点用最小二乘拟合直线
		int e = cur_ptsIdx.size();
		for (int i = 0; i < e; i++) {
			a += cloud->points[pIdx[cur_ptsIdx[i]]].x;
			B += cloud->points[pIdx[cur_ptsIdx[i]]].y;
			c += cloud->points[pIdx[cur_ptsIdx[i]]].x * cloud->points[pIdx[cur_ptsIdx[i]]].x;
			d += cloud->points[pIdx[cur_ptsIdx[i]]].x * cloud->points[pIdx[cur_ptsIdx[i]]].y;
		}
		float k, b;
		float tmp = e * c - a * a;
		if (abs(tmp) > 0.0005) {
			b = (c*B - a * d) / tmp;
			k = (e*d - a * B) / tmp;
		}
		else {
			k = 1; b = 0;
		}

		//求每一个点到直线的距离，小于dist, 即在直线上
		int line_pnum = 0;
		for (int i = 0; i < s; i++) {
			float d, numerator, denominator;             //分子分母        点到直线的距离　d = |kx - y + b| / sqrt(k^2 + 1)
			numerator = abs(k*cloud->points[pIdx[i]].x - cloud->points[pIdx[i]].y + b);
			denominator = sqrt(k*k + 1);
			d = numerator / denominator;
			if (d < dist) {
				line_pnum++;
				tmp_ptsIdx.push_back(pIdx[i]);
			}
		}
		Eigen::Vector3f line_model;
		line_model[0] = k; line_model[1] = b; line_model[2] = line_pnum;
		return line_model;
	}

	vector<int> rangedRand(int range_begin, int range_size, int n)
	{
		int i; vector<int> indices;
		// srand((unsigned)time(NULL));           //生成随机数种子
		for (i = 0; i < n; i++)
		{
			int u = rand() % range_size + range_begin; //生成[range_begin, range_begin+range_size]内的随机数
			// cout<<i<<": "<<u<<endl;
			indices.push_back(u);
		}
		return indices;
	}

	void view(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vector<vector<float>> lines)
	{
		cout << "共找到   " << lines.size() << "  条直线!!!!!!!!" << endl;
		pcl::visualization::PCLVisualizer viewer("Viewer");
		viewer.setBackgroundColor(0.5, 0.5, 0.5, 0);
		viewer.addPointCloud(cloud, "pc"); 
		for (int i = 0; i < lines.size(); i++) {
			pcl::PointXYZ p1(2, 2 * lines[i][0] + lines[i][1], 0);
			pcl::PointXYZ p2(-5, -5 * lines[i][0] + lines[i][1], 0);
			cout << "直线     " << i + 1 << "   上两点坐标： " << p1 << ",  " << p2 << endl;
			viewer.addLine(p1, p2, 2.4 - 0.4 * i, 0, 0, "line" + to_string(i), 0);
		}
		while (!viewer.wasStopped()) {
			viewer.spinOnce();
		}
	}

};


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

//显示
void visualization_point( PointCloud<PointXYZ>::Ptr &raw_point,PointCloud<PointXYZ>::Ptr &sor_cloud
	//,PointCloud<PointXYZ>::Ptr &voxel, PointCloud<PointXYZ>::Ptr &uniform
) {
	visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer("3d viewer"));
	int v1(0), v2(0), v3(0), v4(0);
	viewer->createViewPort(0.0, 0.0, 0.25, 1.0, v1);   
	viewer->addPointCloud(raw_point, "cloud1", v1);


	viewer->createViewPort(0.25, 0.0, 0.5, 1.0, v2);
	viewer->addPointCloud(sor_cloud, "cloud2", v2);


	//viewer->createViewPort(0.5, 0.0, 0.75, 1.0, v3);
	//viewer->addPointCloud(voxel, "cloud3", v3);

	//viewer->createViewPort(0.75, 0.0, 1, 1.0, v3);
	//viewer->addPointCloud(uniform, "cloud4", v3);

	viewer->spin(); 
}

int _tmain(int argc, _TCHAR* argv[])
{ 

	 pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
     pcl::io::loadPCDFile ("table.pcd", *cloud);
	 //pcl::io::loadPLYFile ("E:\\CODE\\three.js\\examples\\models\\ply\\binary\\Lucy100k.ply", *cloud);
	 std::cout << "original cloud size : " << cloud->size() << std::endl;
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
	 boundEst.setAngleThreshold(M_PI*3/4);
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
	 float params3[4] = { 1,-0.5,0,1 };
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
	 for (int i = 0; i < inliers->indices.size(); ++i) {
		 c_plane->points.push_back(through_out3->points.at(inliers->indices[i]));
	 }
 
	 //coefficients->values[2] = 0;
	   
	 visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer("3d viewer"));
	 viewer->addPointCloud(cloud_boundary);

	 pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(c_plane, 255, 0, 0);
	 viewer->addPointCloud(c_plane, single_color1,"bondary",0);
	 viewer->addCoordinateSystem(1.0);
	 viewer->addLine(*coefficients,"line",0);
	 viewer->spin();

	 

	// clock_t start, finish;
	// start = clock();

	// int j = 0;
	// PointCloud<PointXYZ>::Ptr out(new PointCloud<PointXYZ>);
	// while (j<99)
	// {
	//	 pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;// 创建滤波器    
	//	 outrem.setInputCloud(cloud);              //设置输入点云
	//	 //搜索半径设为0.8，在此半径内点必须要有至少1个邻居时，此点才会被保留
	//	 outrem.setRadiusSearch(0.02);              //设置在0.8半径的范围内找邻近点
	//	 outrem.setMinNeighborsInRadius(1);        //设置查询点的邻近点集数小于1的删除
	//	 outrem.filter(*out);           //执行条件滤波，存储结果到cloud_filtered
	//	 printf("res:%d,%d\r\n", cloud->size(), out->size());
	//	 j++;
	// }

	// finish = clock();

	// double duration = (double)(finish - start) / CLOCKS_PER_SEC;
	// printf("%f seconds\n", duration);


	// //噪声点去除
	// PointCloud<PointXYZ>::Ptr sor_cloud(new PointCloud<PointXYZ>);
	// StatisticalOutlierRemoval<PointXYZ> sor;
	// sor.setInputCloud(cloud);
	// sor.setMeanK(8);
	// sor.setStddevMulThresh(1);
	// sor.filter(*sor_cloud);

	// // 使用体素化网格(VoxelGrid)进行下采样
	// pcl::VoxelGrid<PointT> grid; //创建滤波对象
	// const float leaf = 0.05f;
	// grid.setLeafSize(leaf, leaf, leaf); // 设置体素体积
	// grid.setInputCloud(sor_cloud); // 设置点云
	// pcl::PointCloud<PointT>::Ptr voxelResult(new pcl::PointCloud<PointT>);
	// grid.filter(*voxelResult); // 执行滤波，输出结果
	// std::cout << "voxel downsample size :" << voxelResult->size() << std::endl;
	//  

	// // 使用UniformSampling进行下采样
	///* pcl::UniformSampling<PointT> uniform_sampling;
	// uniform_sampling.setInputCloud(sor_cloud);
	// double radius = 0.05f;
	// uniform_sampling.setRadiusSearch(radius);
	// pcl::PointCloud<PointT>::Ptr uniformResult(new pcl::PointCloud<PointT>);
	// uniform_sampling.filter(*uniformResult);
	// std::cout << "UniformSampling size :" << uniformResult->size() << std::endl;*/


	// pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	// pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	// // Create the segmentation object
	// pcl::SACSegmentation<pcl::PointXYZ> seg;
	// // Optional
	// seg.setOptimizeCoefficients(true);
	// // Mandatory
	// seg.setModelType(pcl::SACMODEL_PLANE);
	// seg.setMethodType(pcl::SAC_RANSAC);
	// seg.setMaxIterations(1000);
	// seg.setDistanceThreshold(0.01);

	// // Create the filtering object
	// pcl::ExtractIndices<pcl::PointXYZ> extract;

	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);


	// int i = 0, nr_points = (int)voxelResult->points.size();
	// // While 30% of the original cloud is still there
	// while (voxelResult->points.size() > 0.3 * nr_points)
	// {
	//	 // Segment the largest planar component from the remaining cloud
	//	 seg.setInputCloud(voxelResult);
	//	 seg.segment(*inliers, *coefficients);
	//	 if (inliers->indices.size() == 0)
	//	 {
	//		 std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
	//		 break;
	//	 }

	//	 // Extract the inliers
	//	 extract.setInputCloud(voxelResult);
	//	 extract.setIndices(inliers);
	//	 extract.setNegative(false);
	//	 extract.filter(*cloud_p);
	//	 std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
	//	  
	//	 extract.setNegative(true);
	//	 extract.filter(*cloud_f);
	//	   
	//	 voxelResult.swap(cloud_f);
	//	 i++; 
	//	
	// } 
	//  
	// visualization_point(cloud,sor_cloud, voxelResult, cloud_p);
 
	 system("pause");
    return 0;
}

