#include "pch.h"
#include "Zuobian.h" 
#include <iostream> 
#define _CRT_SECURE_NO_WARNINGS
#define  Defultfilename  "Zuobian"
typedef pcl::PointXYZ PointT;
using namespace pcl;
using namespace std;
#define EPSILON 0.001 //���ݾ�����Ҫ

/*

ֱͨ�˲���

	params[0]:
		//0 ��x����в���
		//1 ��y����в���
		//2 ��z����в���
	params[1]:
		//����ֱͨ�˲���������Χ-min
	params[2]:
		//����ֱͨ�˲���������Χ-max
	params[3]:
		//1��ʾ������Χ�ڣ�0��ʾ������Χ��

*/
int MyPassThrough(pcl::PointCloud<PointT>::Ptr input, pcl::PointCloud<PointT>::Ptr out, float* params)
{
	std::cout << "original cloud size : " << input->size() << std::endl;
	std::cout << "param1: " << params[0] << std::endl;
	std::cout << "param2: " << params[1] << std::endl;
	std::cout << "param3: " << params[2] << std::endl;

	//ֱͨ�˲����Ե��ƽ��д���
	pcl::PassThrough<PointXYZ> passthrough;
	passthrough.setInputCloud(input);//�������

	if (fabs(params[0] - 0) < EPSILON) {
		passthrough.setFilterFieldName("x");//��һ������Ϊ0 ��x����в���
	}
	else if (fabs(params[0] - 1) < EPSILON) {
		passthrough.setFilterFieldName("y");//��һ������Ϊ1 ��y����в���
	}
	else if (fabs(params[0] - 2) < EPSILON) {
		passthrough.setFilterFieldName("z");//��һ������Ϊ2 ��z����в���
	}

	passthrough.setFilterLimits(params[1], params[2]);//����ֱͨ�˲���������Χ

	passthrough.setNegative(fabs(params[3] - 0) < EPSILON);//true��ʾ������Χ�ڣ�false��ʾ������Χ��

	passthrough.filter(*out);//ִ���˲�

	std::cout << "PassThrough size :" << out->size() << std::endl;

	return 0;
}

 
/*

ֱͨ�˲���

	params[0]:
		//0 ��x����в���
		//1 ��y����в���
		//2 ��z����в���
	params[1]:
		//����ֱͨ�˲���������Χ-min
	params[2]:
		//����ֱͨ�˲���������Χ-max
	params[3]:
		//1��ʾ������Χ�ڣ�0��ʾ������Χ��

*/
int Zuobian(pcl::PointCloud<PointT>::Ptr cloud, NodeInput** inputs, NodeOutput** outputs, float* params1)
{
  
	PointCloud<PointXYZ>::Ptr through_out1(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr through_out2(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr through_out3(new PointCloud<PointXYZ>);

	//�˲�
	pcl::VoxelGrid<PointT> grid;
	const float leaf = 0.05f;
	grid.setLeafSize(leaf, leaf, leaf);
	grid.setInputCloud(cloud);
	pcl::PointCloud<PointT>::Ptr voxelResult(new pcl::PointCloud<PointT>);
	grid.filter(*voxelResult);

	//�е��ϱ�
	float params[4] = { 1,-0.5,-0.1,1 };
	MyPassThrough(voxelResult, through_out1, params);

	//�е�����
	float params2[4] = { 0,-0.5,0.3,1 };
	MyPassThrough(through_out1, through_out2, params2);


	//�ҳ��߽�
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

	//�е��ұ�
	float params3[4] = { 0,-0.5,0,1 };
	MyPassThrough(cloud_boundary, through_out3, params3);



	//����һ��ģ�Ͳ����������ڼ�¼���
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers��ʾ��������̵ĵ� ��¼���ǵ��Ƶ����
	pcl::SACSegmentation<pcl::PointXYZ> seg;     // ����һ���ָ���
	seg.setOptimizeCoefficients(true);      // Optional��������ÿ���ѡ�����ƽ��չʾ�ĵ��Ƿָ���ĵ㻹�Ƿָ�ʣ�µĵ㡣
	seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-����Ŀ�꼸����״
	seg.setMethodType(pcl::SAC_RANSAC);     //�ָ�������������
	seg.setDistanceThreshold(0.01);         //����������̷�Χ��Ҳ������ֵ
	seg.setInputCloud(through_out3);               //�������
	seg.segment(*inliers, *coefficients);   //�ָ���ƣ����ƽ��ͷ����� 

	/*�Ӽ���ȡ*/
	// ֱ�ߵ��ȡ
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