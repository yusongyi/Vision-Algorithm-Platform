#pragma once 
#include "string" 
#include <pcl/io/pcd_io.h> 
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>  

using namespace std;

typedef pcl::PointXYZ PointT;
typedef int (*RUN_FUN)(pcl::PointCloud<PointT>::Ptr input, pcl::PointCloud<PointT>::Ptr out, float* params);

class AlgoNode
{
public:
	AlgoNode(void);
	virtual ~AlgoNode(void); 
	string name;
	string chName;
	string id;
	RUN_FUN runAddr;
	long preAddr; 


	int paramSize;
	float *params;

	/*
		-1: û�з�֧
		0: if ����
		1: if С��
		2: if ����
	*/
	int conditionType = -1;
	int conditionParamSize;
	float *conditionParams;
	int conditionCount = 0;
	bool conditionNext = true;



	pcl::PointCloud<PointT>::Ptr input;
	pcl::PointCloud<PointT>::Ptr out;
	string outPath;
	string potreePath;
};

