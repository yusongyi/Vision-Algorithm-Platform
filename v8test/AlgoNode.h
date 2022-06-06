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
	string id;
	int paramSize;
	RUN_FUN runAddr;
	long preAddr; 
	float *params;
	pcl::PointCloud<PointT>::Ptr input;
	pcl::PointCloud<PointT>::Ptr out;
	string outPath;
};

