#pragma once 
#include "string" 
#include "NodeInput.h"
#include "NodeOutput.h"
#include <pcl/io/pcd_io.h> 
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>  

using namespace std;

//定义点云类型
typedef pcl::PointXYZ PointT;

//定义计算方法，申明输入类型，输出类型
typedef int (*RUN_FUN)(pcl::PointCloud<PointT>::Ptr cloud, NodeInput** inputs, NodeOutput** outputs, float* params);


//算法节点类
class AlgoNode
{
public:
	AlgoNode(void);
	virtual ~AlgoNode(void); 

	//算法ID
	string id;

	//算法名称
	string name;

	//算法中文名
	string chName;

	//算法指针
	RUN_FUN runAddr;

	//前置处理函数（暂无）
	long preAddr;  

	//算法参数数量
	int paramSize;

	//算法参数
	float *params;

	//算法节点输入原始点云
	pcl::PointCloud<PointT>::Ptr input;  

	//输入参数数量
	int inputSize;
	//输入参数数组
	NodeInput** inputs;

	//输出参数数量
	int outputSize;
	//输出参数数组
	NodeOutput** outputs;
};

