#pragma once 
#include "string" 
#include "NodeInput.h"
#include "NodeOutput.h"
#include <pcl/io/pcd_io.h> 
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>  

using namespace std;

//�����������
typedef pcl::PointXYZ PointT;

//������㷽���������������ͣ��������
typedef int (*RUN_FUN)(pcl::PointCloud<PointT>::Ptr cloud, NodeInput** inputs, NodeOutput** outputs, float* params);


//�㷨�ڵ���
class AlgoNode
{
public:
	AlgoNode(void);
	virtual ~AlgoNode(void); 

	//�㷨ID
	string id;

	//�㷨����
	string name;

	//�㷨������
	string chName;

	//�㷨ָ��
	RUN_FUN runAddr;

	//ǰ�ô����������ޣ�
	long preAddr;  

	//�㷨��������
	int paramSize;

	//�㷨����
	float *params;

	//�㷨�ڵ�����ԭʼ����
	pcl::PointCloud<PointT>::Ptr input;  

	//�����������
	int inputSize;
	//�����������
	NodeInput** inputs;

	//�����������
	int outputSize;
	//�����������
	NodeOutput** outputs;
};

