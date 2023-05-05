#pragma once
#include "AlgoNode.h"

class CloudQueue
{
	typedef pcl::PointCloud<PointT>::Ptr ElemType;//定义数据类型
	//**********循环队列开始**********
	#define MAXSIZE 10000//初始容量
	typedef struct {
		ElemType data[MAXSIZE];
		int front;//头指针
		int rear;//尾指针，队列非空时，指向队尾元素下个位置
	}SQueue;

public:
	SQueue Q;
	bool pointFlag = false;
	void InitQueue();
	bool QueueFull();
	bool QueueEmpty();
	void EnQueue(ElemType x);
	ElemType DeQueue();
	int QueueSize();
	void start();//生成点云
};
static CloudQueue cloudQueue;