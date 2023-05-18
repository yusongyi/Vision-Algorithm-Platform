#pragma once
#include "AlgoNode.h"
#include <memory>
#include <mutex>
#include <condition_variable>

class CloudQueue
{
	typedef pcl::PointCloud<PointT>::Ptr ElemType;//定义数据类型

public:
	typedef struct {
		int cloudId;//点云Id
		ElemType cloudList;//点云数据
		int index;//循环次数
	} CloudObj;

	//**********循环队列开始**********
	#define MAXSIZE 100000//初始容量
	typedef struct {
		CloudObj data[MAXSIZE];
		int front;//头指针
		int rear;//尾指针，队列非空时，指向队尾元素下个位置
	}SQueue; 
public:
	SQueue Q;  
	void InitQueue();
	bool QueueFull();
	bool QueueEmpty();
	void EnQueue(CloudObj x); 
	CloudObj DeQueue();
	pcl::PointCloud<PointT>::Ptr CloudQueue::getCloud();
	int QueueSize();
	void start();//生成点云 

	static CloudQueue& Instance();
};