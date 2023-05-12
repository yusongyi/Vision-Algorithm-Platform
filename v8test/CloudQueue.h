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
	typedef struct {
		int data[MAXSIZE];
		int front;//头指针
		int rear;//尾指针，队列非空时，指向队尾元素下个位置
	}CQueue;
	std::condition_variable not_empty_cv_;
	mutable std::mutex mutex_;
public:
	SQueue Q;
	CQueue CQ;
	bool running = false;
	void InitQueue();
	bool QueueFull();
	bool QueueEmpty();
	void EnQueue(CloudObj x);
	CloudObj DeQueue();
	int QueueSize();
	void start();//生成点云
	int cloudId;
};
static CloudQueue cloudQueue;