#pragma once
#include "AlgoNode.h"
#include <memory>
#include <mutex>
#include <condition_variable>

class CloudQueue
{
	typedef pcl::PointCloud<PointT>::Ptr ElemType;//������������
	//**********ѭ�����п�ʼ**********
	#define MAXSIZE 10000//��ʼ����
	typedef struct {
		ElemType data[MAXSIZE];
		int front;//ͷָ��
		int rear;//βָ�룬���зǿ�ʱ��ָ���βԪ���¸�λ��
	}SQueue;
	std::condition_variable not_empty_cv_;
	mutable std::mutex mutex_;
public:
	SQueue Q;
	bool running = false;
	void InitQueue();
	bool QueueFull();
	bool QueueEmpty();
	void EnQueue(ElemType x);
	ElemType DeQueue();
	int QueueSize();
	void start();//���ɵ���
};
static CloudQueue cloudQueue;