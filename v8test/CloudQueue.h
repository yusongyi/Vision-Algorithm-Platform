#pragma once
#include "AlgoNode.h"
#include <memory>
#include <mutex>
#include <condition_variable>

class CloudQueue
{
	typedef pcl::PointCloud<PointT>::Ptr ElemType;//������������

public:
	typedef struct {
		int cloudId;//����Id
		ElemType cloudList;//��������
		int index;//ѭ������
	} CloudObj;

	//**********ѭ�����п�ʼ**********
	#define MAXSIZE 100000//��ʼ����
	typedef struct {
		CloudObj data[MAXSIZE];
		int front;//ͷָ��
		int rear;//βָ�룬���зǿ�ʱ��ָ���βԪ���¸�λ��
	}SQueue;
	typedef struct {
		int data[MAXSIZE];
		int front;//ͷָ��
		int rear;//βָ�룬���зǿ�ʱ��ָ���βԪ���¸�λ��
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
	void start();//���ɵ���
	int cloudId;
};
static CloudQueue cloudQueue;