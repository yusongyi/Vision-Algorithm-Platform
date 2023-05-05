#pragma once
#include "AlgoNode.h"

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

public:
	SQueue Q;
	bool pointFlag = false;
	void InitQueue();
	bool QueueFull();
	bool QueueEmpty();
	void EnQueue(ElemType x);
	ElemType DeQueue();
	int QueueSize();
	void start();//���ɵ���
};
static CloudQueue cloudQueue;