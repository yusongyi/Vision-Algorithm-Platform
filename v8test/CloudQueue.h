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
public:
	SQueue Q;  
	void InitQueue();
	bool QueueFull();
	bool QueueEmpty();
	void EnQueue(CloudObj x); 
	CloudObj DeQueue();
	pcl::PointCloud<PointT>::Ptr CloudQueue::getCloud();
	int QueueSize();
	void start();//���ɵ��� 

	static CloudQueue& Instance();
};