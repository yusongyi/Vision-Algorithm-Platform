#include "stdafx.h" 
#include "CloudQueue.h"


pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr * cloudAry;
void CloudQueue::InitQueue()
{

	Q.front = Q.rear = 0;
	cloudAry = new pcl::PointCloud<PointT>::Ptr[1000];
 
	for (int i = 0; i < 1000; ++i) { 
		pcl::PointCloud<PointT>::Ptr cloudLine(new pcl::PointCloud<PointT>);
		for (int j = 0; j < 1000; ++j) {
			float y = 0.1 * rand() / (RAND_MAX + 1.0f);
			float x = 3 + i / 10.0 + y;
			float z = 3 + j / 10.0 + y;
			y = 4 + rand() / (RAND_MAX + 1.0f);
			PointT p;
			p.x = x;
			p.y = y;
			p.z = z;
			cloudLine->push_back(p);
		}
		cloudAry[i] = cloudLine;
		*cloud += *cloudLine; 
	} 
}

bool CloudQueue::QueueFull()
{  
	return (Q.rear + 1) % MAXSIZE == Q.front;//��һȡģ
}

bool CloudQueue::QueueEmpty()
{  
	return Q.front == Q.rear;//������Ϊ��ʱfront ��rear���
}

void CloudQueue::EnQueue(CloudQueue::CloudObj x)
{
	if (QueueFull())
		//�Ƿ�Ϊ�ն���
	{
		return;
	}  
	Q.rear = (Q.rear + 1) % MAXSIZE;//β��ָ����ƣ��絽���ת��ͷ��
	Q.data[Q.rear] = x;//�����β
}

CloudQueue::CloudObj CloudQueue::DeQueue()
{
	if (QueueEmpty()) {
		printf("�����ѿ�! ����ʧ��!\n");
		return{};
	}  
	Q.front = (Q.front + 1) % MAXSIZE;//��ͷָ����ƣ��絽�����ת��ͷ��
	CloudObj x = Q.data[Q.front];
	return x;
}

//��ȡ���г���
int CloudQueue::QueueSize()
{ 
	return Q.rear >= Q.front ? Q.rear - Q.front : Q.rear - Q.front + MAXSIZE;
}

pcl::PointCloud<PointT>::Ptr CloudQueue::getCloud()
{
	 
	return cloud;
}
CloudQueue& CloudQueue::Instance()
{
	static CloudQueue instance;
	return instance;
}


//��������������ݼ�
void CloudQueue::start()
{ 
	int cloudId = 0;
	while (true) {
		if (QueueSize() < 10000) {
			for (int i = 0; i < 1000; i++) {
				//���ṹ�帳ֵ
				CloudQueue::CloudObj cloudObj = { cloudId, cloudAry[i], i };
				//��ӵ�����
				EnQueue(cloudObj);
			} 
			cloudId++; 
		} 
		//�ٵ�1����������
		Sleep(100); 
	}
}


