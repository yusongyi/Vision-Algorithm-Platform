#include "stdafx.h" 
#include "CloudQueue.h"

void CloudQueue::InitQueue()
{
	cloudQueue.Q.front = cloudQueue.Q.rear = 0;
	//���ж����
}

bool CloudQueue::QueueFull()
{ 
	std::lock_guard<std::mutex> lock(cloudQueue.mutex_);
	return (cloudQueue.Q.rear + 1) % MAXSIZE == cloudQueue.Q.front;//��һȡģ
}

bool CloudQueue::QueueEmpty()
{ 
	std::lock_guard<std::mutex> lock(cloudQueue.mutex_);
	return cloudQueue.Q.front == cloudQueue.Q.rear;//������Ϊ��ʱfront ��rear���
}

void CloudQueue::EnQueue(CloudQueue::ElemType x)
{
	if (QueueFull())
		//�Ƿ�Ϊ�ն���
	{
		return;
	} 
	std::lock_guard<std::mutex> lk(cloudQueue.mutex_);
	cloudQueue.Q.rear = (cloudQueue.Q.rear + 1) % MAXSIZE;//β��ָ����ƣ��絽���ת��ͷ��
	cloudQueue.Q.data[cloudQueue.Q.rear] = x;//�����β
}

CloudQueue::ElemType CloudQueue::DeQueue()
{
	if (QueueEmpty()) {
		printf("�����ѿ�! ����ʧ��!\n");
		return{};
	} 
	std::lock_guard<std::mutex> lk(cloudQueue.mutex_);
	cloudQueue.Q.front = (cloudQueue.Q.front + 1) % MAXSIZE;//��ͷָ����ƣ��絽�����ת��ͷ��
	ElemType x = cloudQueue.Q.data[cloudQueue.Q.front];
	return x;
}

//��ȡ���г���
int CloudQueue::QueueSize()
{
	std::lock_guard<std::mutex> lock(cloudQueue.mutex_);
	return cloudQueue.Q.rear >= cloudQueue.Q.front ? cloudQueue.Q.rear - cloudQueue.Q.front : cloudQueue.Q.rear - cloudQueue.Q.front + MAXSIZE;
}


//��������������ݼ�
void CloudQueue::start()
{
	while (CloudQueue::running) {
		pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
		for (int i = 0; i < 1000; ++i) {
			for (int j = 0; j < 1000; ++j) {
				float y = 0.1 * rand() / (RAND_MAX + 1.0f);
				float x = 3 + i / 10.0 + y;
				float z = 3 + j / 10.0 + y;
				y = 4 +  rand() / (RAND_MAX + 1.0f);
				PointT p;
				p.x = x;
				p.y = z;
				p.z = y;
				cloud->push_back(p);
			}
		}
		try {
			//��ӵ�����
			EnQueue(cloud);
		}
		catch (exception e) {
			CloudQueue::running = false;
		}
		//�ٵ�1����������
		Sleep(100);
	}
}

