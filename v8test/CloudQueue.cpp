#include "stdafx.h" 
#include "CloudQueue.h"

void CloudQueue::InitQueue()
{
	cloudQueue.Q.front = cloudQueue.Q.rear = 0;
	//���ж����
}

bool CloudQueue::QueueFull()
{
	//TODO: ʵ���ж�ѭ�� �����Ƿ�Ϊ���ĵĴ��롣
	return (cloudQueue.Q.rear + 1) % MAXSIZE == cloudQueue.Q.front;//��һȡģ
}

bool CloudQueue::QueueEmpty()
{
	//TODO: ʵ���ж�ѭ�������Ƿ�Ϊ�յĵĴ��롣
	return cloudQueue.Q.front == cloudQueue.Q.rear;//������Ϊ��ʱfront ��rear���
}

void CloudQueue::EnQueue(CloudQueue::ElemType x)
{
	if (QueueFull())
		//�Ƿ�Ϊ�ն���
	{
		return;
	}
	//TODO: ʵ��ѭ��������ӵĴ���
	cloudQueue.Q.rear = (cloudQueue.Q.rear + 1) % MAXSIZE;//β��ָ����ƣ��絽���ת��ͷ��
	cloudQueue.Q.data[cloudQueue.Q.rear] = x;//�����β
}

CloudQueue::ElemType CloudQueue::DeQueue()
{
	if (QueueEmpty()) {
		printf("�����ѿ�! ����ʧ��!\n");
		return{};
	}
	//TODO: ʵ��ѭ�����г��ӵĴ���
	cloudQueue.Q.front = (cloudQueue.Q.front + 1) % MAXSIZE;//��ͷָ����ƣ��絽�����ת��ͷ��
	ElemType x = cloudQueue.Q.data[cloudQueue.Q.front];
	return x;
}

//��ȡ���г���
int CloudQueue::QueueSize()
{
	return cloudQueue.Q.rear >= cloudQueue.Q.front ? cloudQueue.Q.rear - cloudQueue.Q.front : cloudQueue.Q.rear - cloudQueue.Q.front + MAXSIZE;
}


//�����������ݼ�
void CloudQueue::start()
{
	while (cloudQueue.pointFlag) {
		pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
		for (int i = 0; i < 100; ++i) {
			for (int j = 0; j < 100; ++j) {
				float y = 0.1 * rand() / (RAND_MAX + 1.0f);
				float x = 3 + i / 10.0 + y;
				float z = 3 + j / 10.0 + y;
				y = 4 + 00.1 * rand() / (RAND_MAX + 1.0f);
				PointT p;
				p.x = x;
				p.y = y;
				p.z = z;
				cloud->push_back(p);
			}
		}
		try {
			//��ӵ�����
			EnQueue(cloud);
		}
		catch (exception e) {
			cloudQueue.pointFlag = false;
		}
		//�ٵ�1����������
		Sleep(1000);
	}
}

