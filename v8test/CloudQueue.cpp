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

void CloudQueue::EnQueue(CloudQueue::CloudObj x)
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

CloudQueue::CloudObj CloudQueue::DeQueue()
{
	if (QueueEmpty()) {
		printf("�����ѿ�! ����ʧ��!\n");
		return{};
	} 
	std::lock_guard<std::mutex> lk(cloudQueue.mutex_);
	cloudQueue.Q.front = (cloudQueue.Q.front + 1) % MAXSIZE;//��ͷָ����ƣ��絽�����ת��ͷ��
	CloudObj x = cloudQueue.Q.data[cloudQueue.Q.front];
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
	int index = 0;
	int cloudId = 0;
	while (CloudQueue::running) {
		pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
		cloud->points.resize(100);  
			for (int j = 0; j < 100; ++j) {
				float y = 0.1 * rand() / (RAND_MAX + 1.0f);
				float x = 3 + index / 10.0 + y;
				float z = 3 + j / 10.0 + y;
				y = 3 +  rand() / (RAND_MAX + 1.0f);
				PointT p;
				p.x = x;
				p.y = y;
				p.z = z;
				cloud->points[j] = p; 
			} 
		try {
			//���ṹ�帳ֵ
			CloudQueue::CloudObj cloudObj = { cloudId, cloud, index};
			//��ӵ�����
			EnQueue(cloudObj);
		}
		catch (exception e) {
			CloudQueue::running = false;
		}
		//�ٵ�1����������
		//Sleep(100);
		++index;
		if (index == 100) {
			std::lock_guard<std::mutex> lock(cloudQueue.mutex_);
			++cloudId;
			cout << "setclouudId = " << cloudId << endl;
			index = 0;
		}
	}
}


