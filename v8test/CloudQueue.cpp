#include "stdafx.h" 
#include "CloudQueue.h"

void CloudQueue::InitQueue()
{
	cloudQueue.Q.front = cloudQueue.Q.rear = 0;
	//将列队清空
}

bool CloudQueue::QueueFull()
{
	//TODO: 实现判断循环 队列是否为满的的代码。
	return (cloudQueue.Q.rear + 1) % MAXSIZE == cloudQueue.Q.front;//加一取模
}

bool CloudQueue::QueueEmpty()
{
	//TODO: 实现判断循环队列是否为空的的代码。
	return cloudQueue.Q.front == cloudQueue.Q.rear;//当队列为空时front 和rear相等
}

void CloudQueue::EnQueue(CloudQueue::ElemType x)
{
	if (QueueFull())
		//是否为空队列
	{
		return;
	}
	//TODO: 实现循环队列入队的代码
	cloudQueue.Q.rear = (cloudQueue.Q.rear + 1) % MAXSIZE;//尾部指针后移，如到最后转到头部
	cloudQueue.Q.data[cloudQueue.Q.rear] = x;//插入队尾
}

CloudQueue::ElemType CloudQueue::DeQueue()
{
	if (QueueEmpty()) {
		printf("队列已空! 出队失败!\n");
		return{};
	}
	//TODO: 实现循环队列出队的代码
	cloudQueue.Q.front = (cloudQueue.Q.front + 1) % MAXSIZE;//队头指针后移，如到在最后转到头部
	ElemType x = cloudQueue.Q.data[cloudQueue.Q.front];
	return x;
}

//获取队列长度
int CloudQueue::QueueSize()
{
	return cloudQueue.Q.rear >= cloudQueue.Q.front ? cloudQueue.Q.rear - cloudQueue.Q.front : cloudQueue.Q.rear - cloudQueue.Q.front + MAXSIZE;
}


//创建点云数据集
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
			//添加到队列
			EnQueue(cloud);
		}
		catch (exception e) {
			cloudQueue.pointFlag = false;
		}
		//再等1秒再生点云
		Sleep(1000);
	}
}

