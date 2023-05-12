#include "stdafx.h" 
#include "CloudQueue.h"


void CloudQueue::InitQueue()
{
	cloudQueue.Q.front = cloudQueue.Q.rear = 0;
	//将列队清空
}

bool CloudQueue::QueueFull()
{ 
	std::lock_guard<std::mutex> lock(cloudQueue.mutex_);
	return (cloudQueue.Q.rear + 1) % MAXSIZE == cloudQueue.Q.front;//加一取模
}

bool CloudQueue::QueueEmpty()
{ 
	std::lock_guard<std::mutex> lock(cloudQueue.mutex_);
	return cloudQueue.Q.front == cloudQueue.Q.rear;//当队列为空时front 和rear相等
}

void CloudQueue::EnQueue(CloudQueue::CloudObj x)
{
	if (QueueFull())
		//是否为空队列
	{
		return;
	} 
	std::lock_guard<std::mutex> lk(cloudQueue.mutex_);
	cloudQueue.Q.rear = (cloudQueue.Q.rear + 1) % MAXSIZE;//尾部指针后移，如到最后转到头部
	cloudQueue.Q.data[cloudQueue.Q.rear] = x;//插入队尾
}

CloudQueue::CloudObj CloudQueue::DeQueue()
{
	if (QueueEmpty()) {
		printf("队列已空! 出队失败!\n");
		return{};
	} 
	std::lock_guard<std::mutex> lk(cloudQueue.mutex_);
	cloudQueue.Q.front = (cloudQueue.Q.front + 1) % MAXSIZE;//队头指针后移，如到在最后转到头部
	CloudObj x = cloudQueue.Q.data[cloudQueue.Q.front];
	return x;
}

//获取队列长度
int CloudQueue::QueueSize()
{
	std::lock_guard<std::mutex> lock(cloudQueue.mutex_);
	return cloudQueue.Q.rear >= cloudQueue.Q.front ? cloudQueue.Q.rear - cloudQueue.Q.front : cloudQueue.Q.rear - cloudQueue.Q.front + MAXSIZE;
}




//创建随机点云数据集
void CloudQueue::start()
{
	int index = 0;
	int cloudId = 0;
	while (CloudQueue::running) {
		pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
		//PointT start;
		//start.x = cloudId;
		//start.y = cloudId;
		//start.z = cloudId;
		//cloud->push_back(start);
		//for (int i = 0; i < 1000; ++i) {
			for (int j = 0; j < 1000; ++j) {
				float y = 0.1 * rand() / (RAND_MAX + 1.0f);
				float x = 3 + index / 10.0 + y;
				float z = 3 + j / 10.0 + y;
				y = 4 +  rand() / (RAND_MAX + 1.0f);
				PointT p;
				p.x = x;
				p.y = z;
				p.z = y;
				cloud->push_back(p);
			}
		//}
		try {
			//给结构体赋值
			CloudQueue::CloudObj cloudObj = { cloudId, cloud };
			//添加到队列
			EnQueue(cloudObj);
		}
		catch (exception e) {
			CloudQueue::running = false;
		}
		//再等1秒再生点云
		//Sleep(100);
		++index;
		if (index == 1000) {
			std::lock_guard<std::mutex> lock(cloudQueue.mutex_);
			++cloudId;
			cout << "setclouudId = " << cloudId << endl;
			index = 0;
		}
	}
}


