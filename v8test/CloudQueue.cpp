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
	return (Q.rear + 1) % MAXSIZE == Q.front;//加一取模
}

bool CloudQueue::QueueEmpty()
{  
	return Q.front == Q.rear;//当队列为空时front 和rear相等
}

void CloudQueue::EnQueue(CloudQueue::CloudObj x)
{
	if (QueueFull())
		//是否为空队列
	{
		return;
	}  
	Q.rear = (Q.rear + 1) % MAXSIZE;//尾部指针后移，如到最后转到头部
	Q.data[Q.rear] = x;//插入队尾
}

CloudQueue::CloudObj CloudQueue::DeQueue()
{
	if (QueueEmpty()) {
		printf("队列已空! 出队失败!\n");
		return{};
	}  
	Q.front = (Q.front + 1) % MAXSIZE;//队头指针后移，如到在最后转到头部
	CloudObj x = Q.data[Q.front];
	return x;
}

//获取队列长度
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


//创建随机点云数据集
void CloudQueue::start()
{ 
	int cloudId = 0;
	while (true) {
		if (QueueSize() < 10000) {
			for (int i = 0; i < 1000; i++) {
				//给结构体赋值
				CloudQueue::CloudObj cloudObj = { cloudId, cloudAry[i], i };
				//添加到队列
				EnQueue(cloudObj);
			} 
			cloudId++; 
		} 
		//再等1秒再生点云
		Sleep(100); 
	}
}


