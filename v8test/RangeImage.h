#pragma once
#include "AlgoNode.h"


//���ͼ
class RangeImage
{
public:

	//����ת���ͼbase64��ʽ
	unsigned char * pointsToImage(pcl::PointCloud<PointT>::Ptr cloud);
 
	//���ݵ������ݴ�С��ȡͼƬ�ֽ���
	int getImgSize(pcl::PointCloud<PointT>::Ptr cloud);
};