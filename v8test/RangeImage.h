#pragma once
#include "AlgoNode.h"


//深度图
class RangeImage
{
public:

	//点云转深度图base64格式
	unsigned char * pointsToImage(pcl::PointCloud<PointT>::Ptr cloud);
 
	//根据点云数据大小获取图片字节数
	int getImgSize(pcl::PointCloud<PointT>::Ptr cloud);
};