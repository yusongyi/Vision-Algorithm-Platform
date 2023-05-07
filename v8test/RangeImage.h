#pragma once
#include "AlgoNode.h"


//深度图
class RangeImage
{
public:

	//点云转深度图base64格式
	string pointsToImage(pcl::PointCloud<PointT>::Ptr cloud);

	//图片转base64
	std::string base64_encode(const char* bytes_to_encode, unsigned int in_len);

	//生成时间戳
	long long systemtime();

	//long转string
	string longtostring(long long t);
};