#pragma once
#include "AlgoNode.h"

class RangeImage
{
public:
	string pointsToImage(pcl::PointCloud<PointT>::Ptr cloud);
	std::string base64_encode(const char* bytes_to_encode, unsigned int in_len);
	long long systemtime();
	string longtostring(long long t);
};