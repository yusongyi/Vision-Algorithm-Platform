#pragma once
#include "AlgoNode.h"


//���ͼ
class RangeImage
{
public:

	//����ת���ͼbase64��ʽ
	string pointsToImage(pcl::PointCloud<PointT>::Ptr cloud);

	//ͼƬתbase64
	std::string base64_encode(const char* bytes_to_encode, unsigned int in_len);

	//����ʱ���
	long long systemtime();

	//longתstring
	string longtostring(long long t);
};