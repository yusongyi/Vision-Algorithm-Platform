#pragma once  
#include <map> 
#include "AlgoNode.h"
#include <string>
using namespace std;


class AlgoStream
{
public: 
	int size;
	AlgoNode* algos; 

	void* clientWs;
	string uuid;

	void loadDll();
	void init(string doc);
	void start(pcl::PointCloud<PointT>::Ptr input);
	void sendNodeRes(AlgoNode node);

};

