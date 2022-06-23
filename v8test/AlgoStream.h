#pragma once  
#include <map> 
#include "AlgoNode.h"
#include "json/json.h"
#include <string>
using namespace std;

enum StreamOpcode
{
	STREAM_START  = 1,
	STREAM_DOING = 2,
	STREAM_END    = 3,
	STREAM_FAIL  = 9
};

class AlgoStream
{
	
	
public: 
	static  string ROOT_PATH;
	int type;
	int size;
	AlgoNode* algos; 
	AlgoNode fileNode;

	void* clientWs;
	string uuid;

	void loadDll();
	int init(Json::Value doc);
	void start();
	void sendNodeRes(AlgoNode node); 
	void sendMsg(StreamOpcode type, string msg);

	pcl::PointCloud<PointT>::Ptr input;
};
