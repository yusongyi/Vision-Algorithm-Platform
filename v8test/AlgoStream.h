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
	NODE_START = 4,
	SEND_CLOUD = 5,
	STREAM_FAIL  = 9,

};

class AlgoStream
{
	
public: 
	static  string ROOT_PATH;
	bool sendPoint = false;
	int type;
	int size;
	AlgoNode* algos; 
	AlgoNode fileNode;

	void* clientWs;
	string uuid;


	void loadConfig(Json::Value config);
	void loadDll();
	int init(Json::Value doc);
	void start();
	void sendNodeRes(AlgoNode node); 
	void sendMsg(StreamOpcode type, string msg);
	void sendCloudData(pcl::PointCloud<PointT>::Ptr cloud);
	pcl::PointCloud<PointT>::Ptr checkSize(pcl::PointCloud<PointT>::Ptr cloud);
	pcl::PointCloud<PointT>::Ptr input;
};
