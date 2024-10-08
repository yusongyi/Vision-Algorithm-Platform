#pragma once  
#include <map> 
#include "AlgoNode.h"
#include "json/json.h"
#include <string>
using namespace std;

enum StreamOpcode
{

	//算法开始
	STREAM_START  = 1,

	//节点输出结果
	STREAM_DOING = 2,

	//算法结束
	STREAM_END    = 3, 

	//节点开始
	NODE_START = 4,

	//输入点云数据
	SEND_CLOUD = 5,

	//算法执行失败
	STREAM_FAIL  = 9,

};
enum MsgType
{

	//算法运行过程中状态信息
	STREAM_STATE = 1,

	//节点输出结果
	DEPTH_IMG = 2,

	//点云数据
	POINT_CLOUD = 3, 

};

//算法整体流程
class AlgoStream
{
	
public:  

	//是否执行算法
	bool running = false;

	//执行类型，调试或运行
	int type;

	//算法节点数量
	int size;

	//当前前端展示的点云ID,用于后端执行限流，否则可能导致websockt缓冲区变大
	int curShowIdx;

	//算法节点
	AlgoNode* algos; 

	//初始节点
	AlgoNode firstNode;

	//客户端websocket
	void* clientWs;

	//本次运行的ID
	string uuid;

	//发送点云的ID
	int cloudId;

	//加载系统配置
	void loadConfig(Json::Value config);

	//加载算法DLL文件
	void loadDll();

	//初始化算法配置
	int init(Json::Value doc);

	//开始执行算法
	void start();

	//发送节点输出结果
	void sendNodeRes(AlgoNode node); 

	//发送消息
	void sendMsg(StreamOpcode type, string msg);

	//发送输入点云数据
	void sendCloudData(pcl::PointCloud<PointT>::Ptr cloud, int cloudId); 
	 
};
