//// v8test.cpp : 定义控制台应用程序的入口点。
////
#include "stdafx.h" 
#include<windows.h>  
#include<iostream> 
#include<future>
#include "string"
#include <map>
#include "json/json.h"
#include "AlgoStream.h"
#include "AlgoNode.h"
#include <iomanip>
#include <direct.h>
#include <io.h>
#include <vector> 
#include "web_sock_server.h"
#include <boost/bind.hpp>
#include "CloudQueue.h"
using namespace std; 
 


static AlgoStream stream;
void* pClient = nullptr;


void parseStream(Json::Value root) {

	int type = root["type"].asInt();
	string uuid = root["uuid"].asString();

	stream.uuid = uuid; 
	stream.type = type;
	Json::Value doc = root["doc"];
	int init_res = stream.init(doc);
	if (init_res != 0) {
		return;
	}

	//异步执行
	new boost::thread(&AlgoStream::start, &stream);
}

void parseCloudQueue() {

	//异步执行
	new boost::thread(&CloudQueue::start, &cloudQueue);
}


void on_message(void* pClient, const std::string data, WsOpcode opcode)
{
	//忽略心跳
	if (data.compare("HeartBeat") == 0) {
		return;
	}

	Json::Reader reader;
	Json::Value root;

	//保存本次计算的uuid和客户端socket通道
	stream.clientWs = pClient;

	cout << "开始解析数据："<< data << endl;
	try {
		if (!reader.parse(data, root, false)) {

			stream.sendMsg(STREAM_FAIL,"failed to parse!");
			printf("failed to parse!\n");
			return;
		}
		//获取控制是否生成点云及发送点云参数
		bool isStart = root["isStart"].asBool();
		//开始生成点云数据}
		cloudQueue.running = isStart;
		stream.running = isStart;
		if (isStart) { 
			parseCloudQueue();
			parseStream(root);
		}
		 
	}catch (exception const & e) {
		stream.sendMsg(STREAM_FAIL,string("error:")+string(e.what()));
		cloudQueue.running = false;
		stream.running = false;
		std::cout << e.what() << std::endl;
		return ;
	}
	 
}

void on_open(void* pClient)
{
	::pClient = pClient;
}

void on_close(void* pClient, std::string msg)
{ 
	//关闭生成与发送点云
	cloudQueue.running = false;
	stream.running = false;
} 

//读取
static std::string readConfig(const char* path) {
	FILE* file = fopen(path, "rb");
	if (!file)
		return std::string("");
	fseek(file, 0, SEEK_END);
	long size = ftell(file);
	fseek(file, 0, SEEK_SET);
	std::string text;
	char* buffer = new char[size + 1];
	buffer[size] = 0;
	if (fread(buffer, 1, size, file) == (unsigned long)size)
		text = buffer;
	fclose(file);
	delete[] buffer;
	return text;
}

 

int _tmain(int argc, _TCHAR* argv[])
{
	string configStr = readConfig("config.json");
	Json::Reader configReader;
	Json::Value config;
	 
	  
	if (!configReader.parse(configStr, config, false)) {
		printf("config error!\n");
		return -1;
	} 
	stream.loadConfig(config);

	stream.loadDll();


	WebSockServer::Instance().Init(9002,
		boost::bind(on_open, _1),
		boost::bind(on_close, _1, _2),
		boost::bind(on_message, _1, _2, _3)
	);
	WebSockServer::Instance().StartServer();

	std::string str;
	while (std::cin >> str)
	{
		if (pClient != nullptr)
		{
			if (str == "close")
			{
				WebSockServer::Instance().Close(pClient);
			}
			else
			{
				WebSockServer::Instance().Send(pClient, str, WsOpcode::TEXT);
			}
		}
	} 


	/*cout << "开始解析脚本" << endl;
	string doc("{\"type\":1,\"uuid\":\"1b8459b2-0297-4029-9f5b-bec5d2a6c27f\",\"doc\":[{\"nodeId\":\"P1\",\"method\":\"FileInput\",\"name\":\"FileInput\",\"chName\":\"输入点云\",\"inputs\":[],\"outputs\":[{\"pid\":\"P1\",\"id\":\"OUT111\",\"name\":\"上\",\"dataType\":1}],\"params\":[\"table.ply\"]},{\"nodeId\":\"P2\",\"name\":\"Shangbian\",\"chName\":\"上边\",\"method\":\"Shangbian\",\"inputs\":[{\"pid\":\"P2\",\"id\":\"INT222\",\"name\":\"来源\",\"resourceId\":\"OUT111\",\"dataType\":1}],\"outputs\":[{\"pid\":\"P2\",\"id\":\"OUT221\",\"name\":\"轮廓点云\",\"dataType\":1},{\"pid\":\"P2\",\"id\":\"OUT222\",\"name\":\"边缘线\",\"dataType\":3},{\"pid\":\"P2\",\"id\":\"OUT223\",\"name\":\"边缘平面\",\"dataType\":2}],\"params\":[1,-10,-0.2,0]},{\"nodeId\":\"P4\",\"name\":\"Jiaodian\",\"chName\":\"交点\",\"method\":\"Jiaodian\",\"inputs\":[{\"pid\":\"P4\",\"id\":\"INT441\",\"name\":\"特征一\",\"resourceId\":\"OUT222\",\"dataType\":3},{\"pid\":\"P4\",\"id\":\"INT442\",\"name\":\"特征二\",\"resourceId\":\"OUT332\",\"dataType\":3}],\"outputs\":[{\"pid\":\"P4\",\"id\":\"OUT441\",\"name\":\"交叉点\",\"dataType\":4}],\"params\":[1,-10,-0.2,0]},{\"nodeId\":\"P3\",\"name\":\"Zuobian\",\"chName\":\"左边\",\"method\":\"Zuobian\",\"inputs\":[{\"pid\":\"P3\",\"id\":\"INT333\",\"name\":\"来源\",\"resourceId\":\"OUT111\",\"dataType\":1}],\"outputs\":[{\"pid\":\"P3\",\"id\":\"OUT331\",\"name\":\"轮廓点云\",\"dataType\":1},{\"pid\":\"P3\",\"id\":\"OUT332\",\"name\":\"边缘线\",\"dataType\":3},{\"pid\":\"P3\",\"id\":\"OUT333\",\"name\":\"边缘平面\",\"dataType\":2}],\"params\":[1,-10,-0.2,0]}]}");
	Json::Reader reader;
	Json::Value root; 
	reader.parse(doc, root, false);
	parseStream(root);*/

	 
    system("pause");
    return 0;
}
