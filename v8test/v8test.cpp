// v8test.cpp : 定义控制台应用程序的入口点。
//
#include "stdafx.h" 
#include<windows.h>  
#include<iostream> 
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
	stream.start(); 
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
		parseStream(root);
		
	}catch (exception const & e) {
		stream.sendMsg(STREAM_FAIL,string("error:")+string(e.what()));
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
	//pClient = nullptr;
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
 

	stream.loadDll();


	//WebSockServer::Instance().Init(9002,
	//	boost::bind(on_open, _1),
	//	boost::bind(on_close, _1, _2),
	//	boost::bind(on_message, _1, _2, _3)
	//);
	//WebSockServer::Instance().StartServer();

	//std::string str;
	//while (std::cin >> str)
	//{
	//	if (pClient != nullptr)
	//	{
	//		if (str == "close")
	//		{
	//			WebSockServer::Instance().Close(pClient);
	//		}
	//		else
	//		{
	//			WebSockServer::Instance().Send(pClient, str, WsOpcode::TEXT);
	//		}
	//	}
	//} 


	cout << "开始解析脚本" << endl;
	string doc("{\"type\":1,\"uuid\":\"1b8459b2-0297-4029-9f5b-bec5d2a6c27f\",\"doc\":[{\"method\":\"FileInput\",\"params\":[\"table.ply\"]},{\"method\":\"PassThrough\",\"params\":[1,-10,-0.2,0]},{\"method\":\"RadiusOutlierRemoval\",\"params\":[0.02,1]},{\"method\":\"VoxelGrid\",\"params\":[0.02]},{\"method\":\"StatisticalOutlierRemoval\",\"params\":[50,1],\"conditionType\":1,\"conditionParams\":[0.8]}]}");
 
	Json::Reader reader;
	Json::Value root; 
	reader.parse(doc, root, false);
	parseStream(root);
 

	 
    system("pause");
    return 0;
}
