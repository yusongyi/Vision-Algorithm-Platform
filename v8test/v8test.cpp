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

void on_message(void* pClient, const std::string data, WsOpcode opcode)
{
	Json::Reader reader;
	Json::Value root;

	cout << "开始解析数据："<< data << endl;
	try {
		if (!reader.parse(data, root, false)) {

			stream.sendMsg("failed to parse!");
			printf("failed to parse!\n");
			return;
		}
		int type = root["type"].asInt();
		string uuid = root["uuid"].asString();

		//保存本次计算的uuid和客户端socket通道
		stream.uuid = uuid;
		stream.clientWs = pClient;

		if (type == 0) {

			//string doc("[{\"method\":\"voxelGrid\",\"paramSize\":1,\"params\":[0.1]}]");
			string doc = root["doc"].asString();
			int init_res = stream.init(doc);
			if (init_res != 0) {
				return;
			}

			string dataPath = root["dataPath"].asString();
			pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
			//pcl::io::loadPCDFile ("C:\\Users\\YuSongyi\\Desktop\\table.pcd", *cloud);
			//pcl::io::loadPLYFile("E:\\CODE\\three.js\\examples\\models\\ply\\binary\\Lucy100k.ply", *cloud);
			pcl::io::loadPLYFile(dataPath, *cloud);

			stream.start(cloud);
		}
	}catch (exception const & e) {
		stream.sendMsg(strcat("error:",e.what()));
		std::cout << e.what() << std::endl;
		return ;
	}

	stream.sendMsg(data);
}

void on_open(void* pClient)
{
	::pClient = pClient;
}

void on_close(void* pClient, std::string msg)
{
	pClient = nullptr;
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


	cout << "开始解析脚本" << endl;
	string doc("[{\"method\":\"statisticalOutlierRemoval\",\"paramSize\":2,\"params\":[1,0.1]},{\"method\":\"voxelGrid\",\"paramSize\":1,\"params\":[0.1]}]");
 

	//string doc = readConfig("algo.json");
	cout << doc << endl;

	stream.init(doc);

	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile("table.pcd", *cloud);
	//pcl::io::loadPCDFile("123_voxelGrid.ply", *cloud);
	std::cout << "original cloud size : " << cloud->size() << std::endl; 

	 
	stream.start(cloud);
    system("pause");
    return 0;
}