#include "StdAfx.h"
#include<windows.h>  
#include <ShellAPI.h>
#include "AlgoStream.h"  
#include "AlgoNode.h"
#include<iostream>
#include <thread>
#include <time.h>
#include "string"
#include <map>
#include "json/json.h"
#include <direct.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/statistical_outlier_removal.h> 
#include <pcl/kdtree/kdtree.h>
#include <liblas/liblas.hpp>
#include "web_sock_server.h"
#include "Thread.h"
#include "SmartTool.h"


using namespace pcl;
using namespace std;

//存储各个算法的地址
map<string, RUN_FUN> mFuncPtr;

typedef pcl::PointXYZ PointT;

//DLL 和 点云文件的根目录
string AlgoStream::ROOT_PATH = "E:\\data\\pcl\\";

//potree转换器的程序地址
static string CONVERT_TOOL_PATH = AlgoStream::ROOT_PATH + "potree\\PotreeConverter.exe";

//演示模式的暂停秒数
static int DEMO_SLEEP = 2;


//根据名字获取函数
RUN_FUN getFunction(string funcName)
{
	map<string, RUN_FUN>::iterator it = mFuncPtr.find(funcName);

	if (it != mFuncPtr.end())
		return it->second;
	return 0;
}

void AlgoStream::loadConfig(Json::Value config) {
	if (config["rootPath"].type() != Json::nullValue) { 
		AlgoStream::ROOT_PATH = config["rootPath"].asString();
	}
	if (config["convertToolPath"].type() != Json::nullValue) {
		CONVERT_TOOL_PATH = config["convertToolPath"].asString();
	}
	if (config["demoSleep"].type() != Json::nullValue) {
		DEMO_SLEEP = config["demoSleep"].asInt();
	}

	cout << "ROOT_PATH:" << AlgoStream::ROOT_PATH << endl;
	cout << "CONVERT_TOOL_PATH:" << CONVERT_TOOL_PATH << endl;
	cout << "DEMO_SLEEP:" << DEMO_SLEEP << endl;

}

//查到DLL
void AlgoStream::loadDll() {
	cout << "开始查找dll并注册" << endl;
	vector<string> files;

	//查看系统当前目录
	char filePath[_MAX_PATH];
	_getcwd(filePath, _MAX_PATH);
	cout << "当前路径:" << filePath << std::endl;
	cout << "----------------" << endl;

	//获取该路径下的所有dll文件
	getFiles(filePath, "dll", files);

	//列表文件输出路径 
	int size = files.size();

	//pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	//pcl::io::loadPCDFile("table.pcd", *cloud);
	//std::cout << "original cloud size : " << cloud->size() << std::endl;
	//pcl::PointCloud<PointT>::Ptr res(new pcl::PointCloud<PointT>); 
	for (int i = 0; i < size; i++)
	{
		cout << "找到 DLL:" << files[i] << endl;

		//获取文件名
		string name;
		get_FileBaseName(files[i], name);

		//获取方法名，dll文件必须以 algo_add.dll 命名，其中 add 为此算法模块的函数名称。
		size_t found = name.find_last_of('.');
		size_t found1 = name.find_last_of('_');
		string method = name.substr(found1 + 1, found - (found1 + 1));

		//加载dll
		HMODULE hDll = LoadLibrary(name.c_str());
		if (hDll != NULL)
		{
			//根据上面获取的方法名调用方法，获取运行方法，后续还有前置方法、后置方法、测试方法  
			RUN_FUN fp1 = RUN_FUN(GetProcAddress(hDll, method.c_str()));
			if (fp1 != NULL)
			{
			/*	float balance[2] = { 0.1,0.1 };
				cout << "函数测试数据大小:" << cloud->size() << endl;
				fp1(cloud, res, balance);
				cout << "函数测试结果大小:" << res->size() << endl;*/


				mFuncPtr.insert(make_pair(method, fp1));
				cout << "函数注册成功:" << method << endl;

			}
		}
		cout << "----------------" << endl;
	}
	cout << "查找结束!" << endl;
	cout << "----------------" << endl;
}

//获取ply点云文件的输出路径，根路径/uuid_算法名称.ply
string getOutPath(string uuid, string node) {
	return AlgoStream::ROOT_PATH+uuid + "_" + node + ".ply";
}

/*

调用PotreeConverter生成Potree文件
	stream: 算法流程，用于获取uuid
	node  : 算法节点，需要节点名称、此节点的生成点云文件

*/
void genPotreeFiles(AlgoStream stream, AlgoNode *node) {
	node->potreePath = "potree_res\\"+stream.uuid + "_" + node->name;
	string params = node->outPath + " -o " + node->potreePath;

	ShellExecuteA(NULL, "open", CONVERT_TOOL_PATH.c_str(), params.c_str(), AlgoStream::ROOT_PATH.c_str(), SW_SHOW);
}
/*

将传入的点云文数据生成点云文件，并转换为Potree文件
	cloud : 点云数据
	stream: 算法流程，用于获取uuid
	node  : 算法节点，需要节点名称、此节点的生成点云文件

*/
void genPotreeFiles(pcl::PointCloud<PointT>::Ptr cloud, AlgoStream stream, AlgoNode *node) {
	//输出ply
	pcl::io::savePLYFileBinary(node->outPath, *cloud);
	genPotreeFiles(stream, node);
}
  
/*
	
发送JSON格式WebSocket消息
	type: 类型 STREAM_START  = 1,
			   STREAM_DOING = 2,
			   STREAM_END    = 3,
			   STREAM_FAIL  = 9
	msg: 信息，可能是json数据

*/
void AlgoStream::sendMsg(StreamOpcode type,string msg) {
	Json::Value root;
	root["type"] = Json::Value(type);
	root["msg"] = Json::Value(msg);
	Json::FastWriter fw;
	//向socket输出
	if (clientWs != NULL) { 
		cout << "sendMsg:" << fw.write(root) << endl;
		WebSockServer::Instance().Send(clientWs, GBKToUTF8(fw.write(root)), WsOpcode::TEXT);
	}
}

/*

发送算法节点的执行结果
	node: 算法节点

*/
void AlgoStream::sendNodeRes(AlgoNode node) {
	Json::Value root;
	root["id"] = Json::Value(node.id);
	root["uuid"] = Json::Value(uuid);
	root["node"] = Json::Value(node.name);
	root["chName"] = Json::Value(node.chName);
	root["outPath"] = Json::Value(node.potreePath);

	if (node.conditionType != -1) {
		root["conditionCount"] = Json::Value(node.conditionCount);
		root["conditionNext"] = Json::Value(node.conditionNext);
	}

	Json::FastWriter fw;
	sendMsg(STREAM_DOING,fw.write(root));
}


void AlgoStream::start(){

	printf("输入数据大小:%d\r\n", input->size());

	//转换输入文件
	genPotreeFiles(*this, &fileNode);

	//开始时发送初始的Potree文件
	Json::Value root; 
	root["uuid"] = Json::Value(uuid);
	root["id"] = Json::Value(fileNode.id);
	root["chName"] = Json::Value(fileNode.chName);
	root["outPath"] = Json::Value(fileNode.potreePath);
	Json::FastWriter fw; 
	sendMsg(STREAM_START, fw.write(root));

	//演示模式暂停
	if (type == 2) {
		mySleep(DEMO_SLEEP);
	}

	for(int i=0;i<size;i++){ 
		algos[i].input = input; 

		//输出的点云文件地址
		algos[i].outPath = getOutPath(uuid, algos[i].name); 

		pcl::PointCloud<PointT>::Ptr res(new pcl::PointCloud<PointT>);
		algos[i].out = res;
 
		do { 
			//调用函数fun1 
			algos[i].runAddr(input, algos[i].out, algos[i].params);

			//生成Potree文件
			genPotreeFiles(algos[i].out, *this, &algos[i]);

			//分支结构则进行计算
			if (algos[i].conditionType != -1) {
				float remain = algos[i].out->size() / (float)(algos[i].input->size());

				printf("BRANCH_RES: node:%s,input size:%d,out size:%d,remain:%f \r\n", algos[i].name.c_str(), algos[i].input->size(), algos[i].out->size(), remain);

				algos[i].conditionCount++;

				//判断剩余点云数量是否符合条件
				algos[i].conditionNext = (
					(algos[i].conditionType == 0 && remain <= algos[i].conditionParams[0]) ||
					(algos[i].conditionType == 1 && remain >= algos[i].conditionParams[0]) ||
					(algos[i].conditionType == 2 && remain != algos[i].conditionParams[0])
					)
					&&
					algos[i].conditionCount < 99;
			}
			else {
				//输出结果
				printf("NODE_RES: node:%s,input size:%d,out size:%d \r\n", algos[i].name.c_str(), algos[i].input->size(), algos[i].out->size());

				//不是分支结构则默认仅执行一次
				algos[i].conditionNext = false;
			}

			//输出本计算节点结果
			sendNodeRes(algos[i]);

			//本机预览
			if (type == 1) {
				visualization_point(input, algos[i].out);
			}

			//演示模式暂停
			if (type == 2) {
				mySleep(DEMO_SLEEP);
			}

			//下个节点的输入为本次的输出
			input = algos[i].out;
		} while (algos[i].conditionNext); 
	}
	sendMsg(STREAM_END,uuid);
}


int AlgoStream::init(Json::Value doc) {
 
	//默认第一个节点的第一个参数为数据文件地址
	string dataPath = doc[0]["params"][0].asString();


	//初始文件输入节点
	fileNode.name = "inputFile"; 
	fileNode.chName = doc[0]["chName"].asString();
	fileNode.id = doc[0]["nodeId"].asString();
	fileNode.outPath = AlgoStream::ROOT_PATH + dataPath;

	//读取点云文件到input中
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	readPointCloud(fileNode.outPath, cloud);
	input = cloud;

	//初始化算法模块前需要将第一个节点去掉 
	doc.removeIndex(0,nullptr);
	 

	//初始化算法模块
	int siNum = doc.size();
	cout << "脚本共" << siNum << "个算法模块" << endl;
	size = siNum;
	algos = new AlgoNode[siNum]();
	for (int i = 0; i < siNum; i++)
	{
		Json::Value DevJson = doc[i];
		std::string DevStr = DevJson["method"].asString();
		cout << "开始初始化‘" << DevStr << "’算法模块" << endl;
		 
		 
		//获取函数
		RUN_FUN fptr = getFunction(DevStr.c_str());

		if (fptr != 0)
		{
			//初始化算法节点
			AlgoNode node;
			node.id = DevJson["nodeId"].asString();
			node.chName = DevJson["chName"].asString();
			node.name = DevStr.c_str();
			node.runAddr = fptr;

			//参数初始化
			if (DevJson["params"].type() != Json::nullValue) { 
				int paramSize = DevJson["params"].size();
				float* params = new float[paramSize]();
				for (int j = 0; j < paramSize; j++) {
					params[j] = DevJson["params"][j].asFloat();
				}
				node.paramSize = paramSize;
				node.params = params;
			}

			//分支条件初始化
			if (DevJson["conditionType"].type() != Json::nullValue) {
				node.conditionType = DevJson["conditionType"].asInt();

				if (DevJson["conditionParams"].type() == Json::nullValue) {
					cout << DevStr << ":模块初始化失败" << endl;
				}
				else { 
					int conditionParamSize = DevJson["conditionParams"].size();
					node.conditionParamSize = conditionParamSize;

					float* conditionParams = new float[conditionParamSize]();
					for (int k = 0; k < conditionParamSize; k++) {
						conditionParams[k] = DevJson["conditionParams"][k].asFloat();
					}
					node.conditionParams = conditionParams;
				}
				
			}
			else {
				node.conditionType = -1;
			}
			algos[i] = node; 
			cout << DevStr << ":模块初始化成功" << endl;
		}
		else {
			sendMsg(STREAM_FAIL,DevStr+string(":模块初始化失败"));
			cout << DevStr << ":模块初始化失败" << endl; 
			//return -1;
		}
		cout << "----------------" << endl;
	}

	return 0;

	
}
