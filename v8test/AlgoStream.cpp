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
#include"topological_sort.h"

using namespace pcl;
using namespace std;

//存储各个算法的地址
map<string, RUN_FUN> mFuncPtr;

//存储各个节点的地址
map<string, AlgoNode*> algoNodeMap;

//存储各个输入的地址
map<string, NodeInput*> nodeInputMap;

//存储各个输出的地址
map<string, NodeOutput*> nodeOutputMap;

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


 AlgoNode*  sortNode(AlgoNode* algos, int size) { 
	Graph_DG graph(algos, size);

	graph.createGraph(); 
	if (graph.topological_sort()) {
		return graph.algos;
	}
	return NULL;
	//graph.topological_sort_by_dfs();
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
 
	for (int i = 0; i < node.outputSize; i++) {
		NodeOutput output = *node.outputs[i];
		root["ouputs"].append(output.toJson());
	}

	Json::FastWriter fw;
	sendMsg(STREAM_DOING,fw.write(root));
}


void AlgoStream::start(){

	printf("输入数据大小:%d\r\n", input->size());
 

	//开始时发送初始的Potree文件
	Json::Value root; 
	root["uuid"] = Json::Value(uuid);
	root["id"] = Json::Value(fileNode.id);
	root["chName"] = Json::Value(fileNode.chName); 
	Json::FastWriter fw; 
	sendMsg(STREAM_START, fw.write(root));

	 

	for(int i=0;i<size;i++){  

		//节点开始
		Json::Value root;
		root["id"] = Json::Value(algos[i].id);  
		Json::FastWriter fw;
		sendMsg(NODE_START, fw.write(root));

		//演示模式暂停
		if (type == 2) {
			mySleep(DEMO_SLEEP);
		}
		
		//调用函数fun1 
		algos[i].runAddr(input, algos[i].inputs, algos[i].outputs, algos[i].params);
		 
		//输出本计算节点结果
		sendNodeRes(algos[i]);

		//本机预览
		if (type == 1) {
			visualization_point(input, algos[i].outputs, algos[i].outputSize);
		}

	
 
	 
	}
	sendMsg(STREAM_END,uuid);
}


int AlgoStream::init(Json::Value doc) {

	//初始化算法模块 
	for (int i = 0; i < doc.size(); i++)
	{
		Json::Value nodeJson = doc[i];
		std::string DevStr = nodeJson["method"].asString();
		if (DevStr.compare("FileInput") == 0) {
			Json::Value t = doc[0];
			doc[0] = nodeJson;
			doc[i] = t;
			break;
		}
	}
 
	//默认第一个节点的第一个参数为数据文件地址
	string dataPath = doc[0]["params"][0].asString();


	//初始文件输入节点
	fileNode.name = "inputFile"; 
	fileNode.chName = doc[0]["chName"].asString();
	fileNode.id = doc[0]["nodeId"].asString(); 

	//读取点云文件到input中
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	readPointCloud(AlgoStream::ROOT_PATH + dataPath, cloud);
	input = cloud;


	//初始化文件输入节点
 	fileNode.outputs = new NodeOutput*[1]();
	NodeOutput *nodeOutput = new NodeOutput();
	Json::Value defaultOut = doc[0]["outputs"][0];
	nodeOutput->pid = fileNode.id;
	nodeOutput->name = defaultOut["name"].asString();
	nodeOutput->id = defaultOut["id"].asString();
	nodeOutput->dataType = defaultOut["dataType"].asInt();
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	nodeOutput->coeff = coefficients;
	
	//文件输入节点存入map中，便于对应input的resource
	nodeOutputMap.insert(make_pair(nodeOutput->id, nodeOutput));
	fileNode.outputs[0] = nodeOutput;


	//初始化算法模块前需要将第一个节点去掉 
	doc.removeIndex(0,nullptr);
	 

	//初始化算法模块
	int siNum = doc.size();
	cout << "脚本共" << siNum << "个算法模块" << endl;
	size = siNum;
	algos = new AlgoNode[siNum]();
	for (int i = 0; i < siNum; i++)
	{
		Json::Value nodeJson = doc[i];
		std::string DevStr = nodeJson["method"].asString();
		cout << "开始初始化‘" << DevStr << "’算法模块" << endl;
		 
		 
		//获取函数
		RUN_FUN fptr = getFunction(DevStr.c_str());

		if (fptr != 0)
		{
			//初始化算法节点
			AlgoNode node;
			node.id = nodeJson["nodeId"].asString();
			node.chName = nodeJson["chName"].asString();
			node.name = DevStr.c_str();
			node.runAddr = fptr;
			

			//输入初始化
			Json::Value inputJson = nodeJson["inputs"];
			int inputSize = inputJson.size();
			node.inputs = new NodeInput*[inputSize]();
			for (int j = 0; j < inputSize; j++) {

				//构造输入节点
				NodeInput *nodeInput = new NodeInput();

				nodeInput->pid = node.id;
				nodeInput->name = inputJson[j]["name"].asString();
				nodeInput->id = inputJson[j]["id"].asString();
				nodeInput->dataType = inputJson[j]["dataType"].asInt();
				nodeInput->resourceId = inputJson[j]["resourceId"].asString();
				nodeInput->pIdx = i;

				//存入map中，便于对应output的target
				nodeInputMap.insert(make_pair(nodeInput->id, nodeInput));
				node.inputs[j] = nodeInput;
			}
			node.inputSize = inputSize;


			//输出初始化
			Json::Value outputJson = nodeJson["outputs"];
			int outputSize = outputJson.size();
			node.outputs = new NodeOutput*[outputSize]();
			for (int k = 0; k < outputSize; k++) {

				//构造输出节点
				NodeOutput *nodeOutput = new NodeOutput();
				nodeOutput->pid = node.id;
				nodeOutput->name = outputJson[k]["name"].asString();
				nodeOutput->id = outputJson[k]["id"].asString();
				nodeOutput->dataType = outputJson[k]["dataType"].asInt();
				nodeOutput->targetSize = 0;

				//存入map中，便于对应input的resource
				nodeOutputMap.insert(make_pair(nodeOutput->id, nodeOutput));
				node.outputs[k] = nodeOutput;

				 
			}
			node.outputSize = outputSize;

			//参数初始化
			if (nodeJson["params"].type() != Json::nullValue) { 
				int paramSize = nodeJson["params"].size();
				float* params = new float[paramSize]();
				for (int j = 0; j < paramSize; j++) {
					params[j] = nodeJson["params"][j].asFloat();
				}
				node.paramSize = paramSize;
				node.params = params;
			}

			 
			algos[i] = node;
			algoNodeMap.insert(make_pair(node.id, &node));
			cout << DevStr << ":模块初始化成功" << endl;
		}
		else {
			sendMsg(STREAM_FAIL,DevStr+string(":模块初始化失败"));
			cout << DevStr << ":模块初始化失败" << endl; 
			//return -1;
		}
		cout << "----------------" << endl;
	}

	//遍历输入节点，将数据来源关联其他输出节点
	map<string, NodeInput*>::iterator inputIter;
	for (inputIter = nodeInputMap.begin(); inputIter != nodeInputMap.end(); inputIter++)
	{
		NodeInput *input = inputIter->second;
		string resourceId = input->resourceId;
		NodeOutput *outPut = nodeOutputMap[resourceId];
		outPut->targetSize++;

		input->resource = outPut;
	}

	//遍历输出节点，初始化关联的输入节点数量
	map<string, NodeOutput*>::iterator outputIter;
	for (outputIter = nodeOutputMap.begin(); outputIter != nodeOutputMap.end(); outputIter++)
	{
		NodeOutput *output = outputIter->second; 
		output->targets = new NodeInput[output->targetSize]();
	 
	}

	//再遍历一遍输入节点，将输入节点放到数据源所在的输出节点末尾
	for (inputIter = nodeInputMap.begin(); inputIter != nodeInputMap.end(); inputIter++)
	{
		NodeInput *input =inputIter->second;
		string resourceId = input->resourceId;
		NodeOutput *outPut = nodeOutputMap[resourceId];
		outPut->targets[outPut->targetIdx++] = *input;
	}

	algos = sortNode(algos, siNum);
	return 0;

	
}
