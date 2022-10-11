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

//�洢�����㷨�ĵ�ַ
map<string, RUN_FUN> mFuncPtr;

//�洢�����ڵ�ĵ�ַ
map<string, AlgoNode*> algoNodeMap;

//�洢��������ĵ�ַ
map<string, NodeInput*> nodeInputMap;

//�洢��������ĵ�ַ
map<string, NodeOutput*> nodeOutputMap;

typedef pcl::PointXYZ PointT;

//DLL �� �����ļ��ĸ�Ŀ¼
string AlgoStream::ROOT_PATH = "E:\\data\\pcl\\";

//potreeת�����ĳ����ַ
static string CONVERT_TOOL_PATH = AlgoStream::ROOT_PATH + "potree\\PotreeConverter.exe";

//��ʾģʽ����ͣ����
static int DEMO_SLEEP = 2;


//�������ֻ�ȡ����
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

//�鵽DLL
void AlgoStream::loadDll() {
	cout << "��ʼ����dll��ע��" << endl;
	vector<string> files;

	//�鿴ϵͳ��ǰĿ¼
	char filePath[_MAX_PATH];
	_getcwd(filePath, _MAX_PATH);
	cout << "��ǰ·��:" << filePath << std::endl;
	cout << "----------------" << endl;

	//��ȡ��·���µ�����dll�ļ�
	getFiles(filePath, "dll", files);

	//�б��ļ����·�� 
	int size = files.size();

	//pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	//pcl::io::loadPCDFile("table.pcd", *cloud);
	//std::cout << "original cloud size : " << cloud->size() << std::endl;
	//pcl::PointCloud<PointT>::Ptr res(new pcl::PointCloud<PointT>); 
	for (int i = 0; i < size; i++)
	{
		cout << "�ҵ� DLL:" << files[i] << endl;

		//��ȡ�ļ���
		string name;
		get_FileBaseName(files[i], name);

		//��ȡ��������dll�ļ������� algo_add.dll ���������� add Ϊ���㷨ģ��ĺ������ơ�
		size_t found = name.find_last_of('.');
		size_t found1 = name.find_last_of('_');
		string method = name.substr(found1 + 1, found - (found1 + 1));

		//����dll
		HMODULE hDll = LoadLibrary(name.c_str());
		if (hDll != NULL)
		{
			//���������ȡ�ķ��������÷�������ȡ���з�������������ǰ�÷��������÷��������Է���  
			RUN_FUN fp1 = RUN_FUN(GetProcAddress(hDll, method.c_str()));
			if (fp1 != NULL)
			{
			/*	float balance[2] = { 0.1,0.1 };
				cout << "�����������ݴ�С:" << cloud->size() << endl;
				fp1(cloud, res, balance);
				cout << "�������Խ����С:" << res->size() << endl;*/


				mFuncPtr.insert(make_pair(method, fp1));
				cout << "����ע��ɹ�:" << method << endl;

			}
		}
		cout << "----------------" << endl;
	}
	cout << "���ҽ���!" << endl;
	cout << "----------------" << endl;
}
 
/*
	
����JSON��ʽWebSocket��Ϣ
	type: ���� STREAM_START  = 1,
			   STREAM_DOING = 2,
			   STREAM_END    = 3,
			   STREAM_FAIL  = 9
	msg: ��Ϣ��������json����

*/
void AlgoStream::sendMsg(StreamOpcode type,string msg) {
	Json::Value root;
	root["type"] = Json::Value(type);
	root["msg"] = Json::Value(msg);
	Json::FastWriter fw;
	//��socket���
	if (clientWs != NULL) { 
		cout << "sendMsg:" << fw.write(root) << endl;
		WebSockServer::Instance().Send(clientWs, GBKToUTF8(fw.write(root)), WsOpcode::TEXT);
	}
}

/*

�����㷨�ڵ��ִ�н��
	node: �㷨�ڵ�

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

	printf("�������ݴ�С:%d\r\n", input->size());
 

	//��ʼʱ���ͳ�ʼ��Potree�ļ�
	Json::Value root; 
	root["uuid"] = Json::Value(uuid);
	root["id"] = Json::Value(fileNode.id);
	root["chName"] = Json::Value(fileNode.chName); 
	Json::FastWriter fw; 
	sendMsg(STREAM_START, fw.write(root));

	 

	for(int i=0;i<size;i++){  

		//�ڵ㿪ʼ
		Json::Value root;
		root["id"] = Json::Value(algos[i].id);  
		Json::FastWriter fw;
		sendMsg(NODE_START, fw.write(root));

		//��ʾģʽ��ͣ
		if (type == 2) {
			mySleep(DEMO_SLEEP);
		}
		
		//���ú���fun1 
		algos[i].runAddr(input, algos[i].inputs, algos[i].outputs, algos[i].params);
		 
		//���������ڵ���
		sendNodeRes(algos[i]);

		//����Ԥ��
		if (type == 1) {
			visualization_point(input, algos[i].outputs, algos[i].outputSize);
		}

	
 
	 
	}
	sendMsg(STREAM_END,uuid);
}


int AlgoStream::init(Json::Value doc) {

	//��ʼ���㷨ģ�� 
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
 
	//Ĭ�ϵ�һ���ڵ�ĵ�һ������Ϊ�����ļ���ַ
	string dataPath = doc[0]["params"][0].asString();


	//��ʼ�ļ�����ڵ�
	fileNode.name = "inputFile"; 
	fileNode.chName = doc[0]["chName"].asString();
	fileNode.id = doc[0]["nodeId"].asString(); 

	//��ȡ�����ļ���input��
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	readPointCloud(AlgoStream::ROOT_PATH + dataPath, cloud);
	input = cloud;


	//��ʼ���ļ�����ڵ�
 	fileNode.outputs = new NodeOutput*[1]();
	NodeOutput *nodeOutput = new NodeOutput();
	Json::Value defaultOut = doc[0]["outputs"][0];
	nodeOutput->pid = fileNode.id;
	nodeOutput->name = defaultOut["name"].asString();
	nodeOutput->id = defaultOut["id"].asString();
	nodeOutput->dataType = defaultOut["dataType"].asInt();
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	nodeOutput->coeff = coefficients;
	
	//�ļ�����ڵ����map�У����ڶ�Ӧinput��resource
	nodeOutputMap.insert(make_pair(nodeOutput->id, nodeOutput));
	fileNode.outputs[0] = nodeOutput;


	//��ʼ���㷨ģ��ǰ��Ҫ����һ���ڵ�ȥ�� 
	doc.removeIndex(0,nullptr);
	 

	//��ʼ���㷨ģ��
	int siNum = doc.size();
	cout << "�ű���" << siNum << "���㷨ģ��" << endl;
	size = siNum;
	algos = new AlgoNode[siNum]();
	for (int i = 0; i < siNum; i++)
	{
		Json::Value nodeJson = doc[i];
		std::string DevStr = nodeJson["method"].asString();
		cout << "��ʼ��ʼ����" << DevStr << "���㷨ģ��" << endl;
		 
		 
		//��ȡ����
		RUN_FUN fptr = getFunction(DevStr.c_str());

		if (fptr != 0)
		{
			//��ʼ���㷨�ڵ�
			AlgoNode node;
			node.id = nodeJson["nodeId"].asString();
			node.chName = nodeJson["chName"].asString();
			node.name = DevStr.c_str();
			node.runAddr = fptr;
			

			//�����ʼ��
			Json::Value inputJson = nodeJson["inputs"];
			int inputSize = inputJson.size();
			node.inputs = new NodeInput*[inputSize]();
			for (int j = 0; j < inputSize; j++) {

				//��������ڵ�
				NodeInput *nodeInput = new NodeInput();

				nodeInput->pid = node.id;
				nodeInput->name = inputJson[j]["name"].asString();
				nodeInput->id = inputJson[j]["id"].asString();
				nodeInput->dataType = inputJson[j]["dataType"].asInt();
				nodeInput->resourceId = inputJson[j]["resourceId"].asString();
				nodeInput->pIdx = i;

				//����map�У����ڶ�Ӧoutput��target
				nodeInputMap.insert(make_pair(nodeInput->id, nodeInput));
				node.inputs[j] = nodeInput;
			}
			node.inputSize = inputSize;


			//�����ʼ��
			Json::Value outputJson = nodeJson["outputs"];
			int outputSize = outputJson.size();
			node.outputs = new NodeOutput*[outputSize]();
			for (int k = 0; k < outputSize; k++) {

				//��������ڵ�
				NodeOutput *nodeOutput = new NodeOutput();
				nodeOutput->pid = node.id;
				nodeOutput->name = outputJson[k]["name"].asString();
				nodeOutput->id = outputJson[k]["id"].asString();
				nodeOutput->dataType = outputJson[k]["dataType"].asInt();
				nodeOutput->targetSize = 0;

				//����map�У����ڶ�Ӧinput��resource
				nodeOutputMap.insert(make_pair(nodeOutput->id, nodeOutput));
				node.outputs[k] = nodeOutput;

				 
			}
			node.outputSize = outputSize;

			//������ʼ��
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
			cout << DevStr << ":ģ���ʼ���ɹ�" << endl;
		}
		else {
			sendMsg(STREAM_FAIL,DevStr+string(":ģ���ʼ��ʧ��"));
			cout << DevStr << ":ģ���ʼ��ʧ��" << endl; 
			//return -1;
		}
		cout << "----------------" << endl;
	}

	//��������ڵ㣬��������Դ������������ڵ�
	map<string, NodeInput*>::iterator inputIter;
	for (inputIter = nodeInputMap.begin(); inputIter != nodeInputMap.end(); inputIter++)
	{
		NodeInput *input = inputIter->second;
		string resourceId = input->resourceId;
		NodeOutput *outPut = nodeOutputMap[resourceId];
		outPut->targetSize++;

		input->resource = outPut;
	}

	//��������ڵ㣬��ʼ������������ڵ�����
	map<string, NodeOutput*>::iterator outputIter;
	for (outputIter = nodeOutputMap.begin(); outputIter != nodeOutputMap.end(); outputIter++)
	{
		NodeOutput *output = outputIter->second; 
		output->targets = new NodeInput[output->targetSize]();
	 
	}

	//�ٱ���һ������ڵ㣬������ڵ�ŵ�����Դ���ڵ�����ڵ�ĩβ
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
