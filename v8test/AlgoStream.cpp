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

//�洢�����㷨�ĵ�ַ
map<string, RUN_FUN> mFuncPtr;

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

//��ȡply�����ļ������·������·��/uuid_�㷨����.ply
string getOutPath(string uuid, string node) {
	return AlgoStream::ROOT_PATH+uuid + "_" + node + ".ply";
}

/*

����PotreeConverter����Potree�ļ�
	stream: �㷨���̣����ڻ�ȡuuid
	node  : �㷨�ڵ㣬��Ҫ�ڵ����ơ��˽ڵ�����ɵ����ļ�

*/
void genPotreeFiles(AlgoStream stream, AlgoNode *node) {
	node->potreePath = "potree_res\\"+stream.uuid + "_" + node->name;
	string params = node->outPath + " -o " + node->potreePath;

	ShellExecuteA(NULL, "open", CONVERT_TOOL_PATH.c_str(), params.c_str(), AlgoStream::ROOT_PATH.c_str(), SW_SHOW);
}
/*

������ĵ������������ɵ����ļ�����ת��ΪPotree�ļ�
	cloud : ��������
	stream: �㷨���̣����ڻ�ȡuuid
	node  : �㷨�ڵ㣬��Ҫ�ڵ����ơ��˽ڵ�����ɵ����ļ�

*/
void genPotreeFiles(pcl::PointCloud<PointT>::Ptr cloud, AlgoStream stream, AlgoNode *node) {
	//���ply
	pcl::io::savePLYFileBinary(node->outPath, *cloud);
	genPotreeFiles(stream, node);
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
	root["outPath"] = Json::Value(node.potreePath);

	if (node.conditionType != -1) {
		root["conditionCount"] = Json::Value(node.conditionCount);
		root["conditionNext"] = Json::Value(node.conditionNext);
	}

	Json::FastWriter fw;
	sendMsg(STREAM_DOING,fw.write(root));
}


void AlgoStream::start(){

	printf("�������ݴ�С:%d\r\n", input->size());

	//ת�������ļ�
	genPotreeFiles(*this, &fileNode);

	//��ʼʱ���ͳ�ʼ��Potree�ļ�
	Json::Value root; 
	root["uuid"] = Json::Value(uuid);
	root["id"] = Json::Value(fileNode.id);
	root["chName"] = Json::Value(fileNode.chName);
	root["outPath"] = Json::Value(fileNode.potreePath);
	Json::FastWriter fw; 
	sendMsg(STREAM_START, fw.write(root));

	//��ʾģʽ��ͣ
	if (type == 2) {
		mySleep(DEMO_SLEEP);
	}

	for(int i=0;i<size;i++){ 
		algos[i].input = input; 

		//����ĵ����ļ���ַ
		algos[i].outPath = getOutPath(uuid, algos[i].name); 

		pcl::PointCloud<PointT>::Ptr res(new pcl::PointCloud<PointT>);
		algos[i].out = res;
 
		do { 
			//���ú���fun1 
			algos[i].runAddr(input, algos[i].out, algos[i].params);

			//����Potree�ļ�
			genPotreeFiles(algos[i].out, *this, &algos[i]);

			//��֧�ṹ����м���
			if (algos[i].conditionType != -1) {
				float remain = algos[i].out->size() / (float)(algos[i].input->size());

				printf("BRANCH_RES: node:%s,input size:%d,out size:%d,remain:%f \r\n", algos[i].name.c_str(), algos[i].input->size(), algos[i].out->size(), remain);

				algos[i].conditionCount++;

				//�ж�ʣ����������Ƿ��������
				algos[i].conditionNext = (
					(algos[i].conditionType == 0 && remain <= algos[i].conditionParams[0]) ||
					(algos[i].conditionType == 1 && remain >= algos[i].conditionParams[0]) ||
					(algos[i].conditionType == 2 && remain != algos[i].conditionParams[0])
					)
					&&
					algos[i].conditionCount < 99;
			}
			else {
				//������
				printf("NODE_RES: node:%s,input size:%d,out size:%d \r\n", algos[i].name.c_str(), algos[i].input->size(), algos[i].out->size());

				//���Ƿ�֧�ṹ��Ĭ�Ͻ�ִ��һ��
				algos[i].conditionNext = false;
			}

			//���������ڵ���
			sendNodeRes(algos[i]);

			//����Ԥ��
			if (type == 1) {
				visualization_point(input, algos[i].out);
			}

			//��ʾģʽ��ͣ
			if (type == 2) {
				mySleep(DEMO_SLEEP);
			}

			//�¸��ڵ������Ϊ���ε����
			input = algos[i].out;
		} while (algos[i].conditionNext); 
	}
	sendMsg(STREAM_END,uuid);
}


int AlgoStream::init(Json::Value doc) {
 
	//Ĭ�ϵ�һ���ڵ�ĵ�һ������Ϊ�����ļ���ַ
	string dataPath = doc[0]["params"][0].asString();


	//��ʼ�ļ�����ڵ�
	fileNode.name = "inputFile"; 
	fileNode.chName = doc[0]["chName"].asString();
	fileNode.id = doc[0]["nodeId"].asString();
	fileNode.outPath = AlgoStream::ROOT_PATH + dataPath;

	//��ȡ�����ļ���input��
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	readPointCloud(fileNode.outPath, cloud);
	input = cloud;

	//��ʼ���㷨ģ��ǰ��Ҫ����һ���ڵ�ȥ�� 
	doc.removeIndex(0,nullptr);
	 

	//��ʼ���㷨ģ��
	int siNum = doc.size();
	cout << "�ű���" << siNum << "���㷨ģ��" << endl;
	size = siNum;
	algos = new AlgoNode[siNum]();
	for (int i = 0; i < siNum; i++)
	{
		Json::Value DevJson = doc[i];
		std::string DevStr = DevJson["method"].asString();
		cout << "��ʼ��ʼ����" << DevStr << "���㷨ģ��" << endl;
		 
		 
		//��ȡ����
		RUN_FUN fptr = getFunction(DevStr.c_str());

		if (fptr != 0)
		{
			//��ʼ���㷨�ڵ�
			AlgoNode node;
			node.id = DevJson["nodeId"].asString();
			node.chName = DevJson["chName"].asString();
			node.name = DevStr.c_str();
			node.runAddr = fptr;

			//������ʼ��
			if (DevJson["params"].type() != Json::nullValue) { 
				int paramSize = DevJson["params"].size();
				float* params = new float[paramSize]();
				for (int j = 0; j < paramSize; j++) {
					params[j] = DevJson["params"][j].asFloat();
				}
				node.paramSize = paramSize;
				node.params = params;
			}

			//��֧������ʼ��
			if (DevJson["conditionType"].type() != Json::nullValue) {
				node.conditionType = DevJson["conditionType"].asInt();

				if (DevJson["conditionParams"].type() == Json::nullValue) {
					cout << DevStr << ":ģ���ʼ��ʧ��" << endl;
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
			cout << DevStr << ":ģ���ʼ���ɹ�" << endl;
		}
		else {
			sendMsg(STREAM_FAIL,DevStr+string(":ģ���ʼ��ʧ��"));
			cout << DevStr << ":ģ���ʼ��ʧ��" << endl; 
			//return -1;
		}
		cout << "----------------" << endl;
	}

	return 0;

	
}
