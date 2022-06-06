#include "StdAfx.h"
#include<windows.h>  
#include "AlgoStream.h"  
#include "AlgoNode.h"
#include<iostream>
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
#include "web_sock_server.h"

using namespace pcl;
using namespace std;
map<string, RUN_FUN> mFuncPtr;

typedef pcl::PointXYZ PointT;

//�������ֻ�ȡ����
RUN_FUN getFunction(string funcName)
{
	map<string, RUN_FUN>::iterator it = mFuncPtr.find(funcName);

	if (it != mFuncPtr.end())
		return it->second;
	return 0;
}
//��ȡ��Ŀ¼�������ļ�����������չ������
void getFiles(string path, string exd, vector<string>& files)
{
	//cout << "getFiles()" << path<< endl; 
	//�ļ����
	long  hFile = 0;
	//�ļ���Ϣ
	struct _finddata_t fileinfo;
	string pathName, exdName;

	if (0 != strcmp(exd.c_str(), ""))
	{
		exdName = "\\algo1*." + exd;
	}
	else
	{
		exdName = "\\algo1*";
	}

	if ((hFile = _findfirst(pathName.assign(path).append(exdName).c_str(), &fileinfo)) != -1)
	{
		do
		{
			//cout << fileinfo.name << endl; 

			//������ļ����������ļ���,����֮
			//�������,�����б�
			if ((fileinfo.attrib & _A_SUBDIR))
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					getFiles(pathName.assign(path).append("\\").append(fileinfo.name), exd, files);
			}
			else
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					files.push_back(pathName.assign(path).append("\\").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}
//��ȡ�ļ�������
void get_FileBaseName(std::string path, std::string &name)
{
	for (int i = path.size() - 1; i > 0; i--)
	{
		if (path[i] == '\\' || path[i] == '/')
		{
			name = path.substr(i + 1);
			return;
		}
	}
	name = path;
}



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

	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile("table.pcd", *cloud);
	std::cout << "original cloud size : " << cloud->size() << std::endl;
	pcl::PointCloud<PointT>::Ptr res(new pcl::PointCloud<PointT>); 
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
				float balance[2] = { 0.1,0.1 };
				cout << "�����������ݴ�С:" << cloud->size() << endl;
				fp1(cloud, res, balance);
				cout << "�������Խ����С:" << res->size() << endl;


				mFuncPtr.insert(make_pair(method, fp1));
				cout << "����ע��ɹ�:" << method << endl;

			}
		}
		cout << "----------------" << endl;
	}
	cout << "���ҽ���!" << endl;
	cout << "----------------" << endl;
}


string getOutPath(string uuid, string node) {
	return  uuid + "_" + node + ".ply";
}
 

void AlgoStream::sendNodeRes(AlgoNode node) {
	Json::Value root;
	root["uuid"] = Json::Value(uuid);
	root["node"] = Json::Value(node.name);
	root["outPath"] = Json::Value(node.outPath);
	Json::FastWriter fw;
	cout << "node res:" << fw.write(root) << endl;

	//��socket���
	if (clientWs != NULL) {
		WebSockServer::Instance().Send(clientWs, fw.write(root), WsOpcode::TEXT);
	}
	return ;
}

//��ʾ
void visualization_point(PointCloud<PointXYZ>::Ptr &raw_point, PointCloud<PointXYZ>::Ptr &sor_cloud
	//, PointCloud<PointXYZ>::Ptr &voxel, PointCloud<PointXYZ>::Ptr &uniform
) {
	visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer("3d viewer"));
	int v1(0), v2(0), v3(0), v4(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->addPointCloud(raw_point, "cloud1", v1);


	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->addPointCloud(sor_cloud, "cloud2", v2);


	/*viewer->createViewPort(0.5, 0.0, 0.75, 1.0, v3);
	viewer->addPointCloud(voxel, "cloud3", v3);

	viewer->createViewPort(0.75, 0.0, 1, 1.0, v3);
	viewer->addPointCloud(uniform, "cloud4", v3);*/

	viewer->spin();
}

void AlgoStream::start(pcl::PointCloud<PointT>::Ptr input){
	printf("�������ݴ�С:%d\r\n", input->size());
	for(int i=0;i<size;i++){ 
		//����ǰ�ô�����

		algos[i].input = input;

		//����ĵ���
		pcl::PointCloud<PointT>::Ptr res(new pcl::PointCloud<PointT>);
		algos[i].out = res;
		algos[i].outPath = getOutPath(uuid, algos[i].name);

		//���ú���fun1 
		algos[i].runAddr(input, res, algos[i].params);

		//���ply
		pcl::io::savePLYFileBinary(algos[i].outPath, *res);

		//������
		printf("RUN_FUN:%s,res:%d,%d\r\n", algos[i].name.c_str(), input->size(), res->size());

		//���������ڵ���
		sendNodeRes(algos[i]);

		//����Ԥ��
		visualization_point(input, res); 

		//�¸��ڵ������Ϊ���ε����
		input = res;

		//���ú��ô�����
	}
}


void AlgoStream::init(string doc) {

	Json::Reader reader;
	Json::Value root;

	if (!reader.parse(doc, root, false)) {
		printf("failed to parse!\n");
		return ;
	}

	int siNum = root.size();
	cout << "�ű���" << siNum << "���㷨ģ��" << endl;
	cout << "----------------" << endl;
	size = siNum;
	algos = new AlgoNode[siNum]();
	for (int i = 0; i < siNum; i++)
	{
		Json::Value DevJson = root[i];
		std::string DevStr = DevJson["method"].asString();
		cout << "��ʼ��ʼ����" << DevStr << "���㷨ģ��" << endl;
		int asParamSize = DevJson["paramSize"].asInt();
		int paramSize = DevJson["params"].size();
		float* params = new float[paramSize]();
		for (int j = 0; j < paramSize; j++) {
			params[j] = DevJson["params"][j].asFloat();
		}


		//��ȡ����
		RUN_FUN fptr = getFunction(DevStr.c_str());

		if (fptr != 0)
		{
			//��ʼ���㷨�ڵ�
			AlgoNode node;
			//node.id = strcat("00", i+"");
			node.name = DevStr.c_str();
			node.paramSize = asParamSize;
			node.runAddr = fptr;
			node.params = params;
			algos[i] = node;

			cout << DevStr << ":ģ���ʼ���ɹ�" << endl;
		}
		else {
			cout << DevStr << ":ģ���ʼ��ʧ��" << endl; 
			return ;
		}
		cout << "----------------" << endl;
	}

	return;

	
}



