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

//根据名字获取函数
RUN_FUN getFunction(string funcName)
{
	map<string, RUN_FUN>::iterator it = mFuncPtr.find(funcName);

	if (it != mFuncPtr.end())
		return it->second;
	return 0;
}
//获取此目录下所有文件，并根据扩展名过滤
void getFiles(string path, string exd, vector<string>& files)
{
	//cout << "getFiles()" << path<< endl; 
	//文件句柄
	long  hFile = 0;
	//文件信息
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

			//如果是文件夹中仍有文件夹,迭代之
			//如果不是,加入列表
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
//获取文件的名称
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

	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile("table.pcd", *cloud);
	std::cout << "original cloud size : " << cloud->size() << std::endl;
	pcl::PointCloud<PointT>::Ptr res(new pcl::PointCloud<PointT>); 
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
				float balance[2] = { 0.1,0.1 };
				cout << "函数测试数据大小:" << cloud->size() << endl;
				fp1(cloud, res, balance);
				cout << "函数测试结果大小:" << res->size() << endl;


				mFuncPtr.insert(make_pair(method, fp1));
				cout << "函数注册成功:" << method << endl;

			}
		}
		cout << "----------------" << endl;
	}
	cout << "查找结束!" << endl;
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

	//向socket输出
	if (clientWs != NULL) {
		WebSockServer::Instance().Send(clientWs, fw.write(root), WsOpcode::TEXT);
	}
	return ;
}

//显示
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
	printf("输入数据大小:%d\r\n", input->size());
	for(int i=0;i<size;i++){ 
		//调用前置处理函数

		algos[i].input = input;

		//输出的点云
		pcl::PointCloud<PointT>::Ptr res(new pcl::PointCloud<PointT>);
		algos[i].out = res;
		algos[i].outPath = getOutPath(uuid, algos[i].name);

		//调用函数fun1 
		algos[i].runAddr(input, res, algos[i].params);

		//输出ply
		pcl::io::savePLYFileBinary(algos[i].outPath, *res);

		//输出结果
		printf("RUN_FUN:%s,res:%d,%d\r\n", algos[i].name.c_str(), input->size(), res->size());

		//输出本计算节点结果
		sendNodeRes(algos[i]);

		//本机预览
		visualization_point(input, res); 

		//下个节点的输入为本次的输出
		input = res;

		//调用后置处理函数
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
	cout << "脚本共" << siNum << "个算法模块" << endl;
	cout << "----------------" << endl;
	size = siNum;
	algos = new AlgoNode[siNum]();
	for (int i = 0; i < siNum; i++)
	{
		Json::Value DevJson = root[i];
		std::string DevStr = DevJson["method"].asString();
		cout << "开始初始化‘" << DevStr << "’算法模块" << endl;
		int asParamSize = DevJson["paramSize"].asInt();
		int paramSize = DevJson["params"].size();
		float* params = new float[paramSize]();
		for (int j = 0; j < paramSize; j++) {
			params[j] = DevJson["params"][j].asFloat();
		}


		//获取函数
		RUN_FUN fptr = getFunction(DevStr.c_str());

		if (fptr != 0)
		{
			//初始化算法节点
			AlgoNode node;
			//node.id = strcat("00", i+"");
			node.name = DevStr.c_str();
			node.paramSize = asParamSize;
			node.runAddr = fptr;
			node.params = params;
			algos[i] = node;

			cout << DevStr << ":模块初始化成功" << endl;
		}
		else {
			cout << DevStr << ":模块初始化失败" << endl; 
			return ;
		}
		cout << "----------------" << endl;
	}

	return;

	
}



