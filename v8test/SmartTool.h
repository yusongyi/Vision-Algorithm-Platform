#pragma once
#include "StdAfx.h"
#include<windows.h>  
#include <ShellAPI.h> 
#include<iostream>
#include <thread>
#include <time.h>
#include "string"
#include <map>
#include "json/json.h"
#include <direct.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h> 
#include <liblas/liblas.hpp> 
#include "Thread.h"
#include "AlgoStream.h"  
#include "AlgoNode.h"

using namespace pcl;
using namespace std;
//�߳���ͣ
void mySleep(int s);

//�������ֻ�ȡ����
RUN_FUN getFunction(string funcName);
//��ȡ��Ŀ¼�������ļ�����������չ������
void getFiles(string path, string exd, vector<string>& files);
//��ȡ�ļ�������
void get_FileBaseName(std::string path, std::string &name);


string getSuffix(string filename);
void readPointCloud(string filename, pcl::PointCloud<PointT>::Ptr cloud);
std::string GBKToUTF8(const std::string& strGBK);

std::string UTF8ToGBK(const std::string& strUTF8);

//��ʾ
void visualization_point(PointCloud<PointXYZ>::Ptr &raw_point, PointCloud<PointXYZ>::Ptr &sor_cloud
	//, PointCloud<PointXYZ>::Ptr &voxel, PointCloud<PointXYZ>::Ptr &uniform
);

