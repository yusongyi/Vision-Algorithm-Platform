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

//��ȡ�ļ���׺��
string getSuffix(string filename);

//���ļ��ж�ȡ��������
void readPointCloud(string filename, pcl::PointCloud<PointT>::Ptr cloud);

//����ת��GBKToUTF8
std::string GBKToUTF8(const std::string& strGBK);

//����ת��UTF8ToGBK
std::string UTF8ToGBK(const std::string& strUTF8);

//Ԥ����������
void visualization_point(PointCloud<PointXYZ>::Ptr &raw_point, NodeOutput** outputs, int outSize);


//��ȡ�ļ����ַ���
std::string readConfig(const char* path);

//�ַ�����ת��
std::wstring string_to_wstring(const std::string &s);

//�ַ�����ת��
std::string wstring_to_string(const std::wstring &s);

//�ַ�����ת��
std::string ansi_to_utf8(const std::string &s);

//�ַ�����ת��
std::string utf8_to_ansi(const std::string& s);
 
