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

//线程暂停
void mySleep(int s);

//根据名字获取函数
RUN_FUN getFunction(string funcName);

//获取此目录下所有文件，并根据扩展名过滤
void getFiles(string path, string exd, vector<string>& files);

//获取文件的名称
void get_FileBaseName(std::string path, std::string &name);

//获取文件后缀名
string getSuffix(string filename);

//从文件中读取点云数据
void readPointCloud(string filename, pcl::PointCloud<PointT>::Ptr cloud);

//编码转换GBKToUTF8
std::string GBKToUTF8(const std::string& strGBK);

//编码转换UTF8ToGBK
std::string UTF8ToGBK(const std::string& strUTF8);

//预览点云数据
void visualization_point(PointCloud<PointXYZ>::Ptr &raw_point, NodeOutput** outputs, int outSize);


//读取文件成字符串
std::string readConfig(const char* path);

//字符编码转换
std::wstring string_to_wstring(const std::string &s);

//字符编码转换
std::string wstring_to_string(const std::wstring &s);

//字符编码转换
std::string ansi_to_utf8(const std::string &s);

//字符编码转换
std::string utf8_to_ansi(const std::string& s);
 
