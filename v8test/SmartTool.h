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


string getSuffix(string filename);
void readPointCloud(string filename, pcl::PointCloud<PointT>::Ptr cloud);
std::string GBKToUTF8(const std::string& strGBK);

std::string UTF8ToGBK(const std::string& strUTF8);

//显示
void visualization_point(PointCloud<PointXYZ>::Ptr &raw_point, PointCloud<PointXYZ>::Ptr &sor_cloud
	//, PointCloud<PointXYZ>::Ptr &voxel, PointCloud<PointXYZ>::Ptr &uniform
);

