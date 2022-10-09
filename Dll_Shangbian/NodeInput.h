#pragma once 
#include "string"  
#include "NodeOutput.h"

using namespace std;

 
class NodeInput
{
public:
	NodeInput(void);
	virtual ~NodeInput(void);

	//节点编号
	string pid;

	//编号
	string id;

	//输入源ID
	string resourceId;

	//名称
	string name;

	//数据类型
	// POINT_CLOUD = 1,
	// PLANE = 2,
	// LINE = 3,
	// POINT = 4
	int dataType;

	//输入源
	NodeOutput* resource;
  
};

