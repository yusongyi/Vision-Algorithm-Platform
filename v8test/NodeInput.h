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

	//节点所在数组下标
	int pIdx;

	//编号
	string id;

	//输入源ID
	string resourceId;

	//名称
	string name;

	//数据类型
	//点云 = 1,
	// 面 = 2,
	// 线 = 3,
	// 点 = 4
	//线段 = 5
	//圆弧 = 6
	//深度图 = 7
	int dataType;

	//输入源
	NodeOutput* resource;
  
};

