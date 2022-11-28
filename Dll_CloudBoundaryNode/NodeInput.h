#pragma once 
#include "string"  
#include "NodeOutput.h"

using namespace std;

 
class NodeInput
{
public:
	NodeInput(void);
	virtual ~NodeInput(void);

	//�ڵ���
	string pid;

	//���
	string id;

	//����ԴID
	string resourceId;

	//����
	string name;

	//��������
	// POINT_CLOUD = 1,
	// PLANE = 2,
	// LINE = 3,
	// POINT = 4
	int dataType;

	//����Դ
	NodeOutput* resource;
  
};

