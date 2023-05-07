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

	//�ڵ����������±�
	int pIdx;

	//���
	string id;

	//����ԴID
	string resourceId;

	//����
	string name;

	//��������
	//���� = 1,
	// �� = 2,
	// �� = 3,
	// �� = 4
	//�߶� = 5
	//Բ�� = 6
	//���ͼ = 7
	int dataType;

	//����Դ
	NodeOutput* resource;
  
};

