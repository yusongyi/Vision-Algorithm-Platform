#pragma once  
#include <map> 
#include "AlgoNode.h"
#include "json/json.h"
#include <string>
using namespace std;

enum StreamOpcode
{

	//�㷨��ʼ
	STREAM_START  = 1,

	//�ڵ�������
	STREAM_DOING = 2,

	//�㷨����
	STREAM_END    = 3, 

	//�ڵ㿪ʼ
	NODE_START = 4,

	//�����������
	SEND_CLOUD = 5,

	//�㷨ִ��ʧ��
	STREAM_FAIL  = 9,

};
enum MsgType
{

	//�㷨���й�����״̬��Ϣ
	STREAM_STATE = 1,

	//�ڵ�������
	DEPTH_IMG = 2,

	//��������
	POINT_CLOUD = 3, 

};

//�㷨��������
class AlgoStream
{
	
public:  

	//�Ƿ�ִ���㷨
	bool running = false;

	//ִ�����ͣ����Ի�����
	int type;

	//�㷨�ڵ�����
	int size;

	//��ǰǰ��չʾ�ĵ���ID,���ں��ִ��������������ܵ���websockt���������
	int curShowIdx;

	//�㷨�ڵ�
	AlgoNode* algos; 

	//��ʼ�ڵ�
	AlgoNode firstNode;

	//�ͻ���websocket
	void* clientWs;

	//�������е�ID
	string uuid;

	//���͵��Ƶ�ID
	int cloudId;

	//����ϵͳ����
	void loadConfig(Json::Value config);

	//�����㷨DLL�ļ�
	void loadDll();

	//��ʼ���㷨����
	int init(Json::Value doc);

	//��ʼִ���㷨
	void start();

	//���ͽڵ�������
	void sendNodeRes(AlgoNode node); 

	//������Ϣ
	void sendMsg(StreamOpcode type, string msg);

	//���������������
	void sendCloudData(pcl::PointCloud<PointT>::Ptr cloud, int cloudId); 
	 
};
