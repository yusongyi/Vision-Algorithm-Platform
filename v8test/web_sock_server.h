#pragma once 
#include "StdAfx.h"
#include <string>
#include <boost/thread.hpp>
#include <boost/function.hpp>


#if defined(_WIN32) || defined(_WIN64)
#  define WEB_SOCKET_DLL_EXPORT __declspec(dllexport)
#else
#  define WEB_SOCKET_DLL_EXPORT
#endif

enum WsOpcode
{
	BINARY = 2,
	
	TEXT= 1,
	OTHER
};

//����ص�����
typedef boost::function<void(void*, const std::string, WsOpcode)> OnMessageFun;//������Ϣ�����ĺ���
typedef boost::function<void(void*)> OnOpenFun;                      //���ӵ���
typedef boost::function<void(void*, std::string)> OnCloseFun;        //���ӶϿ��ĺ���

//�����ķ�����
class WEB_SOCKET_DLL_EXPORT WebSockServer
{
private:
	WebSockServer();
public:
	static WebSockServer& Instance();
	bool Init(uint16_t uPort, OnOpenFun openFun, OnCloseFun closeFun, OnMessageFun msgFun, void* pOiService = nullptr);
	/*
	�������StartServer,Ӧ����һ���߳������ã���Ϊ�����������ѭ�����У����᷵�أ�ֱ����һ���̵߳���StopServer����
	*/
	bool StartServer();
	void StopServer();
	void StopListening();
	/*
	�������ݵĽӿ�
	pClientΪ�ͻ��˱�ʶ
	data ΪҪ���͵�����
	*/
	bool Send(void* pClient, const std::string data);
	bool Send(void* pClient, const void* data, int size);
	void Close(void* pClient);
	void CloseAll();
private:
	void ThreadProccess();               //�̺߳���

public:
	OnOpenFun       m_onOpenFun;
	OnCloseFun      m_onCloseFun;
	OnMessageFun    m_onMsgFun;

private:
	boost::thread * m_threadMain;              //websock ��ѭ���߳�
	bool            m_bServerStart;            //�����Ƿ������ı�־
};

