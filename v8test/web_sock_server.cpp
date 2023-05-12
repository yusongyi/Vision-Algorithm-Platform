
#include "StdAfx.h"
#include "web_sock_server.h"
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <map>
typedef websocketpp::server<websocketpp::config::asio> server;

using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;

typedef server::message_ptr message_ptr;
typedef std::map<void*, websocketpp::connection_hdl> ClientMap;

static server g_server;//ȫ�ֵ�websocket������

//�ݽ����ϲ�Ĳ���ֱ�ӵݽ�websocketpp::connection_hdl���ͣ���Ϊ�ϲ�û��������͵Ķ��壬�������������ͣ�void*����Ψһ��ʶһ���ͻ���
//�����ڱ���ά��һ���ͻ��˱�ʶ����Ӧʵ�ʿͻ��˶����ӳ�䣬
static ClientMap g_mapClient;
static boost::shared_mutex rwMutext;
typedef boost::shared_lock<boost::shared_mutex> ReadLock;
typedef boost::unique_lock<boost::shared_mutex> WriteLock;

// Define a callback to handle incoming messages
static void on_message(WebSockServer* pWebserver, websocketpp::connection_hdl hdl, message_ptr msg) {
	//std::cout << "on_message called with hdl: " << hdl.lock().get()
	//          << " and message: " << msg->get_payload()
	//          << std::endl;
	//�ص��ϲ�
	if (!pWebserver->m_onMsgFun.empty())
	{
		WsOpcode opcode = WsOpcode::OTHER;
		if (msg->get_opcode() == websocketpp::frame::opcode::value::TEXT)
		{
			opcode = WsOpcode::TEXT;
		}
		else if (msg->get_opcode() == websocketpp::frame::opcode::value::BINARY)
		{
			opcode = WsOpcode::BINARY;
		}
		pWebserver->m_onMsgFun(hdl.lock().get(), msg->get_payload(), opcode);
	}
}

static void on_open(WebSockServer* pWebserver, websocketpp::connection_hdl hdl)
{
	//std::cout << "client opend:" << hdl.lock().get() << std::endl;

	//д��
	WriteLock writeLock(rwMutext);

	//���뻺��
	g_mapClient.insert(std::pair<void*, websocketpp::connection_hdl>(hdl.lock().get(), hdl));

	//�ص��ϲ�
	if (!pWebserver->m_onOpenFun.empty())
	{
		pWebserver->m_onOpenFun(hdl.lock().get());
	}
}

static void on_close(WebSockServer* pWebserver, websocketpp::connection_hdl hdl)
{
	//std::cout << "client close:" << hdl.lock().get() << std::endl;
	//�ص��ϲ�
	if (!pWebserver->m_onCloseFun.empty())
	{
		pWebserver->m_onCloseFun(hdl.lock().get(), "");
	}

	//д��
	WriteLock writeLock(rwMutext);

	const ClientMap::iterator it = g_mapClient.find(hdl.lock().get());
	//ɾ������
	if (it != g_mapClient.end())
	{
		g_mapClient.erase(it);
		return;
	}
}

WebSockServer::WebSockServer()
{
	m_bServerStart = false;
	m_threadMain = nullptr;
}

WebSockServer& WebSockServer::Instance()
{
	static WebSockServer instance;
	return instance;
}
bool WebSockServer::Send(void* pClient, const void* data,int size, WsOpcode opcode)
{
	//����
	ReadLock readLock(rwMutext);
	//���ڱ��ز��Ҷ�Ӧ�Ŀͻ��˶���
	const ClientMap::iterator it = g_mapClient.find(pClient);

	if (it == g_mapClient.end())
	{
		return false;
	}

	websocketpp::connection_hdl hdl = it->second;
	std::error_code ec;

	websocketpp::frame::opcode::value sCode = websocketpp::frame::opcode::BINARY;

	g_server.send(hdl, data, size, sCode, ec);//���Ͷ���������

	//��������Ϣ
	if (ec.value() == 0)
	{
		return true;
	}
	else
	{
		std::cout << "> Error sending message: " << ec.message() << std::endl;
		return false;
	}
}
bool WebSockServer::Send(void* pClient, const std::string data, WsOpcode opcode)
{
	//����
	ReadLock readLock(rwMutext);
	//���ڱ��ز��Ҷ�Ӧ�Ŀͻ��˶���
	const ClientMap::iterator it = g_mapClient.find(pClient);

	if (it == g_mapClient.end())
	{
		return false;
	}

	websocketpp::connection_hdl hdl = it->second;
	std::error_code ec;

	websocketpp::frame::opcode::value sCode = websocketpp::frame::opcode::BINARY;

	if (opcode == TEXT)
	{
		sCode = websocketpp::frame::opcode::TEXT;
	}
	g_server.send(hdl, data.c_str(), data.size(), sCode, ec);//���Ͷ���������

	//��������Ϣ
	if (ec.value() == 0)
	{
		return true;
	}
	else
	{
		std::cout << "> Error sending message: " << ec.message() << std::endl;
		return false;
	}
}

void WebSockServer::Close(void* pClient)
{
	//д��
	WriteLock writeLock(rwMutext);

	//���ڱ��ز��Ҷ�Ӧ�Ŀͻ��˶���
	const ClientMap::iterator it = g_mapClient.find(pClient);

	if (it == g_mapClient.end())
	{
		//std::cout << "can not find client. " << (int)pClient << std::endl;
		return;
	}
	websocketpp::connection_hdl hdl = it->second;
	g_server.close(hdl, (websocketpp::close::status::value)0, "");

	//��ӳ����ɾ��
	g_mapClient.erase(it);
}

void WebSockServer::CloseAll()
{
	//д��
	WriteLock writeLock(rwMutext);
	for (auto it = g_mapClient.begin(); it != g_mapClient.end(); it++)
	{
		//�ر���������
		websocketpp::connection_hdl hdl = it->second;
		g_server.close(hdl, (websocketpp::close::status::value)0, "");
	}

	//��ջ���
	g_mapClient.clear();
}

bool WebSockServer::Init(uint16_t uPort, OnOpenFun openFun, OnCloseFun closeFun, OnMessageFun msgFun, void* pOiService)
{
	m_onOpenFun = openFun;
	m_onCloseFun = closeFun;
	m_onMsgFun = msgFun;

	try {
		// Set logging settings
		g_server.set_access_channels(websocketpp::log::alevel::none);
		g_server.clear_access_channels(websocketpp::log::alevel::none);

		// Initialize Asio
		if (pOiService == nullptr)
		{
			g_server.init_asio();
		}
		else
		{
			g_server.init_asio((boost::asio::io_service*)pOiService);
		}

		// Register our message handler
		g_server.set_message_handler(bind(&on_message, this, ::_1, ::_2));
		g_server.set_close_handler(bind(&on_close, this, ::_1));
		g_server.set_open_handler(bind(&on_open, this, ::_1));

		g_server.set_reuse_addr(true);
		// Listen port
		g_server.listen(websocketpp::lib::asio::ip::tcp::v4(), uPort);

		// Start the server accept loop
		g_server.start_accept();
	}
	catch (websocketpp::exception const & e) {
		std::cout << e.what() << std::endl;
		return false;
	}
	catch (...) {
		//std::cout << "other exception" << std::endl;
		return false;
	}

	return true;
}

bool WebSockServer::StartServer()
{
	if (!m_bServerStart)
	{
		//�¿���һ���߳�
		m_threadMain = new boost::thread(boost::bind(&WebSockServer::ThreadProccess, this));
		if (nullptr == m_threadMain)
		{
			return false;
		}
		else
		{
			m_bServerStart = true;
			return true;
		}
	}
	else
	{
		std::cout << "server already start." << std::endl;
		return true;
	}
}

//bool WebSockServer::StartServer()
//{
//	if (!m_bServerStart)
//	{
//		m_bServerStart = true;
//		g_server.run();
//	}
//
//	return true;
//}

void WebSockServer::StopServer()
{
	g_server.stop();

	//�ȴ��߳̽���
	//m_threadmain->join();
	//delete m_threadmain;

	//m_threadmain = nullptr;
	m_bServerStart = false;
}

void WebSockServer::StopListening()
{
	g_server.stop_listening();
}

void WebSockServer::ThreadProccess()
{
	g_server.run();//websock�������ڴ�ѭ����ֱ������g_server.stop()��������ѭ��
	//while (true)
	//{
	//	g_server.poll_one();
	//	Sleep(100);
	//}
}

