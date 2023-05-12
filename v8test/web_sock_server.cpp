
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

static server g_server;//全局的websocket服务器

//递交给上层的不能直接递交websocketpp::connection_hdl类型，因为上层没有这个类型的定义，因此用另外的类型（void*）来唯一标识一个客户端
//并且在本地维护一个客户端标识到对应实际客户端对象的映射，
static ClientMap g_mapClient;
static boost::shared_mutex rwMutext;
typedef boost::shared_lock<boost::shared_mutex> ReadLock;
typedef boost::unique_lock<boost::shared_mutex> WriteLock;

// Define a callback to handle incoming messages
static void on_message(WebSockServer* pWebserver, websocketpp::connection_hdl hdl, message_ptr msg) {
	//std::cout << "on_message called with hdl: " << hdl.lock().get()
	//          << " and message: " << msg->get_payload()
	//          << std::endl;
	//回调上层
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

	//写锁
	WriteLock writeLock(rwMutext);

	//加入缓存
	g_mapClient.insert(std::pair<void*, websocketpp::connection_hdl>(hdl.lock().get(), hdl));

	//回调上层
	if (!pWebserver->m_onOpenFun.empty())
	{
		pWebserver->m_onOpenFun(hdl.lock().get());
	}
}

static void on_close(WebSockServer* pWebserver, websocketpp::connection_hdl hdl)
{
	//std::cout << "client close:" << hdl.lock().get() << std::endl;
	//回调上层
	if (!pWebserver->m_onCloseFun.empty())
	{
		pWebserver->m_onCloseFun(hdl.lock().get(), "");
	}

	//写锁
	WriteLock writeLock(rwMutext);

	const ClientMap::iterator it = g_mapClient.find(hdl.lock().get());
	//删除缓存
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
	//读锁
	ReadLock readLock(rwMutext);
	//先在本地查找对应的客户端对象
	const ClientMap::iterator it = g_mapClient.find(pClient);

	if (it == g_mapClient.end())
	{
		return false;
	}

	websocketpp::connection_hdl hdl = it->second;
	std::error_code ec;

	websocketpp::frame::opcode::value sCode = websocketpp::frame::opcode::BINARY;

	g_server.send(hdl, data, size, sCode, ec);//发送二进制数据

	//检查错误信息
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
	//读锁
	ReadLock readLock(rwMutext);
	//先在本地查找对应的客户端对象
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
	g_server.send(hdl, data.c_str(), data.size(), sCode, ec);//发送二进制数据

	//检查错误信息
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
	//写锁
	WriteLock writeLock(rwMutext);

	//先在本地查找对应的客户端对象
	const ClientMap::iterator it = g_mapClient.find(pClient);

	if (it == g_mapClient.end())
	{
		//std::cout << "can not find client. " << (int)pClient << std::endl;
		return;
	}
	websocketpp::connection_hdl hdl = it->second;
	g_server.close(hdl, (websocketpp::close::status::value)0, "");

	//从映射中删除
	g_mapClient.erase(it);
}

void WebSockServer::CloseAll()
{
	//写锁
	WriteLock writeLock(rwMutext);
	for (auto it = g_mapClient.begin(); it != g_mapClient.end(); it++)
	{
		//关闭所有连接
		websocketpp::connection_hdl hdl = it->second;
		g_server.close(hdl, (websocketpp::close::status::value)0, "");
	}

	//清空缓存
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
		//新开启一个线程
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

	//等待线程结束
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
	g_server.run();//websock服务器在此循环，直到调用g_server.stop()函数结束循环
	//while (true)
	//{
	//	g_server.poll_one();
	//	Sleep(100);
	//}
}

