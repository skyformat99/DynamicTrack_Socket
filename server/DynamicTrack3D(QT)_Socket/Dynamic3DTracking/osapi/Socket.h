
#ifndef _OSAPI_SOCKET_H
#define _OSAPI_SOCKET_H

#define _CRT_SECURE_NO_WARNINGS
#define _WINSOCK_DEPRECATED_NO_WARNINGS

#include <string>


#ifdef _WIN32
// windows �µ�socket����
#include <winsock2.h>
#include <ws2tcpip.h>

typedef SOCKET socket_t;

#define socket_open socket
#define socket_close closesocket
#define socket_ioctl  ioctlsocket
#else
// linux�µ�socket����
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <sys/time.h>
#include <fcntl.h>
typedef int socket_t;

#define socket_open socket
#define socket_close close
#define socket_ioctl  ioctl
#endif

#define socket_accept  accept
#define socket_bind      bind
#define socket_connect connect
#define socket_listen    listen
#define socket_send     send
#define socket_recv      recv
#define socket_sendto   sendto
#define socket_recvfrom recvfrom
#define socket_select    select
#define socket_setsockopt setsockopt
#define socket_getsockopt getsockopt

/* OS_Socket:
	*) ����һ�����࣬�û�����ֱ�Ӹ�ֵ����
	*) �û�Ӧ����ʽ�Ĺر�Close()�������������ﲻ���Զ��ر�
	*) �û�����ֱ�Ӳ���socket handle: hSock
*/

/* ����ֱ��ǿת��sockaddr_in�ṹ*/
class OS_SockAddr
{
public:
	explicit OS_SockAddr();
	explicit OS_SockAddr(const char* ip, unsigned short port);
	explicit OS_SockAddr(const char* ip); // Ĭ�϶˿�Ϊ0
	explicit OS_SockAddr(unsigned short port); // Ĭ��IPΪ0.0.0.0
	explicit OS_SockAddr(sockaddr_in addr);

	void SetIp(const char* ip);
	void SetIp(unsigned int ip);
	void SetPort(unsigned short port);

	std::string GetIp_str() const;
	unsigned int GetIp_n() const;
	unsigned short GetPort() const;

public:
	sockaddr_in iAddr;
};

class OS_Socket
{
public:
	OS_Socket(); 

	// ms=0ʱ������ʱ, ��λms, ms=1������Ϊ����������(1ms�ܿ����)
	int SetOpt_RecvTimeout(int ms); 
	int SetOpt_SendTimeout(int ms);
	int GetOpt_RecvTimeout(); 
	int GetOpt_SendTimeout();

	int Ioctl_SetBlockedIo(bool blocked);
	int SetOpt_ReuseAddr(bool reuse);

	int GetPeerAddr(OS_SockAddr& addr) const;
	int GetLocalAddr(OS_SockAddr& addr) const;

	// select����:��ѯ��д״̬
	// ����ֵ: >0����ʾ���Զ���д =0��ʾ��ʱ��<0��ʾsocket������
	int Select_ForReading(int timeout);
	int Select_ForWriting(int timeout);

public:
	socket_t hSock; // ����ֱ�ӷ������handle
};

class OS_TcpSocket : public OS_Socket
{
public:
	int Open(bool resue = false);
	int Open(const OS_SockAddr& addr , bool reuse = false);

	void Close();

	// ������
	int Listen(int backlog = 16);
	int Accept(OS_TcpSocket* peer);
	
	// �ͻ���
	int Connect(const OS_SockAddr& addr);

	// ���ͽ���
	int Send(const void* buf, int len);
	int Recv(void* buf, int len, int waitall=0);

};

class OS_UdpSocket : public OS_Socket
{
public:
	int Open(bool resue = false);
	int Open(const OS_SockAddr& addr,  bool reuse = false);

	void Close();

	int SendTo(const void* buf, int len, const OS_SockAddr&  peers);
	int RecvFrom( void* buf, int max_len, OS_SockAddr& peer);
};

class OS_McastSock : public OS_Socket
{
public:
	int Open(const char* mcast_ip, int port, const char* local_ip);
	void Close();

	/* ���Ͷಥʱ��ʹ����ͨUdpSock + �ಥ·�ɼ��� */
	//int SendTo(const void* buf, int len, const OS_SockAddr& peer);
	int RecvFrom( void* buf, int max_len, OS_SockAddr& peer);

private:
	ip_mreq m_McReq;
};

#endif


