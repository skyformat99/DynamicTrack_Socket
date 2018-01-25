#pragma once
#include <winsock.h>
#include <stdio.h>
#pragma comment(lib,"ws2_32.lib")
#define MAX_LEN 512*1024 
#include <iostream>

using namespace std;

class SocketClient
{
public:
	SOCKET clientSocket,conn;
	SOCKADDR_IN clientsock_in,socketa;
	int ret, err;
	WORD versionRequired;
	WSADATA wsaData;
	
	//初始化socket
	void CSocket_Init();
	//连接socket并通信
	void CSocket_Send_Connection();
	void CSocket_Recv_Connection();
	void CSocket_Send(char *CSCommand);
	void CSocket_SendFile(char *pathName);
	void CSocket_Receive(char *CRCommand);
	void CSocket_ReceiveFile(char *pathName);

private:

};


