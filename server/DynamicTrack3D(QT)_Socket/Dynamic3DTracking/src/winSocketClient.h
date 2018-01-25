#pragma once
#include <WINSOCK2.H>
#include <stdio.h>
#pragma comment(lib,"ws2_32.lib")
#define MAX_LEN 512*1024 
#include <iostream>

using namespace std;

struct  StruCodePos
{
	int NumPic;
	int CodeNum;
	float X;
	float Y;
	float Z;
};


class SocketClient
{
public:
	SOCKET conn;
	SOCKET clientSocket;
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

	char temp[500];
	StruCodePos m_CodePos3D[15];

};




