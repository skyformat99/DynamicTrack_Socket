#include "winSocketClient.h"


void SocketClient::CSocket_Init()
{
	versionRequired = MAKEWORD(1, 1);
    err = WSAStartup(versionRequired, &wsaData);//Э���İ汾��Ϣ
	//if (!err)    {
	//	printf("�ͻ���Ƕ�����Ѿ���!\n");
	//}
	//else{
	//	printf("ERROR:�ͻ��˵�Ƕ���ִ�ʧ��!\n");
 //    	}

    
}

void SocketClient::CSocket_Send_Connection()
{
	clientSocket = socket(AF_INET, SOCK_STREAM, 0);
	if (clientSocket == INVALID_SOCKET)
	{
		WSACleanup();
		printf("socket() faild!\n");
		return;
	}

	clientsock_in.sin_addr.S_un.S_addr = inet_addr("192.168.1.2");
	clientsock_in.sin_family = AF_INET;
	clientsock_in.sin_port = htons(6000);
	
	ret=connect(clientSocket, (SOCKADDR*)&clientsock_in, sizeof(SOCKADDR));//��ʼ����
	if (ret == SOCKET_ERROR)
	{
		printf("connect() failed!\n");
		closesocket(clientSocket); //�ر��׽���
		WSACleanup();
	}
	
}

void SocketClient::CSocket_Recv_Connection()
{
	clientSocket = socket(AF_INET, SOCK_STREAM, 0);
	if (clientSocket == INVALID_SOCKET)
	{
		WSACleanup();
		printf("socket() faild!\n");
		return;
	}
	clientsock_in.sin_addr.S_un.S_addr = htonl(INADDR_ANY);
	clientsock_in.sin_family = AF_INET;
	clientsock_in.sin_port = htons(6000);

	int n=bind(clientSocket, (SOCKADDR*)&clientsock_in, sizeof(clientsock_in));
	if (n!=0)  
	{
		perror("bind");
		closesocket(clientSocket);
		WSACleanup();
	}
	n=listen(clientSocket, 5);
	if (n != 0)
	{
		perror("listen");
		closesocket(clientSocket);
		WSACleanup();
	}
	int len = sizeof(SOCKADDR);
	conn = accept(clientSocket, (SOCKADDR*)&socketa, &len);
	if (conn <= 0)
	{
		perror("accept");
		closesocket(clientSocket);
		WSACleanup();
	}

}


void SocketClient::CSocket_Send(char *CSCommand)
{
	send(clientSocket, CSCommand, strlen(CSCommand) + 1, 0);
	printf("Send:%s\n",CSCommand);
	closesocket(clientSocket);
	WSACleanup();
}

void SocketClient::CSocket_Receive(char *CRCommand)
{
	recv(conn, CRCommand, 100, 0);
	printf("Recv : %s\n", CRCommand);
	closesocket(conn);
	closesocket(clientSocket);   
	WSACleanup();
}

void SocketClient::CSocket_SendFile(char *pathName)
{
	FILE *fq;
	char buf[MAX_LEN];
	if ((fq = fopen(pathName, "rb")) == NULL)
	{
		perror("File open");
		closesocket(clientSocket);
		exit(1);
	}

	while (!feof(fq))
	{
	
		int len = fread(buf, 1, MAX_LEN, fq);
		printf("%d\n", len);
		if (len != send(clientSocket, buf, len, 0))
		{
			printf("tt%d\n", len);

			perror("send");
			break;

		}
	

	}
	fclose(fq);
	closesocket(clientSocket);
	WSACleanup();
	closesocket(conn);
}

void SocketClient::CSocket_ReceiveFile(char *pathName)
{
	//�����ļ�
	FILE *fp;
	int recevMessage;
	char  recvBuffer[MAX_LEN];
	fp = fopen(pathName, "wb");  //�Զ����Ʒ�ʽ�򿪣��������ļ�
	if (fp == NULL)
	{
		printf("Cannot open file, press any key to exit!\n");
		system("pause");
		exit(0);
	}

	while ((recevMessage = recv(conn, recvBuffer, MAX_LEN, 0)) > 0){
		printf("%d\n", recevMessage);
		fwrite(recvBuffer, recevMessage, 1, fp);
	}

	fclose(fp);
	closesocket(clientSocket);
	WSACleanup();	
	closesocket(conn);
}

//void SocketClient::CSocket_Close()
//{
//	closesocket(clientSocket);
//	WSACleanup();
//}



